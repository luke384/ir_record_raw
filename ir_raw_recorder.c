/*
 * IR RAW Recorder for Flipper Zero (Momentum / OFW-based firmware)
 *
 * Works like Sub-GHz "Read RAW", but for infrared:
 *   - Press OK to start recording. Every IR burst the receiver sees is
 *     appended to one continuous recording, with the real pause between
 *     bursts preserved (approximately).
 *   - Press OK again to stop.
 *   - Then: OK = Send (replay), Left = Erase, Right = Save,
 *           Up/Down = change carrier frequency used for replay.
 *   - "Saved Signals" in the menu opens any .ir file and replays the
 *     first RAW signal in it.
 *
 * Files are written in the standard Flipper "IR signals file" format to
 * /ext/infrared/raw_rec/, so short recordings (<=1024 timings) can also be
 * opened by the stock Infrared app.
 */

#include <furi.h>
#include <furi_hal.h>
#include <gui/gui.h>
#include <gui/view.h>
#include <gui/view_dispatcher.h>
#include <gui/elements.h>
#include <gui/modules/submenu.h>
#include <gui/modules/text_input.h>
#include <dialogs/dialogs.h>
#include <storage/storage.h>
#include <flipper_format/flipper_format.h>
#include <notification/notification_messages.h>
#include <toolbox/path.h>
#include <infrared_worker.h>
#include <infrared_transmit.h>

#define TAG "IrRawRec"

#define IR_ROOT_FOLDER  EXT_PATH("infrared")
#define IR_RAW_FOLDER   EXT_PATH("infrared/raw_rec")
#define IR_RAW_EXT      ".ir"
#define IR_FILE_TYPE    "IR signals file"
#define IR_FILE_VERSION 1

/* Total timings (marks + spaces) one recording can hold. 8192 * 4 B = 32 KB */
#define MAX_TIMINGS 8192
/* Timings kept for the on-screen waveform preview */
#define SNAP_MAX 512
/* The IR worker ends a burst after this much silence (OFW/Momentum: 150 ms) */
#define RX_SILENCE_US 150000
/* Longest pause stored between two bursts */
#define MAX_GAP_US 1500000
/* Stock Infrared app refuses raw signals longer than this */
#define STOCK_IR_MAX_TIMINGS 1024

#define DEFAULT_FREQ 38000
#define DEFAULT_DUTY 0.33f
#define NAME_LEN     40

static const uint32_t freq_presets[] = {30000, 33000, 36000, 38000, 40000, 56000};

typedef enum {
    ViewIdSubmenu,
    ViewIdRecord,
    ViewIdTextInput,
} ViewId;

typedef enum {
    MenuIndexRecord,
    MenuIndexLoad,
} MenuIndex;

typedef enum {
    EventOk = 1,
    EventBack,
    EventErase,
    EventSave,
    EventFreqUp,
    EventFreqDown,
    EventNameEntered,
    EventBufferFull,
} CustomEvent;

typedef enum {
    RecStateIdle,
    RecStateRecording,
    RecStateReady,
    RecStateSending,
} RecState;

typedef struct {
    RecState state;
    bool saved;
    uint32_t freq;
    uint32_t count;
    uint32_t bursts;
    char name[NAME_LEN];
    uint32_t snap_cnt;
    uint32_t snap[SNAP_MAX];
} RecordModel;

typedef struct {
    Gui* gui;
    ViewDispatcher* view_dispatcher;
    Submenu* submenu;
    View* record_view;
    TextInput* text_input;
    DialogsApp* dialogs;
    Storage* storage;
    NotificationApp* notifications;
    InfraredWorker* worker;
    FuriMutex* mutex;

    uint32_t* timings;
    uint32_t count;
    uint32_t bursts;
    int64_t prev_end_us;
    volatile bool overflow_flagged;

    uint32_t freq;
    float duty;
    char name[NAME_LEN];
    char name_input[NAME_LEN];
    FuriString* path;
} IrRawApp;

/* ---------------------------------------------------------------- helpers */

static int64_t now_us(void) {
    return (int64_t)furi_get_tick() * 1000000LL / (int64_t)furi_kernel_get_tick_frequency();
}

static uint32_t freq_step(uint32_t cur, bool up) {
    const size_t n = COUNT_OF(freq_presets);
    if(up) {
        for(size_t i = 0; i < n; i++) {
            if(freq_presets[i] > cur) return freq_presets[i];
        }
        return freq_presets[0];
    } else {
        for(size_t i = n; i > 0; i--) {
            if(freq_presets[i - 1] < cur) return freq_presets[i - 1];
        }
        return freq_presets[n - 1];
    }
}

/* Length of the first burst: stops at the first long space */
static uint32_t first_burst_len(const uint32_t* t, uint32_t n) {
    for(uint32_t i = 1; i < n; i += 2) {
        if(t[i] >= RX_SILENCE_US / 2) return i;
    }
    return n;
}

static uint32_t count_bursts(const uint32_t* t, uint32_t n) {
    if(n == 0) return 0;
    uint32_t b = 1;
    for(uint32_t i = 1; i < n; i += 2) {
        if(t[i] >= RX_SILENCE_US / 2) b++;
    }
    return b;
}

static void make_default_name(IrRawApp* app) {
    DateTime dt;
    furi_hal_rtc_get_datetime(&dt);
    snprintf(
        app->name,
        NAME_LEN,
        "RAW_%04u%02u%02u_%02u%02u%02u",
        dt.year,
        dt.month,
        dt.day,
        dt.hour,
        dt.minute,
        dt.second);
}

static void model_sync(IrRawApp* app, RecState state, bool saved) {
    with_view_model(
        app->record_view,
        RecordModel * m,
        {
            m->state = state;
            m->saved = saved;
            m->freq = app->freq;
            m->count = app->count;
            m->bursts = app->bursts;
            snprintf(m->name, NAME_LEN, "%s", app->name);
        },
        true);
}

static void model_set_snapshot(IrRawApp* app, const uint32_t* t, uint32_t n) {
    if(n > SNAP_MAX) n = SNAP_MAX;
    with_view_model(
        app->record_view,
        RecordModel * m,
        {
            memcpy(m->snap, t, n * sizeof(uint32_t));
            m->snap_cnt = n;
        },
        true);
}

static RecState model_get_state(IrRawApp* app, bool* saved) {
    RecState state = RecStateIdle;
    bool s = false;
    with_view_model(
        app->record_view,
        RecordModel * m,
        {
            state = m->state;
            s = m->saved;
        },
        false);
    if(saved) *saved = s;
    return state;
}

static void show_message(IrRawApp* app, const char* header, const char* text) {
    DialogMessage* msg = dialog_message_alloc();
    dialog_message_set_header(msg, header, 64, 2, AlignCenter, AlignTop);
    dialog_message_set_text(msg, text, 64, 32, AlignCenter, AlignCenter);
    dialog_message_set_buttons(msg, NULL, "OK", NULL);
    dialog_message_show(app->dialogs, msg);
    dialog_message_free(msg);
}

static bool confirm(IrRawApp* app, const char* header, const char* text, const char* yes) {
    DialogMessage* msg = dialog_message_alloc();
    dialog_message_set_header(msg, header, 64, 2, AlignCenter, AlignTop);
    dialog_message_set_text(msg, text, 64, 30, AlignCenter, AlignCenter);
    dialog_message_set_buttons(msg, "Cancel", NULL, yes);
    DialogMessageButton res = dialog_message_show(app->dialogs, msg);
    dialog_message_free(msg);
    return res == DialogMessageButtonRight;
}

/* ------------------------------------------------------------- recording */

/* Runs in the IR worker thread every time a burst ends (after ~150 ms silence) */
static void rx_callback(void* context, InfraredWorkerSignal* signal) {
    IrRawApp* app = context;
    const uint32_t* t = NULL;
    size_t n = 0;
    infrared_worker_get_raw_signal(signal, &t, &n);
    if(!t || n == 0) return;

    int64_t burst_us = 0;
    for(size_t i = 0; i < n; i++) burst_us += t[i];

    /* The callback fires RX_SILENCE_US after the last edge of the burst */
    int64_t end_us = now_us() - RX_SILENCE_US;
    int64_t start_us = end_us - burst_us;

    bool overflow = false;
    furi_mutex_acquire(app->mutex, FuriWaitForever);

    if(app->count > 0) {
        int64_t gap = start_us - app->prev_end_us;
        if(gap < RX_SILENCE_US) gap = RX_SILENCE_US;
        if(gap > MAX_GAP_US) gap = MAX_GAP_US;

        if(app->count % 2 == 1) {
            /* Recording ends with a mark: add the pause as a space */
            if(app->count + 1 + 1 > MAX_TIMINGS) {
                overflow = true;
            } else {
                app->timings[app->count++] = (uint32_t)gap;
            }
        } else {
            /* Recording ends with a space: extend it */
            app->timings[app->count - 1] += (uint32_t)gap;
        }
    }

    if(!overflow) {
        uint32_t space = MAX_TIMINGS - app->count;
        uint32_t copy = (n > space) ? space : (uint32_t)n;
        memcpy(&app->timings[app->count], t, copy * sizeof(uint32_t));
        app->count += copy;
        app->bursts++;
        if(copy < n) overflow = true;
    }
    app->prev_end_us = end_us;

    uint32_t count = app->count;
    uint32_t bursts = app->bursts;
    furi_mutex_release(app->mutex);

    uint32_t snap_n = (n > SNAP_MAX) ? SNAP_MAX : (uint32_t)n;
    with_view_model(
        app->record_view,
        RecordModel * m,
        {
            memcpy(m->snap, t, snap_n * sizeof(uint32_t));
            m->snap_cnt = snap_n;
            m->count = count;
            m->bursts = bursts;
        },
        true);

    /* Can't stop the worker from its own thread, so ask the main thread */
    if(overflow && !app->overflow_flagged) {
        app->overflow_flagged = true;
        view_dispatcher_send_custom_event(app->view_dispatcher, EventBufferFull);
    }
}

static void reset_data(IrRawApp* app) {
    furi_mutex_acquire(app->mutex, FuriWaitForever);
    app->count = 0;
    app->bursts = 0;
    app->prev_end_us = 0;
    furi_mutex_release(app->mutex);

    app->overflow_flagged = false;
    app->duty = DEFAULT_DUTY;
    app->name[0] = '\0';
    with_view_model(app->record_view, RecordModel * m, { m->snap_cnt = 0; }, false);
    model_sync(app, RecStateIdle, false);
}

static void rec_start(IrRawApp* app) {
    reset_data(app);
    model_sync(app, RecStateRecording, false);
    infrared_worker_rx_start(app->worker);
    FURI_LOG_I(TAG, "Recording started");
}

static void rec_stop(IrRawApp* app) {
    infrared_worker_rx_stop(app->worker);
    FURI_LOG_I(TAG, "Recording stopped, %lu timings", (unsigned long)app->count);
    if(app->count > 0) {
        make_default_name(app);
        model_sync(app, RecStateReady, false);
    } else {
        model_sync(app, RecStateIdle, false);
    }
}

static void send_signal(IrRawApp* app) {
    if(app->count == 0) return;

    bool saved = false;
    model_get_state(app, &saved);
    model_sync(app, RecStateSending, saved);
    notification_message(app->notifications, &sequence_blink_start_magenta);

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    /* Blocking: returns when the whole recording has been transmitted */
    infrared_send_raw_ext(app->timings, app->count, true, app->freq, app->duty);
    furi_mutex_release(app->mutex);

    notification_message(app->notifications, &sequence_blink_stop);
    model_sync(app, RecStateReady, saved);
}

/* ------------------------------------------------------------ save/load */

static bool save_signal(IrRawApp* app) {
    storage_simply_mkdir(app->storage, IR_ROOT_FOLDER);
    storage_simply_mkdir(app->storage, IR_RAW_FOLDER);
    furi_string_printf(app->path, "%s/%s%s", IR_RAW_FOLDER, app->name, IR_RAW_EXT);

    FlipperFormat* ff = flipper_format_file_alloc(app->storage);
    bool ok = false;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    do {
        if(!flipper_format_file_open_always(ff, furi_string_get_cstr(app->path))) break;
        if(!flipper_format_write_header_cstr(ff, IR_FILE_TYPE, IR_FILE_VERSION)) break;
        if(!flipper_format_write_comment_cstr(ff, "Recorded with IR RAW Recorder")) break;
        if(!flipper_format_write_string_cstr(ff, "name", app->name)) break;
        if(!flipper_format_write_string_cstr(ff, "type", "raw")) break;
        if(!flipper_format_write_uint32(ff, "frequency", &app->freq, 1)) break;
        if(!flipper_format_write_float(ff, "duty_cycle", &app->duty, 1)) break;
        if(!flipper_format_write_uint32(ff, "data", app->timings, app->count)) break;
        ok = true;
    } while(false);
    furi_mutex_release(app->mutex);

    flipper_format_free(ff);
    FURI_LOG_I(TAG, "Save %s: %s", furi_string_get_cstr(app->path), ok ? "ok" : "failed");
    return ok;
}

static bool load_signal(IrRawApp* app) {
    DialogsFileBrowserOptions opts;
    dialog_file_browser_set_basic_options(&opts, IR_RAW_EXT, NULL);
    opts.base_path = IR_ROOT_FOLDER;

    furi_string_set(app->path, IR_RAW_FOLDER);
    if(!dialog_file_browser_show(app->dialogs, app->path, app->path, &opts)) return false;

    reset_data(app);

    FlipperFormat* ff = flipper_format_file_alloc(app->storage);
    FuriString* tmp = furi_string_alloc();
    FuriString* type = furi_string_alloc();
    const char* error = "Could not read file";
    bool ok = false;
    uint32_t version = 0;
    uint32_t freq = DEFAULT_FREQ;
    uint32_t cnt = 0;
    float duty = DEFAULT_DUTY;

    furi_mutex_acquire(app->mutex, FuriWaitForever);
    do {
        if(!flipper_format_file_open_existing(ff, furi_string_get_cstr(app->path))) break;
        if(!flipper_format_read_header(ff, tmp, &version)) break;
        if(furi_string_cmp_str(tmp, IR_FILE_TYPE) != 0) {
            error = "Not an IR file";
            break;
        }

        error = "No RAW signal\nin this file";
        /* Walk the signals and take the first RAW one */
        while(flipper_format_read_string(ff, "name", tmp)) {
            if(!flipper_format_read_string(ff, "type", type)) break;
            if(furi_string_cmp_str(type, "raw") != 0) continue;

            error = "Corrupted RAW signal";
            if(!flipper_format_read_uint32(ff, "frequency", &freq, 1)) break;
            if(!flipper_format_read_float(ff, "duty_cycle", &duty, 1)) break;
            if(!flipper_format_get_value_count(ff, "data", &cnt)) break;
            if(cnt == 0) break;
            if(cnt > MAX_TIMINGS) {
                error = "Signal too long";
                break;
            }
            if(!flipper_format_read_uint32(ff, "data", app->timings, cnt)) break;
            ok = true;
            break;
        }
    } while(false);

    if(ok) {
        app->count = cnt;
        app->bursts = count_bursts(app->timings, cnt);
        app->freq = freq;
        app->duty = duty;
    }
    furi_mutex_release(app->mutex);

    flipper_format_free(ff);
    furi_string_free(type);

    if(ok) {
        path_extract_filename_no_ext(furi_string_get_cstr(app->path), tmp);
        snprintf(app->name, NAME_LEN, "%s", furi_string_get_cstr(tmp));
        model_set_snapshot(app, app->timings, first_burst_len(app->timings, app->count));
        model_sync(app, RecStateReady, true);
    } else {
        show_message(app, "Load failed", error);
    }
    furi_string_free(tmp);
    return ok;
}

/* --------------------------------------------------------- record view */

static void draw_wave(Canvas* canvas, const uint32_t* t, uint32_t n) {
    uint64_t total = 0;
    for(uint32_t i = 0; i < n; i++) total += t[i];
    if(total == 0) return;

    const int x0 = 2, w = 124, y_hi = 17, y_lo = 36;
    int x = x0;
    int prev_y = y_lo;
    uint64_t acc = 0;
    bool mark = true;

    for(uint32_t i = 0; i < n; i++) {
        acc += t[i];
        int x_end = x0 + (int)(acc * (uint64_t)w / total);
        int y = mark ? y_hi : y_lo;
        if(y != prev_y) canvas_draw_line(canvas, x, prev_y, x, y);
        canvas_draw_line(canvas, x, y, x_end, y);
        x = x_end;
        prev_y = y;
        mark = !mark;
    }
    if(prev_y != y_lo) canvas_draw_line(canvas, x, prev_y, x, y_lo);
}

static void record_draw_callback(Canvas* canvas, void* model) {
    RecordModel* m = model;
    char buf[48];

    canvas_clear(canvas);
    canvas_set_color(canvas, ColorBlack);

    /* Header: state + carrier frequency */
    static const char* state_names[] = {"IR RAW", "REC", "Ready", "Sending"};
    canvas_set_font(canvas, FontPrimary);
    canvas_draw_str(canvas, 2, 10, state_names[m->state]);

    canvas_set_font(canvas, FontSecondary);
    if(m->freq % 1000 == 0) {
        snprintf(buf, sizeof(buf), "%lukHz", (unsigned long)(m->freq / 1000));
    } else {
        snprintf(
            buf,
            sizeof(buf),
            "%lu.%lukHz",
            (unsigned long)(m->freq / 1000),
            (unsigned long)((m->freq % 1000) / 100));
    }
    canvas_draw_str_aligned(canvas, 126, 2, AlignRight, AlignTop, buf);

    /* Waveform box */
    canvas_draw_frame(canvas, 0, 13, 128, 28);
    if(m->state == RecStateRecording) {
        canvas_draw_disc(canvas, 122, 18, 2);
    }

    if(m->state == RecStateSending) {
        canvas_draw_str_aligned(canvas, 64, 27, AlignCenter, AlignCenter, "Sending...");
    } else if(m->snap_cnt > 0) {
        draw_wave(canvas, m->snap, m->snap_cnt);
    } else if(m->state == RecStateRecording) {
        canvas_draw_str_aligned(canvas, 64, 27, AlignCenter, AlignCenter, "Waiting for IR...");
    } else {
        canvas_draw_str_aligned(canvas, 64, 27, AlignCenter, AlignCenter, "Press OK to record");
    }

    /* Status line */
    if(m->state == RecStateReady && m->saved) {
        canvas_draw_str_aligned(canvas, 64, 47, AlignCenter, AlignCenter, m->name);
    } else {
        snprintf(
            buf,
            sizeof(buf),
            "%lu burst%s  %lu/%u",
            (unsigned long)m->bursts,
            m->bursts == 1 ? "" : "s",
            (unsigned long)m->count,
            MAX_TIMINGS);
        canvas_draw_str_aligned(canvas, 64, 47, AlignCenter, AlignCenter, buf);
    }

    /* Button hints */
    switch(m->state) {
    case RecStateIdle:
        elements_button_center(canvas, "REC");
        break;
    case RecStateRecording:
        elements_button_center(canvas, "Stop");
        break;
    case RecStateReady:
        elements_button_left(canvas, "Erase");
        elements_button_center(canvas, "Send");
        elements_button_right(canvas, "Save");
        break;
    case RecStateSending:
    default:
        break;
    }
}

static bool record_input_callback(InputEvent* event, void* context) {
    IrRawApp* app = context;

    /* Always consume Back so the view dispatcher doesn't navigate on its own */
    if(event->key == InputKeyBack) {
        if(event->type == InputTypeShort) {
            view_dispatcher_send_custom_event(app->view_dispatcher, EventBack);
        }
        return true;
    }
    if(event->type != InputTypeShort) return false;

    switch(event->key) {
    case InputKeyOk:
        view_dispatcher_send_custom_event(app->view_dispatcher, EventOk);
        break;
    case InputKeyLeft:
        view_dispatcher_send_custom_event(app->view_dispatcher, EventErase);
        break;
    case InputKeyRight:
        view_dispatcher_send_custom_event(app->view_dispatcher, EventSave);
        break;
    case InputKeyUp:
        view_dispatcher_send_custom_event(app->view_dispatcher, EventFreqUp);
        break;
    case InputKeyDown:
        view_dispatcher_send_custom_event(app->view_dispatcher, EventFreqDown);
        break;
    default:
        return false;
    }
    return true;
}

/* ------------------------------------------------------ dispatcher glue */

static void text_input_done_callback(void* context) {
    IrRawApp* app = context;
    view_dispatcher_send_custom_event(app->view_dispatcher, EventNameEntered);
}

static uint32_t back_to_record_callback(void* context) {
    UNUSED(context);
    return ViewIdRecord;
}

static bool navigation_callback(void* context) {
    UNUSED(context);
    /* Only reached from the main menu: returning false exits the app */
    return false;
}

static bool custom_event_callback(void* context, uint32_t event) {
    IrRawApp* app = context;
    bool saved = false;
    RecState state = model_get_state(app, &saved);

    switch(event) {
    case EventOk:
        if(state == RecStateIdle) {
            rec_start(app);
        } else if(state == RecStateRecording) {
            rec_stop(app);
        } else if(state == RecStateReady) {
            send_signal(app);
        }
        break;

    case EventBufferFull:
        if(state == RecStateRecording) {
            rec_stop(app);
            show_message(app, "Buffer full", "Recording stopped");
        }
        break;

    case EventErase:
        if(state == RecStateReady) reset_data(app);
        break;

    case EventSave:
        if(state == RecStateReady) {
            if(app->name[0] == '\0') make_default_name(app);
            snprintf(app->name_input, NAME_LEN, "%s", app->name);
            text_input_set_header_text(app->text_input, "Name the signal");
            text_input_set_result_callback(
                app->text_input,
                text_input_done_callback,
                app,
                app->name_input,
                NAME_LEN,
                false);
            view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdTextInput);
        }
        break;

    case EventNameEntered:
        if(app->name_input[0] != '\0') {
            snprintf(app->name, NAME_LEN, "%s", app->name_input);
        }
        view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdRecord);
        if(save_signal(app)) {
            model_sync(app, RecStateReady, true);
            notification_message(app->notifications, &sequence_success);
            if(app->count > STOCK_IR_MAX_TIMINGS) {
                show_message(
                    app, "Saved", "Long recording: the stock\nIR app may not open it.\nReplay it from here.");
            }
        } else {
            show_message(app, "Error", "Could not save file");
        }
        break;

    case EventFreqUp:
    case EventFreqDown:
        if(state == RecStateIdle || state == RecStateReady) {
            app->freq = freq_step(app->freq, event == EventFreqUp);
            /* Changing the carrier makes a saved file out of date */
            model_sync(app, state, false);
        }
        break;

    case EventBack:
        if(state == RecStateRecording) {
            rec_stop(app);
        } else if(state == RecStateReady && !saved) {
            if(confirm(app, "Discard recording?", "Unsaved data\nwill be lost", "Discard")) {
                reset_data(app);
                view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdSubmenu);
            }
        } else if(state != RecStateSending) {
            view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdSubmenu);
        }
        break;

    default:
        return false;
    }
    return true;
}

static void submenu_callback(void* context, uint32_t index) {
    IrRawApp* app = context;
    if(index == MenuIndexRecord) {
        reset_data(app);
        view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdRecord);
    } else if(index == MenuIndexLoad) {
        if(load_signal(app)) {
            view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdRecord);
        }
    }
}

/* --------------------------------------------------------- alloc / free */

static IrRawApp* app_alloc(void) {
    IrRawApp* app = malloc(sizeof(IrRawApp));
    memset(app, 0, sizeof(IrRawApp));

    app->timings = malloc(MAX_TIMINGS * sizeof(uint32_t));
    app->mutex = furi_mutex_alloc(FuriMutexTypeNormal);
    app->path = furi_string_alloc();
    app->freq = DEFAULT_FREQ;
    app->duty = DEFAULT_DUTY;

    app->gui = furi_record_open(RECORD_GUI);
    app->dialogs = furi_record_open(RECORD_DIALOGS);
    app->storage = furi_record_open(RECORD_STORAGE);
    app->notifications = furi_record_open(RECORD_NOTIFICATION);

    storage_simply_mkdir(app->storage, IR_ROOT_FOLDER);
    storage_simply_mkdir(app->storage, IR_RAW_FOLDER);

    app->worker = infrared_worker_alloc();
    infrared_worker_rx_enable_signal_decoding(app->worker, false);
    infrared_worker_rx_enable_blink_on_receiving(app->worker, true);
    infrared_worker_rx_set_received_signal_callback(app->worker, rx_callback, app);

    app->view_dispatcher = view_dispatcher_alloc();
    view_dispatcher_set_event_callback_context(app->view_dispatcher, app);
    view_dispatcher_set_custom_event_callback(app->view_dispatcher, custom_event_callback);
    view_dispatcher_set_navigation_event_callback(app->view_dispatcher, navigation_callback);
    view_dispatcher_attach_to_gui(app->view_dispatcher, app->gui, ViewDispatcherTypeFullscreen);

    app->submenu = submenu_alloc();
    submenu_set_header(app->submenu, "IR RAW Recorder");
    submenu_add_item(app->submenu, "Record RAW", MenuIndexRecord, submenu_callback, app);
    submenu_add_item(app->submenu, "Saved Signals", MenuIndexLoad, submenu_callback, app);
    view_dispatcher_add_view(app->view_dispatcher, ViewIdSubmenu, submenu_get_view(app->submenu));

    app->record_view = view_alloc();
    view_set_context(app->record_view, app);
    view_allocate_model(app->record_view, ViewModelTypeLocking, sizeof(RecordModel));
    view_set_draw_callback(app->record_view, record_draw_callback);
    view_set_input_callback(app->record_view, record_input_callback);
    view_dispatcher_add_view(app->view_dispatcher, ViewIdRecord, app->record_view);

    app->text_input = text_input_alloc();
    view_set_previous_callback(text_input_get_view(app->text_input), back_to_record_callback);
    view_dispatcher_add_view(
        app->view_dispatcher, ViewIdTextInput, text_input_get_view(app->text_input));

    model_sync(app, RecStateIdle, false);
    return app;
}

static void app_free(IrRawApp* app) {
    view_dispatcher_remove_view(app->view_dispatcher, ViewIdSubmenu);
    view_dispatcher_remove_view(app->view_dispatcher, ViewIdRecord);
    view_dispatcher_remove_view(app->view_dispatcher, ViewIdTextInput);
    submenu_free(app->submenu);
    view_free(app->record_view);
    text_input_free(app->text_input);
    view_dispatcher_free(app->view_dispatcher);

    infrared_worker_free(app->worker);

    furi_record_close(RECORD_NOTIFICATION);
    furi_record_close(RECORD_STORAGE);
    furi_record_close(RECORD_DIALOGS);
    furi_record_close(RECORD_GUI);

    furi_string_free(app->path);
    furi_mutex_free(app->mutex);
    free(app->timings);
    free(app);
}

int32_t ir_raw_recorder_app(void* p) {
    UNUSED(p);
    IrRawApp* app = app_alloc();
    view_dispatcher_switch_to_view(app->view_dispatcher, ViewIdSubmenu);
    view_dispatcher_run(app->view_dispatcher);
    app_free(app);
    return 0;
}
