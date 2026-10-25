# IR RAW Recorder (Flipper Zero, Momentum firmware)

Records infrared continuously like Sub-GHz "Read RAW", saves to a `.ir` file, and replays it.

## Controls
Record screen:
- OK: start recording / stop recording / send (replay) once recorded
- Left: erase the recording
- Right: save (asks for a name)
- Up/Down: change the carrier frequency used for replay (30, 33, 36, 38, 40, 56 kHz)
- Back: stop recording, or return to the menu (asks before discarding unsaved data)

Menu > Saved Signals opens any `.ir` file and loads its first RAW signal for replay.

Files go to `/ext/infrared/raw_rec/` in the standard IR file format.

## Build
With uFBT pointed at the Momentum SDK:

    pip install --upgrade ufbt
    ufbt update --index-url=https://up.momentum-fw.dev/firmware/directory.json
    cd ir_raw_recorder
    ufbt            # builds dist/ir_raw_recorder.fap
    ufbt launch     # builds, installs and runs on a USB-connected Flipper

Or copy the folder into `applications_user/` of the Momentum firmware repo and run
`./fbt launch APPSRC=ir_raw_recorder`.

## Notes
- The IR receiver demodulates the carrier, so the original frequency can't be measured.
  38 kHz is the default; change it with Up/Down if a device doesn't respond.
- The IR worker ends a burst after 150 ms of silence. Pauses shorter than that stay
  inside one burst exactly; longer pauses between bursts are measured with ~1 ms
  accuracy and capped at 1.5 s.
- Up to 8192 timings per recording. The stock Infrared app only opens raw signals
  of 1024 timings or fewer, so longer recordings should be replayed from this app.
