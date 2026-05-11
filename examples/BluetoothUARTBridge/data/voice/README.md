# Bluetooth voice files

Put 16-bit PCM WAV files here and upload this `data` folder to ESP32 SPIFFS.

Required format:
- WAV, PCM, 16-bit
- 44100 Hz
- mono or stereo

Segment filenames used by `BluetoothUARTBridge.ino`:

- `left.wav`
- `center.wav`
- `right.wav`
- `caution.wav`
- `danger.wav`
- `safe.wav` optional
- `unknown.wav`
- `object.wav`

Common class filenames:

- `person.wav`
- `bicycle.wav`
- `car.wav`
- `motorcycle.wav`
- `bus.wav`
- `truck.wav`
- `cat.wav`
- `dog.wav`
- `chair.wav`

When ESP32 receives a warning packet, it plays:

`direction + class/object/unknown + severity`

If any segment is missing, the sketch logs `[VOICE] missing ...` and falls back to
the existing beep pattern when no voice segment was queued.
