# DaisySound

Audio coprocessor firmware for **Daisy**, a homebrew personal computer.
Runs on an Arduino Uno (ATmega328) and synthesizes three voices entirely
in software from a high-rate timer interrupt.

DaisyOS sends note, waveform, envelope, and sequencer commands over a
hardware UART; DaisySound generates the audio.

## Role in the system

| Unit       | MCU         | Role                                        |
|------------|-------------|---------------------------------------------|
| [DaisyOS](https://github.com/pineconecomputer/DaisyOS) | SAM3X (Due) | Brain. BASIC, editor, terminal, keyboard |
| [DaisyVideo](https://github.com/pineconecomputer/DaisyVideo) | ATmega2560 | Text + graphics, composite video out |
| DaisySound | ATmega328   | This repo. 3-voice synthesizer              |

DaisyOS drives this board over its `Serial3` at 115200 baud using the
`audio_messages` framing described in
[DaisyOS's ARCHITECTURE.md](https://github.com/pineconecomputer/DaisyOS/blob/main/ARCHITECTURE.md).

## Building

Requires [PlatformIO](https://platformio.org/install) and an Arduino Uno.
PlatformIO fetches the AVR toolchain on first build; there are no library
dependencies.

```sh
git clone https://github.com/pineconecomputer/DaisySound.git
cd DaisySound
pio run                  # build
pio run -t upload        # build and flash
```

Current footprint: ~7.4 KB flash (23% of 31.5 KB), ~1.4 KB RAM (70% of
2 KB).

## Layout

```
include/
  buffer.h        ring buffer for incoming serial commands
  timer.h         timer helpers
src/
  main.cpp        synthesis ISR, voice state, command loop
  buffer.cpp
  timer.cpp
```

## Code style

Formatted to the [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
via the checked-in `.clang-format`:

```sh
clang-format -i $(find include src -name '*.cpp' -o -name '*.h')
```

Include sorting is disabled, since include order matters in Arduino
sources.

## License

Licensed under the **GNU General Public License, version 3**. See
[LICENSE](LICENSE) for the full text.

    Copyright (C) 2026 Joe Cassara
