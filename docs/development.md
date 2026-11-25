# Local engine development

Use the extracted US PS2 disc (`SLUS-20579`). Keep the ISO, extracted files, and generated screenshots and traces outside the Git checkout.

## Build and smoke test

From the engine checkout, configure with CMake and Ninja. On macOS, use the command-line tools and make CMake, Ninja, `pkg-config`, and FFmpeg available in your shell. CMake fetches the pinned SDL3 source and uses the system OpenGL framework.

```sh
cmake -S . -B build -G Ninja
cmake --build build --parallel
sh tools/smoke-disc.sh build /path/to/extracted-disc
```

The smoke script validates disc data and checks startup, simulation, and screenshot output for the main menu, Skyrail Arena, The Exchange, and Paris Prelude. It prints the directory with the logs and BMPs. SDL still needs a graphical session to create its hidden OpenGL window. On Linux, `xvfb-run -a` can supply one.

## Check the live launch path

On macOS, run `build/nightfire.app/Contents/MacOS/nightfire /path/to/extracted-disc --page 0x40000002`. On Linux, use `build/nightfire` with the same arguments. From the main menu pick **NightFire → Default → Operative → The Exchange**. Confirm that the castle and HUD appear and the view responds to movement. This check caught the campaign's missing buffer swap. A hidden capture reads the back buffer, so it cannot verify that the live window presents the frame.

For regressions, record the disc region, build commit, launch steps, and first visible symptom. Use `nfgame --inputs` and `--trace` or `--mp-trace` for repeatable input and state comparisons. See [the PCSX2 reference workflow](oracle.md) for comparing changes with the original disc. Its automated pad and window controls assume Linux. A local smoke pass establishes that the tested cases run. Completing missions, matching PS2 behavior, and checking other platforms each require their own tests.
