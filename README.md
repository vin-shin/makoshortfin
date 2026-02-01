Mako Shortfin - Build instructions

Quick build (PowerShell) - tested on Windows with Ninja and GNU Arm toolchain:

1. Ensure toolchain and dependencies are installed (GNU Arm Embedded Toolchain).
2. From project root (PowerShell):

```powershell
git pull
cmake -S . -B build/Debug -G Ninja -DCMAKE_BUILD_TYPE=Debug -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake
cmake --build build/Debug --config Debug
```

Notes:
- The project was built and tested with GNU Arm toolchain (arm-none-eabi) GCC 10.3.1.
- A minor linker-script tweak was made for compatibility with GCC 10: some GCC11-only `(READONLY)` section attributes were removed from `STM32G474XX_FLASH.ld`. If you upgrade to GCC 11+, you can reintroduce them if desired.
- If you switch machines or change source locations, delete the `build/` directory and re-run the `cmake` configure command above to regenerate CMake cache.

If you want me to add a small PowerShell build wrapper or CI config, tell me which CI provider you use (GitHub Actions, Azure Pipelines, etc.).
