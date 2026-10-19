# Third-party code and license scope

The root MIT license covers Oliver's original contributions, not the entire dependency tree. No vendor license or existing copyright header was removed during publication preparation. `.gitattributes` marks copied/vendor code separately from original application code, including vendor files stored under `Core/`.

## Original project code

The author confirms authorship of:

- `Core/Src/Kalman/kalman.c` and `Core/Inc/Kalman/kalman.h`: originally written by the author for VariometerV2, then reused in this course project.
- `Core/Src/tasks.c` and `Core/Inc/tasks.h`.
- `Core/Src/peakDetection.c` and `Core/Inc/peakDetection.h`.
- `Core/Src/WiFi/wifi_task.c` and `Core/Inc/WiFi/wifi_task.h`.
- Custom application additions in the `USER CODE` sections of `Core/Src/main.c`. The surrounding ST-generated code retains its original notices and terms.

The public Wi-Fi example, documentation and SVG artwork were added during repository preparation. No original authorship is claimed for the copied matrix/Cholesky code or adapted quaternion algorithms.

## Numerical support: kalman-clib / EJML

Files:

- `Core/Src/Kalman/matrix.c`, `Core/Inc/Kalman/matrix.h`
- `Core/Src/Kalman/cholesky.c`, `Core/Inc/Kalman/cholesky.h`
- `Core/Inc/Kalman/compiler.h`

The author recalls copying these routines from an external Kalman package. Source comparison identifies matching routines, compiler macros and EJML credits in [sunsided/kalman-clib](https://github.com/sunsided/kalman-clib). Its [pre-2025 revision `efb87f3`](https://github.com/sunsided/kalman-clib/tree/efb87f379e288b1a7ca4a78d852bf3143a29331f) was inspected during preparation. The exact revision originally copied is not known; these files include local changes and rearrangements.

The upstream [MIT notice](https://github.com/sunsided/kalman-clib/blob/efb87f379e288b1a7ca4a78d852bf3143a29331f/LICENSE.md), copyright 2014–2015 Markus Mayer and contributors, is preserved in [`LICENSES/kalman-clib-MIT.txt`](LICENSES/kalman-clib-MIT.txt). Existing references to the Efficient Java Matrix Library remain in the source. This dependency credit is distinct from the author's own Kalman application implementation.

## Quaternion orientation code

Files: `Core/Src/QuaternionFilter.c`, `Core/Inc/QuaternionFilter.h`.

The author confirms that portions were taken from an Arduino library but can no longer identify the original library or version. The module includes Mahony and Madgwick orientation routines and additional roll/pitch Kalman experiments.

Related implementations exist in the [SparkFun MPU-9250 library](https://github.com/sparkfun/SparkFun_MPU-9250_Breakout_Arduino_Library/blob/ea379ce8c930ef90f4ebc37799544d68108e3126/src/quaternionFilters.cpp). This is a comparison reference, not a claim that it was the exact source copied or that its terms cover this file. No license has been inferred merely from an algorithm name or the age of the code.

The adapted portions are excluded from the project's MIT grant. Their original provenance/license remains unresolved. Before representing the whole firmware as redistributable under known terms, identify the original source/license or replace those portions with an implementation whose provenance is documented.

## STM32, ARM and FreeRTOS

| Component | Files | Preserved license information |
| --- | --- | --- |
| STM32 HAL | `Drivers/STM32L4xx_HAL_Driver/**` | [`LICENSE.txt`](Drivers/STM32L4xx_HAL_Driver/LICENSE.txt) and per-file headers; package terms or stated BSD-3-Clause fallback |
| ARM CMSIS | `Drivers/CMSIS/**` | [`LICENSE.txt`](Drivers/CMSIS/LICENSE.txt), device-level [`LICENSE.txt`](Drivers/CMSIS/Device/ST/STM32L4xx/LICENSE.txt), [`License.md`](Drivers/CMSIS/Device/ST/STM32L4xx/License.md), and per-file notices; includes Apache-2.0 text |
| FreeRTOS / CMSIS-RTOS integration | `Middlewares/Third_Party/FreeRTOS/**` | Bundled [`Source/LICENSE`](Middlewares/Third_Party/FreeRTOS/Source/LICENSE) and file headers; kernel MIT, wrapper/port notices preserved |
| ST sensor drivers | `Core/Inc/lsm6dsl_reg.h`, `Core/Src/lsm6dsl_reg.c`, `Core/Inc/lps22hb.h`, `Core/Src/lps22hb.c` | BSD-3-Clause references and ST copyright notices in the files |
| ST QSPI BSP/component support | `Core/Inc/mx25r6435f.h`, `Core/Inc/stm32l475e_iot01_qspi.h`, `Core/Src/stm32l475e_iot01_qspi.c` | Original ST headers; some refer to a component-root license or AS-IS fallback |
| ST / Inventek Wi-Fi support | `Core/Inc/WiFi/es_wifi*.h`, `Core/Src/WiFi/es_wifi*.c`, `Core/Inc/WiFi/wifi.h`, `Core/Src/WiFi/wifi.c` | Original notices and component-license references retained; excludes the original application `wifi_task` files |
| Generated startup/configuration | `Core/Startup/**`, `Core/Src/main.c` scaffolding, `freertos.c`, `stm32l4xx*.c`, `system_stm32l4xx.c`, `syscalls.c`, `sysmem.c`, associated headers and linker scripts | Original ST/generated notices retained; not relicensed by the root MIT grant |

Some copied ST files reference a component-root `LICENSE` which was not included at that location in the original checkout. Adding the project's MIT license does not supply ST's missing license or override those notices. The references are documented here instead of inventing terms.
