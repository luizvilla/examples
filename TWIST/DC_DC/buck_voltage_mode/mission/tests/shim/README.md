# Host stand-ins

Test-only headers that let `control.cpp` and OwnTech's `control_library` compile on a PC. They are never compiled into the firmware. Copied from the uSolarVerter firmware's `tests/common/shim/` (Core2).

| File | Stands in for |
|---|---|
| `arm_math.h`, `arm_math_types.h` | The CMSIS-DSP symbols the control library uses (`float32_t`, …) |
| `zephyr/logging/log.h` | Zephyr logging; every `LOG_*` macro expands to nothing |
