# RoboMaster Basic Framework - AI Coding Agent Instructions

## Architecture (3-Layer, Strict Separation)
```
APP (application/)  → Module (modules/) → BSP (bsp/) → HAL (HAL_N_Middlewares/)
```
**Never cross-reference layers**: APP→Module→BSP only. Entry point: `HAL_N_Middlewares/Src/main.c`

## First Steps: `application/robot_def.h`
Controls board mode (`ONE_BOARD`/`GIMBAL_BOARD`/`CHASSIS_BOARD`), vision protocol (`VISION_USE_VCP`/`VISION_USE_UART`/`VISION_USE_SP`), robot type, and hardware params. **Only ONE board mode can be defined**—changing requires full rebuild.

## Build Commands (Windows)
```powershell
mingw32-make -j24           # Build
mingw32-make download_dap   # Flash via DAP-Link  
mingw32-make download_jlink # Flash via J-Link
```

## Naming Conventions
- **Functions**: `PascalCase` verb-noun (e.g., `SetMotorControl()`)
- **Variables**: `snake_case` (e.g., `gimbal_recv_cmd`)
- **Types**: `IMU_Data_t` (simple data), `Motor_Controller_s` (complex struct), `CANInstance` (module instance)
- **Private**: Prefix `_` for static vars/functions in .c files

## Module Initialization Pattern
Modules are NOT auto-initialized. APP must register them explicitly:
```c
// In gimbal.c - init motors and pub-sub
yaw_motor = DJIMotorInit(&yaw_config);  
gimbal_pub = PubRegister("gimbal_feed", sizeof(Gimbal_Upload_Data_s));
```

## Inter-APP Communication: Message Center
**Never use globals** between APP modules. Use pub-sub:
```c
PubPushMessage(pub, &data);       // Publish
SubGetMessage(sub, &recv);        // Subscribe
```
For dual-board: use `can_comm` module, not message_center.

## FreeRTOS Critical Rules
- **RobotInit()**: Interrupts disabled—use only `DWT_Delay()`, never `osDelay()`/`HAL_Delay()`
- **ISRs**: Use `*FromISR()` variants only
- **Periodic tasks**: Use `vTaskDelayUntil()`, not `osDelay()` for guaranteed timing

## Memory Safety
```c
#pragma pack(1)  // Required for CAN/UART protocol structs
typedef struct { uint8_t cmd; float angle; } Vision_Ctrl_t;
#pragma pack()
```
Always NULL-init and check pointers before use.

## DJI Motor Usage
```c
DJIMotorSetRef(yaw_motor, target_angle);  // CAN grouping is automatic
// Motors use triple-loop cascade PID (angle → speed → current)
```

## Key Files
| File | Purpose |
|------|---------|
| `application/robot.c` | Entry point, calls all init/task functions |
| `application/robot_def.h` | Configuration hub (**review first**) |
| `modules/motor/DJImotor/` | GM6020/M3508/M2006 drivers |
| `modules/message_center/` | Pub-sub for inter-app comms |
| `bsp/dwt/bsp_dwt.c` | High-precision delay (preferred over HAL_Delay) |

## Common Pitfalls
1. Missing `BSPInit()` before module init
2. Using `osDelay()` before scheduler starts—use `DWT_Delay()`
3. Shared task/ISR variables without mutex/critical section
4. `static` vars in .h files create per-.c copies—use `extern`

## CubeMX Regeneration
User code MUST be in `/* USER CODE BEGIN */` blocks—everything else gets overwritten.
