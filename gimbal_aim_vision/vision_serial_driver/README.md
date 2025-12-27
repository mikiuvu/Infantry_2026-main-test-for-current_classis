# vision_serial_driver
ROS2视觉串口驱动包

默认打开/dev/ttyACM0串口
## 用例

```
ros2 run vision_serial_driver vision_serial_driver_node <device_name>
```
## 步兵数据包定义
### 1.visionArray - TX
| Byte | Data |
| - | - |
| 0 | 0xA5 |
| 1 | 0x00 |
| 2-5 | aimYaw |
| 6-9 | aimPitch |

### 2.robotArray - RX
| Byte | Data |
| - | - |
| 0 | 0xA5 |
| 1 | 0x00 |
| 2 | Mode |
| 3 | foeColor |
| 4-7 | robotYaw |
| 8-11 | robotPitch |
| 12-15 | muzzleSpeed |

## Pub
/serial_driver/robot
## Sub
/serial_driver/aim_target