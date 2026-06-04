# 气体传感器

## 编译

```bash
cd ~/Workspace/algor_ws
colcon build --packages-select gas_monitor --symlink-install
```

## 启动

```bash
source ~/Workspace/algor_ws/install/setup.zsh
ros2 launch gas_monitor gas_monitor.launch.py

ros2 service call /monitor/gas/start std_srvs/srv/Trigger "{}"
ros2 topic echo /monitor/gas/status | grep -E "(level|name|message|hardware_id)"
ros2 service call /monitor/gas/stop std_srvs/srv/Trigger "{}"
```

## 测试

```bash
ros2 service call /monitor/gas/test_alarm std_srvs/srv/Trigger "{}"
```

## 接口

| 名称                      | 类型                                   |
| ------------------------- | -------------------------------------- |
| `/monitor/gas/start`      | `std_srvs/srv/Trigger`                 |
| `/monitor/gas/stop`       | `std_srvs/srv/Trigger`                 |
| `/monitor/gas/status`     | `diagnostic_msgs/msg/DiagnosticStatus` |
| `/monitor/gas/test_alarm` | `std_srvs/srv/Trigger`                 |

`/monitor/gas/status`示例：

```text
level: "\x02"
name: gas_sensor
message: 气体传感器状态异常：低报
hardware_id: /dev/ttyUSB0
values:
  - {key: sensor_count, value: '6'}
  - {key: sensor_ids, value: 1,2,3,4,5,6}
  - {key: sensor_6.id, : '6'}
  - {key: sensor_6.valid, : 'true'}
  - {key: sensor_6.gas, : O3}
  - {key: sensor_6.gas_type_code, : '68'}
  - {key: sensor_6.concentration, : '0.120'}
  - {key: sensor_6.unit, : ppm}
  - {key: sensor_6.low_alarm, : '0.100'}
  - {key: sensor_6.high_alarm, : '0.500'}
  - {key: sensor_6.full_scale, : '10.000'}
  - {key: sensor_6.status_code, : '5'}
  - {key: sensor_6.status, : 低报}
  - {key: sensor_6.temp, : '25.800'}
  - {key: sensor_6.humidity, : '85.800'}
  - {key: sensor_6.error, : ''}
```

## 排错

```bash
ls -l /dev/ttyUSB0
crw-rw---- 1 root dialout 188, 0  6月  2 10:59 /dev/ttyUSB0

groups
cat root dialout sudo audio video realtime gpio

sudo bash -c 'stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb raw -echo -ixon -ixoff; printf "\x01\x03\x00\x00\x00\x0A\xC5\xCD" > /dev/ttyUSB0; timeout 1 cat /dev/ttyUSB0 | xxd -g 1'
00000000: 01 03 14 00 00 00 3b 01 f4 05 dc 0b b8 00 01 00  ......;.........
00000010: 3b 01 02 06 00 03 5a 38 ad                       ;.....Z8.
```
