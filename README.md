# 气体传感器

## 编译

```bash
cd ~/Workspace/task_ws
colcon build --packages-select gas_monitor --symlink-install
```

## 启动

```bash
sudo usermod -aG dialout cat

sudo cp ~/Workspace/task_ws/src/gas_monitor/systemd/gas_monitor_pump_agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now gas_monitor_pump_agent.service
sudo systemctl restart gas_monitor_pump_agent.service

source ~/Workspace/task_ws/install/setup.zsh
ros2 launch gas_monitor gas_monitor.launch.py

ros2 service call /monitor/gas/start std_srvs/srv/Trigger "{}"
ros2 topic echo /monitor/gas/status | grep -E "(level|name|message|hardware_id)"
ros2 service call /monitor/gas/set_parameters rcl_interfaces/srv/SetParameters \
"{parameters: [
    {name: 'use_config_alarm_thresholds', value: {type: 1, bool_value: true}},
    {name: 'alarm_threshold_slave_ids', value: {type: 7, integer_array_value: [1, 2, 3, 4]}},
    {name: 'low_alarm_overrides', value: {type: 8, double_array_value: [1.0, 2.0, 20.0, 15.0]}},
    {name: 'high_alarm_overrides', value: {type: 8, double_array_value: [3.0, 5.0, 50.0, 30.0]}},
    {name: 'gas_type_overrides', value: {type: 9, string_array_value: ['HF', 'SO2', 'CH4', 'H2S']}}
]}"
ros2 service call /monitor/gas/stop std_srvs/srv/Trigger "{}"
```

## 测试

```bash
ros2 service call /monitor/gas/test_alarm std_srvs/srv/Trigger "{}"
```

## 接口

| 名称                          | 类型                                   |
| ----------------------------- | -------------------------------------- |
| `/monitor/gas/start`          | `std_srvs/srv/Trigger`                 |
| `/monitor/gas/stop`           | `std_srvs/srv/Trigger`                 |
| `/monitor/gas/status`         | `diagnostic_msgs/msg/DiagnosticStatus` |
| `/monitor/gas/test_alarm`     | `std_srvs/srv/Trigger`                 |
| `/monitor/gas/set_parameters` | `rcl_interfaces/srv/SetParameters`     |

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
cat dialout sudo audio video render

sudo bash -c 'stty -F /dev/ttyUSB0 9600 cs8 -cstopb -parenb raw -echo -ixon -ixoff; printf "\x01\x03\x00\x00\x00\x0A\xC5\xCD" > /dev/ttyUSB0; timeout 1 cat /dev/ttyUSB0 | xxd -g 1'
00000000: 01 03 14 00 00 00 3b 01 f4 05 dc 0b b8 00 01 00  ......;.........
00000010: 3b 01 02 06 00 03 5a 38 ad                       ;.....Z8.
```

```bash
ls -l /dev/ttyS5
crw-rw---- 1 root dialout 4, 69  8月 27 17:34 /dev/ttyS5

groups
cat dialout sudo audio video

sudo bash -c 'stty -F /dev/ttyS5 9600 cs8 -cstopb -parenb raw -echo -ixon -ixoff; printf "\x01\x03\x00\x00\x00\x0A\xC5\xCD" > /dev/ttyS5; timeout 1 cat /dev/ttyS5 | xxd -g 1'
00000000: 01 03 14 00 00 00 00 01 f4 05 dc 0b b8 00 01 00  ................
00000010: 00 01 02 06 00 03 5a d1 53                       ......Z.S
```

```bash
sudo ~/Workspace/task_ws/src/gas_monitor/scripts/run_gas_monitor_pump_agent.sh
[gas_monitor_pump_agent]: 气体泵继电器root权限进程代理已启动，socket=/run/gas_monitor/pump_relay.sock

sudo systemctl status gas_monitor_pump_agent.service
● gas_monitor_pump_agent.service - Gas Monitor Pump Relay Root Agent
     Loaded: loaded (/etc/systemd/system/gas_monitor_pump_agent.service; enabled; preset: enabled)
     Active: active (running) since Wed 2026-07-08 11:01:05 CST; 20s ago
   Main PID: 18175 (gas_monitor_pum)
      Tasks: 15 (limit: 19108)
     Memory: 8.7M ()
     CGroup: /system.slice/gas_monitor_pump_agent.service
             └─18175 /home/cat/Workspace/task_ws/install/gas_monitor/lib/gas_monitor/gas_monitor_pump_agent /home/cat/Workspace/task_ws/install/gas_monitor/share/gas_monitor/config/gas_params_default.yaml

ls -l /run/gas_monitor/pump_relay.sock
srw-rw-rw- 1 root root 0  7月  8 10:45 /run/gas_monitor/pump_relay.sock
```
