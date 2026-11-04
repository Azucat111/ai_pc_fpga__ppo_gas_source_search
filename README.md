# FPGA气味搜索小车源码

PYNQ-Z2 / XC7Z020车端源码，基于已部署的v0.1.12 TGS版本，合入2026年11月3日的本机遥控更新。仓库包含RTL、PYNQ软件、ESP32-C3固件源码、构建脚本、配置和离线测试。

| 目录 | 内容 |
|---|---|
| rtl | UART协议、动作租期、双编码器、轮速PI、PWM、I²C仲裁、TGS特征、GAS10邮箱、LD5提示 |
| software/car | PYNQ配置与服务、BME688扫描、TGS日志、协议及主机客户端 |
| firmware/esp32c3 | PlatformIO / Arduino固件源码，UDP与UART桥接 |
| vivado | Vivado构建脚本、管脚及时钟约束 |
| config | 车辆配置与实际采用的PI参数配置 |
| tools | 构建、诊断、离线验证及Windows遥控入口 |
| tests | RTL测试台与Python测试 |
| vendor | 固定版本BME68x驱动和PYNQ PS预设，保留原许可证 |

## FPGA构建

使用Vivado 2022.2，在仓库根目录执行：

```sh
vivado -mode batch -source vivado/build.tcl
```

生成的bit/hwh在build/output，XSA在build/hardware_export。PL时钟5 MHz，轮速控制100 Hz，PWM约20 kHz，LD5蓝灯使用G14。

## PYNQ运行

将源码目录放到/home/xilinx/jupyter_notebooks/FPGA_CAR_v0_1_12_tgs，先构建硬件，再在PYNQ上编译BME68x适配库：

```sh
sh tools/build_bme.sh
sudo sh tools/pynq_run.sh -m car.board --enable --bme-mode gas10 --tgs-log /home/xilinx/tgs_live_20261102.jsonl --config config/car_remote_tuned_20261103.json
```

config/car.json保留Kp10/Ki1基线；当前实际采用config/car_remote_tuned_20261103.json的Kp20/Ki2。software/systemd/fpga-car.service使用这一配置，启动后STOP待命，收到有效的新运动命令才运动。

## ESP32-C3构建

安装PlatformIO，在firmware/esp32c3目录执行：

```sh
pio run
```

热点、UDP与UART配置在include/wifi_config.h。默认热点FPGA-CAR，UDP端口4242，UART为460800、8N1，GPIO20接RX、GPIO21接TX。

## Windows遥控

在仓库根目录创建Python环境，然后双击RUN_REMOTE.cmd：

```sh
python -m venv .venv
```

W/S前后30 rpm，A/D原地转向12 rpm，按住运动、松开停车，Space/Esc显式停车。窗口启动只登记遥测。运行会话临时调整无线性能设置，退出恢复原设置。界面另提供LD5蓝灯5秒闪烁按钮，事件复用同一个通信客户端。

## 离线验证

安装Icarus Verilog，在仓库根目录执行：

```sh
python tools/run_tests.py
python tools/verify_tgs_golden.py
```

测试输出写入build/tests，黄金模型回放输出写入validation/golden。以上命令进行离线测试。Windows可使用tools/bootstrap_sim_windows.py准备Icarus工具。

第三方文件来源与SHA256保留在vendor/manifest.json，各自授权见vendor目录内LICENSE。
