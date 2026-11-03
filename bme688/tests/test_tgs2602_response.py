#!/usr/bin/env python3
"""
实时监测 TGS2602 传感器电压值
用于测试对酒精的响应速度

使用方法：
1. 确保 TGS2602 已连接到 PYNQ（通过分压电路）
2. 在 PYNQ 上运行对应的 ADC 读取服务
3. 运行此脚本监测电压变化

或者如果直接连接到 AI PC 的 ADC：
需要指定 ADC 设备路径
"""

import time
import sys
import argparse
from datetime import datetime
import socket
import json


class TGS2602Monitor:
    """TGS2602 传感器监测器"""

    def __init__(self, host='192.168.2.99', port=8766):
        """
        初始化监测器

        Args:
            host: PYNQ IP 地址
            port: ADC 服务端口（与 BME688 的 8765 不同）
        """
        self.host = host
        self.port = port
        self.socket = None
        self.baseline = None

    def connect(self):
        """连接到 PYNQ ADC 服务"""
        try:
            print(f"Connecting to {self.host}:{self.port}...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(10.0)
            self.socket.connect((self.host, self.port))

            # 接收 hello 消息
            data = self._receive_message()
            if data.get('type') == 'hello':
                print(f"Connected to {data.get('device', 'Unknown')}")
                print(f"ADC Channels: {data.get('channels', [])}")
                return True
            return False
        except Exception as e:
            print(f"Connection failed: {e}")
            print("\nMake sure PYNQ ADC server is running:")
            print("  ssh xilinx@192.168.2.99")
            print("  cd /home/xilinx/jupyter_notebooks/AIR_SENSITIVE")
            print("  sudo python3 pynq_tgs2602_server.py")
            return False

    def _receive_message(self):
        """接收一条 JSON 消息"""
        buffer = b''
        while True:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise ConnectionError("Connection closed")
            buffer += chunk
            if b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                return json.loads(line.decode('utf-8'))

    def get_sample(self):
        """获取一个 ADC 样本"""
        data = self._receive_message()
        if data.get('type') == 'sample':
            return data.get('payload', {})
        return None

    def monitor_realtime(self, duration=60, interval=0.5, baseline_samples=10):
        """
        实时监测电压变化

        Args:
            duration: 监测时长（秒）
            interval: 采样间隔（秒）
            baseline_samples: 基线样本数（用于计算初始值）
        """
        print(f"\n{'='*70}")
        print(f"TGS2602 Real-time Voltage Monitor")
        print(f"{'='*70}")
        print(f"Duration: {duration}s | Interval: {interval}s")
        print(f"\nCollecting baseline ({baseline_samples} samples)...")

        # 采集基线
        baseline_values = []
        for i in range(baseline_samples):
            sample = self.get_sample()
            if sample:
                voltage = sample.get('voltage_left', 0)  # 假设左侧传感器
                baseline_values.append(voltage)
                print(f"  [{i+1}/{baseline_samples}] {voltage:.3f}V")
            time.sleep(0.2)

        if not baseline_values:
            print("Failed to collect baseline")
            return

        self.baseline = sum(baseline_values) / len(baseline_values)
        print(f"\nBaseline: {self.baseline:.3f}V")
        print(f"\n{'='*70}")
        print(f"{'Time':^12} | {'Voltage':^10} | {'Delta':^10} | {'Change%':^10} | {'Bar':^20}")
        print(f"{'='*70}")

        start_time = time.time()
        max_voltage = self.baseline
        min_voltage = self.baseline

        try:
            while time.time() - start_time < duration:
                sample = self.get_sample()
                if not sample:
                    continue

                # 获取电压值
                voltage_left = sample.get('voltage_left', 0)
                voltage_right = sample.get('voltage_right', 0)

                # 使用左侧传感器（或平均值）
                voltage = voltage_left

                # 计算变化
                delta = voltage - self.baseline
                change_pct = (delta / self.baseline * 100) if self.baseline > 0 else 0

                # 更新最大最小值
                max_voltage = max(max_voltage, voltage)
                min_voltage = min(min_voltage, voltage)

                # 生成条形图
                bar_length = int(abs(change_pct) * 0.5)  # 每 2% 一个字符
                if delta > 0:
                    bar = '+' * min(bar_length, 20)
                else:
                    bar = '-' * min(bar_length, 20)

                # 显示
                elapsed = time.time() - start_time
                timestamp = f"{elapsed:6.1f}s"

                print(f"{timestamp:^12} | {voltage:^10.3f} | {delta:^+10.3f} | {change_pct:^+10.1f} | {bar:<20}")

                time.sleep(interval)

        except KeyboardInterrupt:
            print("\n\nStopped by user")

        # 总结
        print(f"\n{'='*70}")
        print(f"Summary:")
        print(f"  Baseline:     {self.baseline:.3f}V")
        print(f"  Max voltage:  {max_voltage:.3f}V (+{(max_voltage-self.baseline):.3f}V, +{(max_voltage-self.baseline)/self.baseline*100:.1f}%)")
        print(f"  Min voltage:  {min_voltage:.3f}V ({(min_voltage-self.baseline):.3f}V, {(min_voltage-self.baseline)/self.baseline*100:.1f}%)")
        print(f"  Range:        {max_voltage-min_voltage:.3f}V")
        print(f"{'='*70}")

    def close(self):
        """关闭连接"""
        if self.socket:
            self.socket.close()


# 备用：直接读取本地 ADC（如果 TGS2602 连接到 AI PC）
class LocalADCMonitor:
    """本地 ADC 监测器（备用）"""

    def __init__(self, device='/dev/ttyUSB0', channel=0):
        """
        Args:
            device: ADC 设备路径
            channel: ADC 通道
        """
        self.device = device
        self.channel = channel
        print(f"Local ADC mode: {device}, channel {channel}")
        print("Note: This requires proper ADC driver setup")

    def read_voltage(self):
        """读取电压值（需要根据实际 ADC 实现）"""
        # 这里需要根据实际使用的 ADC 芯片实现
        # 例如 MCP3008, ADS1115 等
        raise NotImplementedError("Local ADC reading not implemented yet")


def main():
    parser = argparse.ArgumentParser(
        description='Real-time TGS2602 voltage monitor',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Monitor for 60 seconds, sample every 0.5s
  python test_tgs2602_response.py --host 192.168.2.99 --duration 60

  # Quick test: 30 seconds, sample every 0.2s
  python test_tgs2602_response.py --duration 30 --interval 0.2

Test procedure:
  1. Run this script
  2. Wait for baseline stabilization (10 samples)
  3. Apply alcohol vapor near sensor
  4. Observe voltage change and response time
  5. Remove alcohol and observe recovery time
        """
    )
    parser.add_argument('--host', default='192.168.2.99',
                        help='PYNQ IP address')
    parser.add_argument('--port', type=int, default=8766,
                        help='PYNQ ADC server port')
    parser.add_argument('--duration', type=int, default=60,
                        help='Monitor duration (seconds)')
    parser.add_argument('--interval', type=float, default=0.5,
                        help='Sample interval (seconds)')
    parser.add_argument('--baseline-samples', type=int, default=10,
                        help='Number of samples for baseline')

    args = parser.parse_args()

    # 创建监测器
    monitor = TGS2602Monitor(args.host, args.port)

    # 连接
    if not monitor.connect():
        return 1

    # 开始监测
    try:
        monitor.monitor_realtime(
            duration=args.duration,
            interval=args.interval,
            baseline_samples=args.baseline_samples
        )
    finally:
        monitor.close()

    return 0


if __name__ == '__main__':
    sys.exit(main())
