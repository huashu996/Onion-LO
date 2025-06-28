#!/usr/bin/env python3
import subprocess
import time
import psutil
import matplotlib.pyplot as plt
from datetime import datetime
import os

def get_ros_nodes():
    try:
        output = subprocess.check_output(['rosnode', 'list'], text=True)
        return output.strip().split('\n')
    except:
        return []

def get_node_pid(node_name):
    try:
        output = subprocess.check_output(['rosnode', 'info', node_name], text=True)
        for line in output.splitlines():
            if 'Pid' in line:
                return int(line.split(':')[-1].strip())
    except:
        return None

def get_mem_usage(pid):
    try:
        p = psutil.Process(pid)
        return p.memory_info().rss / 1024 / 1024  # MB
    except:
        return None

def monitor_and_plot(interval=5.0, duration=300):
    print(f"⏱️ 开始监控，每 {interval}s 记录一次，持续 {duration}s ...")
    t0 = time.time()

    memory_records = {}  # 每个节点对应一个列表 [(timestamp, memory)]

    try:
        while time.time() - t0 < duration:
            timestamp = datetime.now().strftime('%H:%M:%S')
            nodes = get_ros_nodes()
            for node in nodes:
                pid = get_node_pid(node)
                if pid:
                    mem = get_mem_usage(pid)
                    if mem is not None:
                        if node not in memory_records:
                            memory_records[node] = []
                        memory_records[node].append((timestamp, mem))
                        print(f"[{timestamp}] {node:<30s} PID: {pid:<6d} Mem: {mem:.2f} MB")
            time.sleep(interval)

    except KeyboardInterrupt:
        print("\n⛔ 手动终止监控。")

    print("📈 正在绘图并保存 PNG 文件...")

    os.makedirs("mem_plots", exist_ok=True)
    peak_summary = {}

    for node, records in memory_records.items():
        times = [t for t, _ in records]
        mems = [m for _, m in records]
        peak = max(mems)
        peak_time = times[mems.index(peak)]
        peak_summary[node] = peak

        plt.figure(figsize=(10, 4))
        plt.plot(times, mems, label='Memory Usage (MB)')
        plt.axhline(peak, color='red', linestyle='--', label=f'Max: {peak:.2f} MB at {peak_time}')
        plt.xticks(rotation=45)
        plt.xlabel("Time")
        plt.ylabel("Memory (MB)")
        plt.title(f"ROS Node Memory Usage: {node}")
        plt.legend()
        plt.tight_layout()

        filename = f"mem_plots/{node.strip('/').replace('/', '_')}_mem_usage.png"
        plt.savefig(filename, dpi=150)
        plt.close()
        print(f"✅ 图像已保存: {filename}")

    # 保存峰值内存记录 txt
    txt_path = "mem_plots/peak_memory.txt"
    with open(txt_path, "w") as f_txt:
        f_txt.write("ROS Node Peak Memory Usage (MB)\n")
        f_txt.write("=" * 40 + "\n")
        for node, peak in sorted(peak_summary.items(), key=lambda x: -x[1]):
            f_txt.write(f"{node:<30s} : {peak:.2f} MB\n")

    print(f"\n📄 所有节点峰值内存使用已保存至：{txt_path}")

if __name__ == '__main__':
    monitor_and_plot(interval=5.0, duration=100000)  # 每5秒采样，持续300秒
