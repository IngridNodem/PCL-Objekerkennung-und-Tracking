#!/usr/bin/env python3
import argparse
import subprocess
import time
from collections import defaultdict

NODES = [
    'crop_box_node',
    'voxel_filter_node',
    'ransac_ground_node',
    'cluster_extraction_node',
    'tracking_node',
]

def pids_for(exe: str, ns: str):
    try:
        # pgrep -af shows "pid cmdline"; filter ns in cmdline
        out = subprocess.check_output(['pgrep','-af',exe], text=True)
    except subprocess.CalledProcessError:
        return []
    pids = []
    token = f"__ns:=/{ns}"
    for line in out.strip().splitlines():
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            continue
        pid, cmd = parts
        if token in cmd:
            pids.append(int(pid))
    return pids

def sample_ps(pid: int):
    try:
        out = subprocess.check_output(['ps','-p',str(pid),'-o','%cpu=','-o','rss='], text=True)
    except subprocess.CalledProcessError:
        return None
    try:
        # e.g. " 5.3  123456"
        parts = out.strip().split()
        if len(parts) < 2:
            return None
        cpu = float(parts[0])
        rss_kb = float(parts[1])
        return cpu, rss_kb
    except Exception:
        return None

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--ns', default='bench', help='Namespace (default: bench)')
    ap.add_argument('--duration', type=float, default=10.0, help='Duration in seconds (default: 10)')
    ap.add_argument('--interval', type=float, default=0.5, help='Sampling interval seconds (default: 0.5)')
    args = ap.parse_args()

    sums_cpu = defaultdict(float)
    sums_rss = defaultdict(float)
    counts = defaultdict(int)

    t_end = time.monotonic() + args.duration
    while time.monotonic() < t_end:
        total_cpu = 0.0
        total_rss = 0.0
        for exe in NODES:
            cpu_agg = 0.0
            rss_agg = 0.0
            for pid in pids_for(exe, args.ns):
                s = sample_ps(pid)
                if s is None:
                    continue
                cpu, rss_kb = s
                cpu_agg += cpu
                rss_agg += rss_kb
            sums_cpu[exe] += cpu_agg
            sums_rss[exe] += rss_agg
            counts[exe] += 1
            total_cpu += cpu_agg
            total_rss += rss_agg
        sums_cpu['TOTAL'] += total_cpu
        sums_rss['TOTAL'] += total_rss
        counts['TOTAL'] += 1
        time.sleep(args.interval)

    def fmt_avg(exe):
        n = max(counts.get(exe,0),1)
        cpu = sums_cpu.get(exe,0.0)/n
        rss_mb = (sums_rss.get(exe,0.0)/n)/1024.0
        return cpu, rss_mb, n

    print(f"Namespace=/{args.ns}, duration={args.duration}s, interval={args.interval}s")
    for exe in NODES + ['TOTAL']:
        cpu, rss_mb, n = fmt_avg(exe)
        print(f"{exe:24s} avg CPU: {cpu:6.2f}%  avg RSS: {rss_mb:7.2f} MB  samples: {n}")

if __name__ == '__main__':
    main()

