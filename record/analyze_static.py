#!/usr/bin/env python3

import sys, argparse, numpy as np, rosbag
from scipy.signal import welch

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--topic", default="/Odometry")
ap.add_argument("--trim", type=float, default=20.0, help="seconds to skip at start (init)")
args = ap.parse_args()

t, x, y, z = [], [], [], []
with rosbag.Bag(args.bag) as b:
    for _, msg, _ in b.read_messages(topics=[args.topic]):
        t.append(msg.header.stamp.to_sec())
        p = msg.pose.pose.position
        x.append(p.x); y.append(p.y); z.append(p.z)
t = np.asarray(t); x = np.asarray(x); y = np.asarray(y); z = np.asarray(z)
t -= t[0]
mask = t >= args.trim
t, x, y, z = t[mask], x[mask], y[mask], z[mask]
fs = 1.0 / np.median(np.diff(t))

print(f"samples={len(t)}  duration={t[-1]-t[0]:.1f}s  fs={fs:.2f} Hz")
for name, a in [("X", x), ("Y", y), ("Z", z)]:
    a = a - a.mean()
    f, Pxx = welch(a, fs=fs)
    f_peak = f[np.argmax(Pxx)]
    print(f"{name}: std={1000*a.std():6.2f} mm "
        f"pkpk={1000*(a.max()-a.min()):6.1f} mm  "
        f"peak_freq={f_peak:.3f} Hz")


"""
rosbag record -O /home/gale/gale_ws/src/record/static_$(analyse_bag).bag /Odometry /imu/data --duration=90

python3 analyze_static.py analyse_bag.bag


"""