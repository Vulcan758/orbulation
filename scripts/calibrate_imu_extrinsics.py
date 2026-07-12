#!/usr/bin/env python3
"""Jointly estimate IMU->camera rotation AND image-vs-IMU time offset.

w_cam from rotation-only optical flow (linear LSQ, no decomposition
ambiguity); w_imu interpolated from the gyro; scan time offsets, solve
Wahba's problem at each, keep the offset with the lowest residual.
"""
import sqlite3

import cv2
import numpy as np
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, Imu

BAG = "/home/mahir/orbulating/dataset/zed_wave/zed_wave_0.db3"
K = np.array([[477.525, 0, 329.005], [0, 477.805, 190.107], [0, 0, 1]])
DIST = np.array([-0.0663476, -0.0353098, -9.25829e-05, -9.31486e-05, -0.0344863])

con = sqlite3.connect(BAG)
topics = dict(con.execute("SELECT name, id FROM topics").fetchall())

imu_t, imu_w = [], []
for (blob,) in con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
                           (topics["/zed/imu/data"],)):
    m = deserialize_message(bytes(blob), Imu)
    imu_t.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
    imu_w.append((m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z))
imu_t, imu_w = np.array(imu_t), np.array(imu_w)
t0_ref = imu_t[0]
imu_t -= t0_ref

rows = con.execute("SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
                   (topics["/zed/left/image_raw"],)).fetchall()

def decode(blob):
    m = deserialize_message(bytes(blob), Image)
    t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 - t0_ref
    img = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.width, 3)
    return t, cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

lk = dict(winSize=(21, 21), maxLevel=3,
          criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))

t_mid, w_cam = [], []
prev = None
for i in range(len(rows)):
    t, gray = decode(rows[i][0])
    if prev is None:
        prev = (t, gray)
        continue
    tp, gp = prev
    prev = (t, gray)
    dt = t - tp
    if not (0.02 < dt < 0.15):
        continue
    p0 = cv2.goodFeaturesToTrack(gp, maxCorners=300, qualityLevel=0.01, minDistance=14)
    if p0 is None or len(p0) < 50:
        continue
    p1, st, _ = cv2.calcOpticalFlowPyrLK(gp, gray, p0, None, **lk)
    ok = st.ravel() == 1
    if ok.sum() < 50:
        continue
    q0 = cv2.undistortPoints(p0[ok], K, DIST).reshape(-1, 2)
    q1 = cv2.undistortPoints(p1[ok], K, DIST).reshape(-1, 2)
    x, y = q0[:, 0], q0[:, 1]
    du = q1 - q0
    # rotation-only flow model: du = A(x,y) @ (w*dt)
    A = np.zeros((len(x) * 2, 3))
    A[0::2, 0] = x * y
    A[0::2, 1] = -(1 + x * x)
    A[0::2, 2] = y
    A[1::2, 0] = 1 + y * y
    A[1::2, 1] = -x * y
    A[1::2, 2] = -x
    b = du.reshape(-1)
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    # robust re-fit: drop worst 30% residual points once
    r = (A @ sol - b).reshape(-1, 2)
    keep = np.linalg.norm(r, axis=1) < np.percentile(np.linalg.norm(r, axis=1), 70)
    kk = np.repeat(keep, 2)
    sol, *_ = np.linalg.lstsq(A[kk], b[kk], rcond=None)
    w = sol / dt
    if np.linalg.norm(w) < 0.15:
        continue
    t_mid.append(0.5 * (t + tp))
    w_cam.append(w)

t_mid = np.array(t_mid)
Wc = np.array(w_cam)
print(f"flow samples: {len(Wc)}")

def wahba(offset):
    tq = t_mid + offset
    ok = (tq > imu_t[0] + 0.02) & (tq < imu_t[-1] - 0.02)
    Wi = np.column_stack([np.interp(tq[ok], imu_t, imu_w[:, k]) for k in range(3)])
    Wcc = Wc[ok]
    H = Wcc.T @ Wi
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(U @ Vt))])
    R = U @ D @ Vt
    res = Wcc - (R @ Wi.T).T
    num = np.linalg.norm(res, axis=1)
    den = np.maximum(np.linalg.norm(Wcc, axis=1), 1e-6)
    return R, float(np.median(num / den))

best = None
for off in np.arange(-0.40, 0.41, 0.01):
    R, e = wahba(off)
    if best is None or e < best[2]:
        best = (off, R, e)
off, R_cb, e = best
print(f"best offset (imu_time = image_stamp + offset): {off:+.3f}s, median rel residual {e:.3f}")
for o in (-0.2, -0.1, 0.0, 0.1, 0.2):
    print(f"  residual at offset {o:+.2f}: {wahba(o)[1]:.3f}")

print("R_cb (imu->cam):\n", np.round(R_cb, 4))
R_bc = R_cb.T
print("R_bc (cam->imu, T_b_c1 rotation):\n", np.round(R_bc, 4))

up_imu = np.array([-9.78, -0.51, 0.01]) / 9.79
print("gravity check, predicted camera up (expect ~[0,-1,0]):",
      np.round(R_cb @ up_imu, 3))

P = np.zeros((3, 3))
for r in range(3):
    c = np.argmax(np.abs(R_bc[r]))
    P[r, c] = np.sign(R_bc[r, c])
ang = np.degrees(np.arccos(np.clip((np.trace(P.T @ R_bc) - 1) / 2, -1, 1)))
print(f"nearest signed permutation (det {np.linalg.det(P):.0f}, {ang:.1f} deg away):\n", P)
