import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

plt.rcParams.update({'font.size': 22})

titles = ['Roll', 'Pitch', 'Yaw']

vc_rpy = [[], [], []]
init_yaw = 0
with open('demo_data_no_angles/slow_sidewind/vc_estimator_q.txt') as f:
  for linenum, line in enumerate(f):
    q = line.split(' ')
    r = R.from_quat([float(e) for e in q[1:]] + [float(q[0])])
    if linenum == 0:
      rpy = r.as_euler('xyz', degrees=True)
      init_yaw = rpy[2]
    rpy = r.as_euler('xyz', degrees=True)
    rpy[2] = (rpy[2] - init_yaw + 180)%360 - 180
    for i in range(3):
      vc_rpy[i].append(rpy[i])

m1_rpy = [[], [], []]
with open('demo_data_no_angles/slow_sidewind/m1_estimator_q.txt') as f:
  for linenum, line in enumerate(f):
    q = line.split(' ')
    r = R.from_quat([float(e) for e in q[1:]] + [float(q[0])])
    if linenum == 0:
      rpy = r.as_euler('xyz', degrees=True)
      init_yaw = rpy[2]
    rpy = r.as_euler('xyz', degrees=True)
    rpy[2] = (rpy[2] - init_yaw + 180)%360 - 180
    for i in range(3):
      m1_rpy[i].append(rpy[i])

vins_rpy = [[], [], []]
with open('demo_data_no_angles/slow_sidewind/vins_q.txt') as f:
  for linenum, line in enumerate(f):
    q = line.split(' ')
    r = R.from_quat([float(e) for e in q[1:]] + [float(q[0])])
    if linenum == 0:
      rpy = r.as_euler('xyz', degrees=True)
      init_yaw = rpy[2]
    rpy = r.as_euler('xyz', degrees=True)
    rpy[2] = (rpy[2] - init_yaw + 180)%360 - 180
    for i in range(3):
      vins_rpy[i].append(rpy[i])

complementary_rpy = [[], [], []]
with open('demo_data_no_angles/slow_sidewind/complementary_q.txt') as f:
  for linenum, line in enumerate(f):
    q = line.split(' ')
    r = R.from_quat([float(e) for e in q[1:]] + [float(q[0])])
    if linenum == 0:
      rpy = r.as_euler('xyz', degrees=True)
      init_yaw = rpy[2]
    rpy = r.as_euler('xyz', degrees=True)
    rpy[2] = (rpy[2] - init_yaw + 180)%360 - 180
    for i in range(3):
      complementary_rpy[i].append(rpy[i])

fig, (ax1, ax2, ax3) = plt.subplots(3)
fig.suptitle('Head Orientation Estimate for Slow Sidewind Gait')
plt.xlabel('Time (s)')

length = len(m1_rpy[0])
m1_ts = np.arange(length)*0.02
length = len(vins_rpy[0])
vins_ts = np.arange(length)*0.02
length = len(vc_rpy[0])
vc_ts = np.arange(length)*0.02

ax1.plot(m1_ts, m1_rpy[0], label='m1 estimator')
ax1.plot(vins_ts, vins_rpy[0], label='VINS-Fusion')
ax1.plot(vc_ts, vc_rpy[0], label='vc estimator')
ax1.set_ylabel('Roll (degrees)')
ax1.legend(loc='lower left')

ax2.plot(m1_ts, m1_rpy[1], label='m1 estimator')
ax2.plot(vins_ts, vins_rpy[1], label='VINS-Fusion')
ax2.plot(vc_ts, vc_rpy[1], label='vc estimator')
ax2.set_ylabel('Pitch (degrees)')
ax2.legend(loc='lower left')

ax3.plot(m1_ts, m1_rpy[2], label='m1 estimator')
ax3.plot(vins_ts, vins_rpy[2], label='VINS-Fusion')
ax3.plot(vc_ts, vc_rpy[2], label='vc estimator')
ax3.set_ylabel('Yaw (degrees)')
ax3.legend(loc='lower left')

plt.show()
