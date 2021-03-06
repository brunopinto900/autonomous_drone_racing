#!/usr/bin/env python
# coding: utf-8

# In[1]:


from bagpy import bagreader
import bagpy
import pandas as pd
import seaborn as sea
import matplotlib.pyplot as plt
b = bagreader('test3_09-23-59.bag')

# Read Laser Data
csv = b.laser_data()
df = pd.read_csv(csv[0])

# Read Velocity Data
ms = b.vel_data()
vel = pd.read_csv(ms[0])

# Read Standard Messages
s = b.std_data()
data = pd.read_csv(s[0])

# Read odometry Data
odom = b.odometry_data()
odomdata = pd.read_csv(odom[0])

# Read Wrench Data
w = b.wrench_data()
wdata = pd.read_csv(w[0])       

# Get the plots
b.plot_odometry()
b.plot_vel()
b.plot_wrench()
# Animate Velocity Timeseries
bagpy.animate_timeseries(vel['Time'], vel['linear.x'], title='Velocity Timeseries Plot')
