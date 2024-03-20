# -*- coding: utf-8 -*-
"""
Created on Wed Feb 28 16:13:08 2024

@author: clair
"""

import numpy as np
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

NOx = 15E12  # pptv 10^12

T = np.arange(0, 720, 1)

dNOx = 0
slope = []
lijn = []

for t in T:
    if t < 120:
        dNOx = -NOx / (1.5 * 24)
        NOx = NOx + dNOx
        slope.append(NOx)

    if 120 <= t < 480:
        # dNOx = -NOx/(1.5*24*60) + 0.394/(24*60)
        # dNOx = -NOx/(36) +215000000
        dNOx = 2.15E23 - (NOx / (1.5 * 24))
        # dNOx = 0.12/24 * 0.029/0.014 * 6.02*10**23- NOx/(1.5*24)
        NOx = NOx + dNOx
        slope.append(NOx)

    if t >= 480:
        dNOx = - NOx / (1.5 * 24)
        NOx = NOx + dNOx
        slope.append(NOx)

slopeunit = []
for i in range(len(slope)):
    slope_unit = slope[i] / (6.02E26) * 14E15
    slopeunit.append(slope_unit)

# plt.plot(T, slope)
plt.plot(T, slopeunit)
plt.xlabel('Time in hours')
plt.ylabel('kg N/box')
plt.title('Kilograms N in the Box over Time')
plt.show()

