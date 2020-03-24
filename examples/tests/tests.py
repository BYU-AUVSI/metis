import os, pickle

from shapely.geometry import Polygon
import matplotlib.pyplot as plt
import numpy as np

with open('./mis.pkl', 'r') as f:
    mission = pickle.load(f)

print(mission)



shapes = []
for o in mission.obstacles:
    resolution = 360
    pts = [(o.r*np.cos(x) + o.e, o.r*np.sin(x) + o.n) for x in np.linspace(0, np.pi*2, resolution)]
    circle = Polygon(pts)
    shapes.append(circle)

import matplotlib.pyplot as plt
# n = np.array([item.n for item in landing_waypoint_list])
# e = np.array([item.e for item in landing_waypoint_list])
# plt.plot(e, n, 'rx-')
# for i in range(len(n)):
#     plt.text(e[i], n[i], str(i))

for o in shapes:
    plt.plot(*o.exterior.xy)
plt.axis('equal')
plt.show()

# cir = shapes[0]