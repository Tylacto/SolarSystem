import math
import numpy as np
import matplotlib.pyplot as plt

data = np.load("TwoBodyTest.npy")

x = []
y = []
t = []
dK_list = []

earth_init = data[0][1]
satellite_init = data[0][2]

init_K = 0.5 * (satellite_init.mass * satellite_init.velocity.dot(satellite_init.velocity) + earth_init.mass * earth_init.velocity.dot(earth_init.velocity))

for line in data:
    earth = line[1]
    satellite = line[2]

    t.append(line[0])
    x.append(satellite.position[0])
    y.append(satellite.position[1])

    K = 0.5 * (satellite.mass * satellite.velocity.dot(satellite.velocity) + earth.mass * earth.velocity.dot(earth.velocity))
    dK_list.append(K - init_K)

plt.figure(1)
plt.xlabel("x-position / m")
plt.ylabel("y-position / m")
plt.title("Satellite Orbit")
plt.plot(x, y, "r-")

plt.figure(2)
plt.title("Change in Kinetic Energy of the System with Time")
plt.ylabel("Change in Kinetic Energy / J")
plt.xlabel("Time / s")
plt.plot(t, dK_list, "g-")

plt.show()