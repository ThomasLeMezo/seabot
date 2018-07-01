#!/bin/python
from math import *
import matplotlib.pyplot as plt
import numpy as np

tick_to_volume = (1.75e-3/24.0)*((0.05/2.0)**2)*pi
# V0 = (0.12/2.0)**2 * pi * 0.63

m=8.810 # kg
rho_eau = 1000.0 # 1e3/m3
V0 = m/rho_eau

# K = 1/3 * E/(1-2*nu)
# e = s/K
# e : taux de déformation
# s : contrainte

### POMC
## Poisson = 0.35 (POMC)
## E traction : 3100 MPa
## E flexion : 2900 MPa
# E = 2900e6
# nu = 0.35

### Polycarbonate
E = 2000e6
nu = 0.37

# contrainte pour 5 bar = 5e5 Pa

##########################
K = (1.0/3.0) * E/(1.0-2.0	*nu)


def compute_delta(depth):
	s = depth*1e4
	e = s/K
	# V = ((0.12*(1-e))/2.0)**2 * pi * 0.63*(1-e)
	V = m*(1-e)/rho_eau
	return (V0 - V)/tick_to_volume

d = np.linspace(0, 50, 100)

tick = compute_delta(d)

print(d)
plt.plot(d, tick)
plt.show()

