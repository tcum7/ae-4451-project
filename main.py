import math
from engine.py import Engine

T_a = 220 # Kelvin
P_a = 10.0 # kPa
M = 1.50
Prc = 30
Prf = 1.2
Beta = 2
b = 0.1
f = 0.018
f_ab = 0.010

turbofan_engine = Engine(T_a, P_a, M, Prc, Prf, Beta, b, f, f_ab)

[P_o_inlet, T_o_inlet] = turbofan_engine.inlet
print P_o_inlet, T_o_inlet
