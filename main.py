import math
from Jetprofinal import Engine
import numpy as np
from scipy.optimize import minimize
# ------------------------------------------------------------
# EXAMPLE USAGE / ORGANIZED PRINT
# ------------------------------------------------------------
if __name__ == "__main__":
    engine = Engine(
        T_a=220.0,
        P_a=10000.0,
        M=1.5,
        Prc=30,
        Prf=1.2,
        Beta=2,
        b=0.1,
        f=0.018,
        f_ab=0.01
    )
 
    results = engine.run_cycle()

    print("\n\EXAMPLE")
 
    print("\n=== STATION STATES (P (kPa), T(K)) ===")
    for key in [
        "Inlet (P_o2, T_o2)",
        "Fan exit (P_o3f, T_o3f)",
        "Compressor exit (P_o3, T_o3)",
        "Burner exit (P_o4, T_o4)",
        "Core turbine exit (P_o5_1, T_o5_1))",
        "Turbine mixer exit (P_o5m, T_o5m)",
        "Fan turbine exit (P_o5_2, T_o5_2)",
        "Afterburner exit (P_o6,  T_o6)",
    ]:
        print(f"{key:35s}: {results[key]}")

    print("\n=== FUEL PUMP ===")
    for key in [
        "Fuel pump exit pressure (kPa)",
        "Fuel pump work (kJ/kg)",
    ]:
        print(f"{key:35s}: {results[key]}")
 
    print("\n=== NOZZLE EXIT CONDITIONS ===")
    for key in [
        "Core nozzle sep (T_e, P_e)",
        "Fan nozzle sep (T_ef, P_ef)",
        "Combined nozzles (T_o7, gammma_nm, P_o7, T_ec)",
    ]:
        print(f"{key:35s}: {results[key]}")
 
    print("\n=== PERFORMANCE: SEPARATE NOZZLES ===")
    for key in [
        "Separate Nozzle Speeds (u_e (m/s), u_ef (m/s))",
        "Separate Nozzle Performance (T/ma (kNs/kg), TSFC (kg/kNs), eff_th (%), eff_p(%), eff_o(%))", 
    ]:
        print(f"{key:35s}: {results[key]}")

    print("\n=== Work Required ===")
    for key in [
        "Work Required in kJ/kg (w_c, w_p, w_ft)",
    ]:
        print(f"{key:35s}: {results[key]}")

    print("\n=== Max F/A Ratios ===")
    for key in [
        "Max F/A Ratios (f_max, f_max_ab)",
    ]:
        print(f"{key:35s}: {results[key]}")
 
    print("\n=== PERFORMANCE: COMBINED NOZZLE ===")
    for key in [
        "Combined Nozzle Performance (u_ec, T/ma, TSFC, eff_th, eff_p, eff_o)",
    ]:
        print(f"{key:35s}: {results[key]}")


    

    







