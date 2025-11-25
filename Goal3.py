import math
from Jetprofinal import Engine
import numpy as np
from scipy.optimize import minimize
# ------------------------------------------------------------
# EXAMPLE USAGE / ORGANIZED PRINT
# ------------------------------------------------------------
if __name__ == "__main__":

    #Part 3
    #FROM PREVIOUS PARTS
    Prc = 39.68317
    Prf = 1.5
    Beta = 5.4628

    #To Iterate
    b = 0.012
    f = 0.024
    f_ab = 0.032

    #GroundMaxThrust

    Ground = Engine(285, 100, 0, Prc, Prf, Beta, 0.012, 0.024, 0.032)


    def maximizeThrust(y):
        Ground.change(Prc, Prf, Beta,y[0],y[1],y[2])
        return -Ground.maximizeThrust()


    def turbineTempWithinGround(y):
        Ground.change(Prc, Prf, Beta,y[0],y[1],y[2])
        return Ground.turbineTemperatureContraint()
 
    
    def abTempWithinGround(y):
        Ground.change(Prc, Prf, Beta,y[0],y[1],y[2])
        return Ground.abTemperatureContraint()
    
    def withinWorkGround(y):
        return Ground.withinFanTurbineWork(Prc, Prf, Beta,y[0],y[1],y[2])
    
    constrain = [
        {'type':'ineq','fun':withinWorkGround},
        {'type':'ineq','fun':turbineTempWithinGround},
        {'type':'ineq','fun':abTempWithinGround},
    ]

    y0 = [0.05, 0.05, 0.06]

    bound = ((0.0,0.12),(0.001,0.2),(0,0.2))
    

    result = minimize(maximizeThrust, y0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    Ground = Engine(285, 100, 0, Prc, Prf, Beta, result.x[0], result.x[1], result.x[2])


    results = Ground.run_cycle()

    print("\n\nGROUND Max Thrust")
 
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



    #Cruise
    #FlightMaxThrust

    Flight = Engine(220, 29, 0.86, Prc, Prf, Beta, 0.12, 0.024, 0.032)


    def maximizeFlightThrust(y):
        Flight.change(Prc, Prf, Beta,y[0],y[1],y[2])
        return -Flight.maximizeThrust()


    def turbineTempWithinFlight(y):
        Flight.change(Prc, Prf, Beta,y[0],y[1],y[2])
        return Flight.turbineTemperatureContraint()
 
    
    def abTempWithinFlight(y):
        Flight.change(Prc, Prf, Beta,y[0],y[1],y[2])
        return Flight.abTemperatureContraint()

    
    def withinWorkFlight(y):
        return Flight.withinFanTurbineWork(Prc, Prf, Beta,y[0],y[1],y[2])
    
    constrain = [
        {'type':'ineq','fun':withinWorkFlight},
        {'type':'ineq','fun':turbineTempWithinFlight},
        {'type':'ineq','fun':abTempWithinFlight},
    ]

    y0 = [0.12, 0.02, 0.03]

    bound = ((0.0,0.12),(0.001,0.2),(0,0.2))
    

    result = minimize(maximizeFlightThrust, y0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    Flight = Engine(220, 29, 0.86, Prc, Prf, Beta, result.x[0], result.x[1], result.x[2])


    results = Flight.run_cycle()

    print("\n\nFlight Max Thrust")
 
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
   

    

    







