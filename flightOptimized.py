import math
from Jetprofinal import Engine
import numpy as np
from scipy.optimize import minimize
# ------------------------------------------------------------
# EXAMPLE USAGE / ORGANIZED PRINT
# ------------------------------------------------------------
if __name__ == "__main__":

    #Part 1

    #FLIGHT
    T_a = 220 # Kelvin
    P_a = 29.0 # kPa
    M = 0.85
    Prc = 60
    Prf = 1.23
    Beta = 10.8
    b = 0.02
    f = 0.01
    f_ab = 0.01

    turbofan_engine = Engine(T_a, P_a*1000, M, Prc, Prf, Beta, b, f, f_ab)
    x0 = [ Prf, Prc, Beta, b, f, f_ab]

    #minimizer function
    #Inputs Prf,Prc,Beta,b,f,f_ab
    def minFlightTSFC(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.tsfcCase()

    def prcineqConstraint(x):
        return 60/x[0] - x[1]

    def turbineTempWithin(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.turbineTemperatureContraint()
    
    def abTempWithin(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.abTemperatureContraint()

    def minThrustFlight(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.minimumThrust(0.86)
    
    def withinWork(x):
        return turbofan_engine.withinFanTurbineWork(x[0],x[1],x[2],x[3],x[4],x[5])
    


    constrain = [
        {'type':'ineq','fun':withinWork},
        {'type':'ineq','fun':prcineqConstraint},
        {'type':'ineq','fun':turbineTempWithin},
        {'type':'ineq','fun':abTempWithin},
        {'type':'ineq','fun':minThrustFlight},

    ]

    bound = ((1.1,1.5),(1.0,None),(0.0,30.0),(0.0,0.12),(0.001,0.2),(0,0.2))

    #Only if using Cobyla
    '''
    for factor in range(len(bound)):
        if (factor == 1):
            l = {'type': 'ineq',
                'fun': lambda x, lb=1, i=factor: x[1] - lb}
            constrain.append(l)
            constrain.append({'type':'ineq','fun':prcineqConstraint})
        else: 
            lower, upper = bound[factor]
            l = {'type': 'ineq',
                'fun': lambda x, lb=lower, i=factor: x[i] - lb}
            u = {'type': 'ineq',
                'fun': lambda x, ub=upper, i=factor: ub - x[i]}
            constrain.append(l)
            constrain.append(u)
    '''
    

    result = minimize(minFlightTSFC, x0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    flight_optimal_turbofan_engine = Engine(T_a, P_a*1000, M, result.x[1], result.x[0], result.x[2], result.x[3], result.x[4], result.x[5])


    results = flight_optimal_turbofan_engine.run_cycle()

    print("\nFlight OPTIMIZATION")
    print("Optimal Congifuration for Flight to minimize TSFC: Prc = ", result.x[1], "Prf =", result.x[0], "Beta =", \
          result.x[2], "b = ", result.x[3], "f = ",result.x[4], "f_ab = ",result.x[5])
 
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

    


    flightOp_groundroll_turbofan_engine = Engine(285, 100*1000, 0, result.x[1], result.x[0], result.x[2], 0.05, 0.01, 0.01)


    #Minimize TSFC by changing last 3
    def minGroundTSFC(y):
        flightOp_groundroll_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return flightOp_groundroll_turbofan_engine.tsfcCase()


    def turbineTempWithinGround(y):
        flightOp_groundroll_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return flightOp_groundroll_turbofan_engine.turbineTemperatureContraint()
 
    
    def abTempWithinGround(y):
        flightOp_groundroll_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return flightOp_groundroll_turbofan_engine.abTemperatureContraint()

    def minThrustGround(y):
        flightOp_groundroll_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return flightOp_groundroll_turbofan_engine.minimumThrust(2.8)
    
    def withinWorkGround(y):
        return flightOp_groundroll_turbofan_engine.withinFanTurbineWork(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
    
    constrain = [
        {'type':'ineq','fun':withinWorkGround},
        {'type':'ineq','fun':turbineTempWithinGround},
        {'type':'ineq','fun':abTempWithinGround},
        {'type':'ineq','fun':minThrustGround},

    ]

    y0 = [0.009, 0.016, 0.02]

    bound = ((0.0,0.12),(0.001,0.2),(0,0.2))
    

    result = minimize(minGroundTSFC, y0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    results = flightOp_groundroll_turbofan_engine.run_cycle()

    print("\n\nFlight OPTIMIZED AIRLINER AT Ground")
 
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
   

    

    







