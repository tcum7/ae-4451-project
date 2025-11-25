import math
from Jetprofinal import Engine
import numpy as np
from scipy.optimize import minimize
# ------------------------------------------------------------
# EXAMPLE USAGE / ORGANIZED PRINT
# ------------------------------------------------------------
if __name__ == "__main__":

    #Part 1
    #Minimize fuel consumption at cruise priority
    #FLIGHT
    T_a = 220 # Kelvin
    P_a = 29.0 # kPa
    M = 0.86
    Prc = 54
    Prf = 1.23
    Beta = 9
    b = 0.02
    f = 0.01
    f_ab = 0.01

    turbofan_engine = Engine(T_a, P_a*1000, M, Prc, Prf, Beta, b, f, f_ab)
    x0 = [ Prf, Prc, Beta, b, f, f_ab]

    #minimizer function
    #Inputs Prf,Prc,Beta,b,f,f_ab
    def minFlightTSFC(x):
        turbofan_engine.changeFree(T_a, P_a*1000, M)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.tsfcCase()

    def prcineqConstraint(x):
        return 60/x[0] - x[1]

    def turbineTempWithin(x):
        turbofan_engine.changeFree(T_a, P_a*1000, M)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.turbineTemperatureContraint()
    
    def turbineTempWithinGround(x):
        turbofan_engine.changeFree(285, 100*1000, 0)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.turbineTemperatureContraint()
    
    def abTempWithin(x):
        turbofan_engine.changeFree(T_a, P_a*1000, M)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.abTemperatureContraint()
    
    def abTempWithinGround(x):
        turbofan_engine.changeFree(285, 100*1000, 0)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.abTemperatureContraint()

    def minThrustFlight(x):
        turbofan_engine.changeFree(T_a, P_a*1000, M)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.minimumThrust(0.86)
    
    def minThrustGround(x):
        turbofan_engine.changeFree(285, 100*1000, 0)
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.minimumThrust(2.8)
    
    def withinWork(x):
        turbofan_engine.changeFree(T_a, P_a*1000, M)
        return turbofan_engine.withinFanTurbineWork(x[0],x[1],x[2],x[3],x[4],x[5])
    
    def withinWorkGround(x):
        turbofan_engine.changeFree(285, 100*1000, 0)
        return turbofan_engine.withinFanTurbineWork(x[0],x[1],x[2],x[3],x[4],x[5])
    
    constrain = [
        {'type':'ineq','fun':withinWork},
        {'type':'ineq','fun':prcineqConstraint},
        {'type':'ineq','fun':turbineTempWithin},
        {'type':'ineq','fun':abTempWithin},
        {'type':'ineq','fun':minThrustFlight},
        {'type':'ineq','fun':withinWorkGround},
        {'type':'ineq','fun':turbineTempWithinGround},
        {'type':'ineq','fun':abTempWithinGround},
        {'type':'ineq','fun':minThrustGround},

    ]

    bound = ((1.1,1.5),(1.0,None),(0.0,30.0),(0.0,0.12),(0.001,0.2),(0,0.2))
    

    result = minimize(minFlightTSFC, x0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    cruise = Engine(220, 29*1000, 0.86, result.x[1], result.x[0], result.x[2], result.x[3], result.x[4], result.x[5])


    results = cruise.run_cycle()

    print("\nFlight Optimized")
    print("Optimal Congifuration for Flight to minimize TSFC: Prc = ", result.x[1], "Prf =", result.x[0], "Beta =", \
          result.x[2], "b = ", result.x[3], "f = ",result.x[4], "f_ab = ",result.x[5])
 
    cruise.outputResults(results)

    

    ground = Engine(285, 100*1000, 0, result.x[1], result.x[0], result.x[2], result.x[3], result.x[4], result.x[5])


    results = ground.run_cycle()

    print("\n\nFlight tsfc optimized at Ground")
 
    ground.outputResults(results)
   

    

    







