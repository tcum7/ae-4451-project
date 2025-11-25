import math
from Jetprofinal import Engine
import numpy as np
from scipy.optimize import minimize
# ------------------------------------------------------------
# EXAMPLE USAGE / ORGANIZED PRINT
# ------------------------------------------------------------
if __name__ == "__main__":

    #Part 1 

    #Ground
    T_a = 285 # Kelvin
    P_a = 100.0 # kPa
    M = 0
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
    def minGroundTSFC(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.maximizeThrust()

    def prcineqConstraint(x):
        return 60/x[0] - x[1]

    def turbineTempWithin(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.turbineTemperatureContraint()
    
    def abTempWithin(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.abTemperatureContraint()

    def minThrustGround(x):
        turbofan_engine.change(x[0],x[1],x[2],x[3],x[4],x[5])
        return turbofan_engine.minimumThrust(2.8)
    
    def withinWork(x):
        return turbofan_engine.withinFanTurbineWork(x[0],x[1],x[2],x[3],x[4],x[5])
    

    constrain = [
        {'type':'ineq','fun':withinWork},
        {'type':'ineq','fun':prcineqConstraint},
        {'type':'ineq','fun':turbineTempWithin},
        {'type':'ineq','fun':abTempWithin},
        {'type':'ineq','fun':minThrustGround},

    ]

    bound = ((1.1,1.5),(1.0,None),(0.0,30.0),(0.0,0.12),(0.001,0.2),(0,0.2))
    

    result = minimize(minGroundTSFC, x0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    ground_optimal_turbofan_engine = Engine(T_a, P_a*1000, M, result.x[1], result.x[0], result.x[2], result.x[3], result.x[4], result.x[5])


    results = ground_optimal_turbofan_engine.run_cycle()

    print("\n\nGROUND OPTIMIZATION")
    print("Optimal Congifuration for Ground to minimize TSFC: Prc = ", result.x[1], "Prf =", result.x[0], "Beta =", \
          result.x[2], "b = ", result.x[3], "f = ",result.x[4], "f_ab = ",result.x[5])
    
    ground_optimal_turbofan_engine.outputResults(results)



    groundOp_cruiseLevel_turbofan_engine = Engine(220, 29*1000, 0.86, result.x[1], result.x[0], result.x[2], 0.05, 0.02, 0.02)


    #Minimize TSFC by changing last 3
    def minFlightTSFC(y):
        groundOp_cruiseLevel_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return groundOp_cruiseLevel_turbofan_engine.tsfcCase()


    def turbineTempWithinFlight(y):
        groundOp_cruiseLevel_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return groundOp_cruiseLevel_turbofan_engine.turbineTemperatureContraint()
 
    
    def abTempWithinFlight(y):
        groundOp_cruiseLevel_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return groundOp_cruiseLevel_turbofan_engine.abTemperatureContraint()

    def minThrustFlight(y):
        groundOp_cruiseLevel_turbofan_engine.change(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
        return groundOp_cruiseLevel_turbofan_engine.minimumThrust(0.86)
    
    def withinWorkFlight(y):
        return groundOp_cruiseLevel_turbofan_engine.withinFanTurbineWork(result.x[0],result.x[1],result.x[2],y[0],y[1],y[2])
    
    constrain = [
        {'type':'ineq','fun':withinWorkFlight},
        {'type':'ineq','fun':turbineTempWithinFlight},
        {'type':'ineq','fun':abTempWithinFlight},
        {'type':'ineq','fun':minThrustFlight},

    ]

    y0 = [0.05, 0.02, 0.03]

    bound = ((0.0,0.12),(0.001,0.2),(0,0.2))
    

    result = minimize(minFlightTSFC, y0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)


    results = groundOp_cruiseLevel_turbofan_engine.run_cycle()

    print("\n\nGROUND OPTIMIZED AIRLINER AT CRUISE")

    groundOp_cruiseLevel_turbofan_engine.outputResults(results)
 
    