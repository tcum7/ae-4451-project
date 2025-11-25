import math
from Jetprofinal import Engine
import numpy as np
from scipy.optimize import minimize
# ------------------------------------------------------------
# EXAMPLE USAGE / ORGANIZED PRINT
# ------------------------------------------------------------
if __name__ == "__main__":

    #Part 3
    #FROM PREVIOUS PARTS (Currently from Meeting both Goals)
    Prc = 48.432
    Prf = 1.239
    Beta = 8.2994

    #To Iterate
    b = 0.012
    f = 0.024
    f_ab = 0.032

    #GroundMaxThrust

    ground = Engine(285, 100*1000, 0, Prc, Prf, Beta, 0.012, 0.024, 0.032)


    def maximizeThrust(y):
        ground.change(Prf, Prc, Beta,y[0],y[1],y[2])
        return -ground.maximizeThrust()


    def turbineTempWithinGround(y):
        ground.change(Prf, Prc, Beta,y[0],y[1],y[2])
        return ground.turbineTemperatureContraint()
 
    
    def abTempWithinGround(y):
        ground.change(Prf, Prc, Beta,y[0],y[1],y[2])
        return ground.abTemperatureContraint()
    
    def withinWorkGround(y):
        return ground.withinFanTurbineWork(Prf, Prc, Beta,y[0],y[1],y[2])
    
    constrain = [
        {'type':'ineq','fun':withinWorkGround},
        {'type':'ineq','fun':turbineTempWithinGround},
        {'type':'ineq','fun':abTempWithinGround},
    ]

    y0 = [0.05, 0.01, 0.02]

    bound = ((0.0,0.12),(0.001,0.1),(0,0.1))

    result = minimize(maximizeThrust, y0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    ground = Engine(285, 100*1000, 0, Prc, Prf, Beta, result.x[0], result.x[1], result.x[2])


    results = ground.run_cycle()


    print("\n\nGROUND Max Thrust")
    print("Optimal Congifuration for ground: ","b = ", result.x[0], "f = ",result.x[1], "f_ab = ",result.x[2])
 
 
    ground.outputResults(results)



    #Cruise
    #FlightMaxThrust

    flight = Engine(220, 29*1000, 0.86, Prc, Prf, Beta, 0.11, 0.024, 0.032)


    def maximizeFlightThrust(y):
        flight.change(Prf, Prc,Beta,y[0],y[1],y[2])
        return -flight.maximizeThrust()


    def turbineTempWithinFlight(y):
        flight.change(Prf, Prc, Beta,y[0],y[1],y[2])
        return flight.turbineTemperatureContraint()
 
    
    def abTempWithinFlight(y):
        flight.change(Prf, Prc, Beta,y[0],y[1],y[2])
        return flight.abTemperatureContraint()

    
    def withinWorkFlight(y):
        return flight.withinFanTurbineWork(Prf, Prc,Beta,y[0],y[1],y[2])
    
    constrain = [
        {'type':'ineq','fun':withinWorkFlight},
        {'type':'ineq','fun':turbineTempWithinFlight},
        {'type':'ineq','fun':abTempWithinFlight},
    ]

    y0 = [0.05, 0.01, 0.02]

    bound = ((0.0,0.12),(0.001,0.1),(0,0.1))
    

    result = minimize(maximizeFlightThrust, y0, constraints = constrain, method='SLSQP', bounds = bound)
    # Print the results
    print("Optimization results:")
    print(f"Optimal solution (x): {result.x}")
    print("Success:", result.success)
    print("Message:", result.message)

    flight = Engine(220, 29*1000, 0.86, Prc, Prf, Beta, result.x[0], result.x[1], result.x[2])


    results = flight.run_cycle()

    print("\n\nFlight Max Thrust")

    print("Optimal Congifuration for Flight: ","b = ", result.x[0], "f = ",result.x[1], "f_ab = ",result.x[2])

 
    flight.outputResults(results)
   

    

    







