import math

class Engine: 
    '''
    Initialize the engine object
    Inputs:
        T_a is ambient temperature
        P_a is ambient pressure
        Prc  is compressure stagnation pressure ratio
        Prf is fan stagnation pressure ratio
        Beta is Bypass ratio
        f is the Fuel-air ratio for the main burner
        fab is Fuel-air ratio for the afterburner
        b is the Bleed ratio
    '''
    def _init_(self, T_a, P_a, M, Prc, Prf, Beta, b, f, f_ab):
        self.T_a = T_a
        self.P_a = P_a
        self.M = M
        self.Prc = Prc
        self.Prf = Prf
        self.Beta = Beta
        self.b = b
        self.f = f
        self.f_ab = f_ab
        self.R_uni = 8314
        self.rho_amb = P_a/(T_a*self.R_uni/28.8)
        self.c_inf = math.sqrt(1.4*T_a*self.R_uni/28.8)
        self.v_inf = self.c_inf*M

    def inlet(self):
        gamma = 1.4
        T_o_inlet = self.T_a*(1+gamma/2*self.M**2)
        if self.M<1:
            r_d = 1
        else:
            r_d = 1-0.075((self.M-1)**1.35)
        eff_d = 0.92
        P_o_inlet = r_d*self.P_a*((1+eff_d*(gamma-1)/2*self.M**2)**((gamma-1)/gamma))
        return P_o_inlet,T_o_inlet

    def fan(self,P,T):
        gamma = 1.4
        eff_fan = (self.Prf**((gamma-1)/gamma)-1)/(self.Prf**((gamma-1)/(gamma*0.90))-1)
        C_beta_1 = 0.245*1000
        delta_drag = C_beta_1*self.M**2*(self.P_a/101325)*self.Beta**1.5
        T_o_fan = T*(1+1/eff_fan*((self.Prf)**((gamma-1)/gamma)-1))
        P_o_fan = P*self.Prf
        Cp_fan = gamma*self.R_uni/28.8/(gamma-1)
        work = Cp_fan*(T_o_fan-T)
        return P_o_fan,T_o_fan,delta_drag, work

    def compressor(self,P,T):
        gamma = 1.38
        eff_c = (self.Prc**((gamma-1)/gamma)-1)/(self.Prc**((gamma-1)/(gamma*0.90))-1)
        T_o_c = T*(1+1/eff_c*((self.Prc)**((gamma-1)/gamma)-1))
        P_o_c = P*self.Prc
        Cp_c = gamma*self.R_uni/28.8/(gamma-1)
        work = Cp_c*(T_o_c-T)
        return P_o_c,T_o_c, work
    
    def burner(self,P,T):
        gamma = 1.33
        eff_burner = 0.99
        Cp_burner = gamma*self.R_uni/28.8/(gamma-1)
        Q = 45*10**6
        T_o_burner = (eff_burner*self.f*Q/Cp_burner+T)/(1+self.f)
        P_o_burner = P*0.98
        return T_o_burner, P_o_burner
    
    def Turbine(self,P,T):
        gamma = 1.33

        


        

        


        

