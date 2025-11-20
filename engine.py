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
        #work rate over mass flow rate at inlet
        work = Cp_fan*(T_o_fan-T)
        return P_o_fan,T_o_fan,delta_drag, work

    def compressor(self,P_o,T_o):
        gamma = 1.38
        eff_c = (self.Prc**((gamma-1)/gamma)-1)/(self.Prc**((gamma-1)/(gamma*0.90))-1)
        T_o_c = T_o*(1+1/eff_c*((self.Prc)**((gamma-1)/gamma)-1))
        P_o_c = P_o*self.Prc
        Cp_c = gamma*self.R_uni/28.8/(gamma-1)
        work = Cp_c*(T_o_c-T_o)
        return P_o_c,T_o_c, work
    
    def burner(self,P_o,T_o):
        gamma = 1.33
        eff_burner = 0.99
        Cp_burner = gamma*self.R_uni/28.8/(gamma-1)
        Q = 45*10**6
        T_o_burner = (eff_burner*self.f*Q/Cp_burner+T_o)/(1+self.f)
        P_o_burner = P_o*0.98
        return T_o_burner, P_o_burner
    
    def turbine(self,P_o,T_o,work):
        gamma = 1.33
        Cp_turbine = gamma*self.R_uni/28.8/(gamma-1)
        T_o_turbine = T_o-work/(Cp_turbine(1+self.f))
        Tr = T_o_turbine/T_o
        eff_turbine = (Tr-1)/(Tr**(1/0.92)-1)
        P_o_turbine = P_o*(1-1/eff_turbine*(Tr**((gamma-1)/gamma)))
        bmax = 0.12
        C_b_1 = 700
        Tmax = 1300+C_b_1*math.sqrt(self.b/bmax)
        return P_o_turbine,T_o_turbine,Tmax
    
    def turbinemixer(self,P_o_turbine,T_o_turbine,T_o_compressor):
        gamma=1.34
        T_o_tm = (T_o_turbine*(1-self.b) + T_o_compressor*self.b)
        P_o_tm = (T_o_tm*T_o_compressor**(-self.b)*T_o_turbine**(self.b-1))**(gamma/(gamma-1))*P_o_turbine*P_o_turbine**(self.b)
        return P_o_tm, T_o_tm
    
    def afterburner(self,P_o,T_o):
        gamma = 1.32
        Q = 45*10**6
        Cp_ab = gamma*self.R_uni/28.8/(gamma-1)
        T_o_ab = (0.96*self.f_ab*Q/Cp_ab+(1+self.f)*T_o)/(1+self.f+self.f_ab)
        P_o_ab = P_o*0.97
        return P_o_ab,T_o_ab
    
    def corenozzle(self,P_o,T_o):
        gamma = 1.35
        Cp_cN = gamma*self.R_uni/28.8/(gamma-1)
        T_e = T_o*(1-0.95*(1-(self.P_a/P_o)**((gamma-1)/gamma)))
        u_e = math.sqrt(2*Cp_cN*(T_o-T_e))
        return T_e,u_e
    
    def fannozzle(self,P_o,T_o):
        gamma = 1.4
        Cp_cN = gamma*self.R_uni/28.8/(gamma-1)
        T_e = T_o*(1-0.95*(1-(self.P_a/P_o)**((gamma-1)/gamma)))
        u_e = math.sqrt(2*Cp_cN*(T_o-T_e))
        return T_e,u_e
    
    def combinednozzle(self,P_o_core,T_o_core,P_o_fan,T_o_fan):
        
        T_o_mix = (T_o_core+T_o_fan*self.Beta)/(1+self.Beta)
        gamma_mix = 1.44-1.39*10**(-4)*T_o_mix+3.57*10**(-8)*T_o_mix**2
        P_o_mix = 0.80*(T_o_mix*T_o_fan**(-self.Beta/(1+self.Beta))*T_o_core**(-1/(1+self.Beta)))**(gamma_mix/(gamma_mix-1))*P_o_core*P_o_fan**(self.Beta/(self.Beta+1))
        gamma = 1.37
        T_e = T_o_mix*(1-0.95*(1-(self.P_a/P_o_mix)**((gamma-1)/gamma)))
        Cp_cN = gamma*self.R_uni/28.8/(gamma-1)
        u_e = math.sqrt(2*Cp_cN*(T_o_mix-T_e))
        return T_e,P_o_mix,u_e
    
    def fuelpump(self,P_o_burner):
        rho_f = 780
        P_o_inlet = 104000
        P_o_e = 550000+P_o_burner
        work = (P_o_e-P_o_inlet)/(0.35*rho_f)
        return P_o_e, work
        




        


        

        


        

