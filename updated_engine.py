import math
import numpy as np
from scipy.optimize import brentq
 
class Engine:
    '''
    Turbofan parametric cycle model with:
    - Inlet
    - Fan
    - Compressor
    - Burner
    - Turbine
    - Turbine mixer
    - Afterburner
    - Nozzles (fan/core/combined)
    '''
 
    def __init__(self, T_a, P_a, M, Prc, Prf, Beta, b, f, f_ab):
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
        self.R = self.R_uni / 28.8
        self.gamma_air = 1.4
 
        self.rho_amb = P_a/(T_a*self.R)
        self.c_inf = math.sqrt(self.gamma_air*self.R*T_a)
        self.v_inf = self.c_inf*M
 
    # ------------------------------------------------------------
    # INLET
    # ------------------------------------------------------------
    def inlet(self):
        gamma = 1.4
 
        T_o_inlet = self.T_a * (1 + (gamma - 1)/2 * self.M**2)
 
        if self.M < 1:
            r_d = 1
        else:
            r_d = 1 - 0.075 * ((self.M - 1)**1.35)
 
        eff_d = 0.92
        P_o_inlet = r_d * self.P_a * (1 + eff_d*(gamma - 1)/2 * self.M**2)**(gamma/(gamma - 1))
 
        return P_o_inlet, T_o_inlet
 
    # ------------------------------------------------------------
    # FAN
    # ------------------------------------------------------------
    def fan(self, P, T):
        gamma = 1.4
 
        eff_fan = (self.Prf**((gamma - 1)/gamma) - 1) / \
                  (self.Prf**((gamma - 1)/(gamma*0.90)) - 1)
 
        C_beta_1 = 245
        delta_drag = C_beta_1 * self.M**2 * (self.P_a/101325) * self.Beta**1.5
 
        T_o_fan = T * (1 + (1/eff_fan) * (self.Prf**((gamma - 1)/gamma) - 1))
        P_o_fan = P * self.Prf
 
        Cp = gamma*self.R/(gamma - 1)
        work = Cp*(T_o_fan - T)
 
        return P_o_fan, T_o_fan, delta_drag, work
 
    # ------------------------------------------------------------
    # COMPRESSOR
    # ------------------------------------------------------------
    def compressor(self, P_o, T_o):
        gamma = 1.38
 
        eff_c = (self.Prc**((gamma - 1)/gamma) - 1) / \
                (self.Prc**((gamma - 1)/(gamma*0.90)) - 1)
 
        T_o_c = T_o * (1 + (1/eff_c) * (self.Prc**((gamma - 1)/gamma) - 1))
        P_o_c = P_o * self.Prc
 
        Cp = gamma*self.R/(gamma - 1)
        work = Cp*(T_o_c - T_o)
 
        return P_o_c, T_o_c, work
 
    # ------------------------------------------------------------
    # BURNER
    # ------------------------------------------------------------
    def burner(self, P_o, T_o):
        gamma = 1.33
        eff_burner = 0.99
 
        Cp = gamma*self.R/(gamma - 1)
        Q = 45e6
 
        T_o_burner = (eff_burner*self.f*Q/Cp + (1-self.b)*T_o)/(1-self.b+self.f)
        P_o_burner = P_o * 0.98
 
        return T_o_burner, P_o_burner
 
    # ------------------------------------------------------------
    # TURBINE
    # ------------------------------------------------------------
    def turbine(self, P_o, T_o, work_c):
        gamma = 1.33
        Cp = gamma*self.R/(gamma - 1)
 
        T_o_turb = T_o - work_c / (Cp*(1 - self.b + self.f))
        Tr = T_o_turb / T_o
        eff_t = (Tr - 1) / (Tr**(1/0.92) - 1)
        print(eff_t)
        def equation(Pr):
            exp1 = ((gamma-1)/gamma)
            exp2 = ((gamma-1)/(gamma*0.92))
            return eff_t-(Pr**exp1-1)/(Pr**exp2-1)
        Pr=brentq(equation,0.01,0.99)
        print(Pr)
        P_o_turb = Pr*P_o
 
        return P_o_turb, T_o_turb
 
    # ------------------------------------------------------------
    # TURBINE MIXER
    # ------------------------------------------------------------
    def turbinemixer(self, P_t, T_t, T_comp):
        gamma = 1.34
       
        T_mix = (T_t*(1 - self.b) + self.b*T_comp)
 
        P_mix = P_t  # Simplified placeholder
 
        return P_mix, T_mix
 
    # ------------------------------------------------------------
    # AFTERBURNER
    # ------------------------------------------------------------
    def afterburner(self, P_o, T_o):
        gamma = 1.32
        Q = 45e6
 
        Cp = gamma*self.R/(gamma - 1)
        T_o_ab = (0.96*self.f_ab*Q/Cp + (1 + self.f)*T_o) / (1 + self.f + self.f_ab)
        P_o_ab = P_o * 0.97
 
        return P_o_ab, T_o_ab
 
    # ------------------------------------------------------------
    # CORE NOZZLE
    # ------------------------------------------------------------
    def corenozzle(self, P_o, T_o):
        gamma = 1.35
        Cp = gamma*self.R/(gamma - 1)
 
        T_e = T_o * (1 - 0.95*(1 - (self.P_a/P_o)**((gamma - 1)/gamma)))
        u_e = math.sqrt(2*Cp*(T_o - T_e))
 
        return T_e, u_e
 
    # ------------------------------------------------------------
    # FAN NOZZLE
    # ------------------------------------------------------------
    def fannozzle(self, P_o, T_o):
        gamma = 1.4
        Cp = gamma*self.R/(gamma - 1)
 
        T_e = T_o * (1 - 0.95*(1 - (self.P_a/P_o)**((gamma - 1)/gamma)))
        u_e = math.sqrt(2*Cp*(T_o - T_e))
 
        return T_e, u_e
 
    # ------------------------------------------------------------
    # COMBINED NOZZLE
    # ------------------------------------------------------------
    def combinednozzle(self, P_core, T_core, P_fan, T_fan):
        T_mix = (T_core + self.Beta*T_fan) / (1 + self.Beta)
 
        gamma = 1.37
        Cp = gamma*self.R/(gamma - 1)
 
        # Mixed pressure (simplified)
        P_mix = 0.8*(P_core + self.Beta*P_fan)/(1 + self.Beta)
 
        T_e = T_mix * (1 - 0.95*(1 - (self.P_a/P_mix)**((gamma - 1)/gamma)))
        u_e = math.sqrt(2*Cp*(T_mix - T_e))
 
        return T_e, P_mix, u_e
 
    # ------------------------------------------------------------
    # FUEL PUMP
    # ------------------------------------------------------------
    def fuelpump(self, P_burner):
        rho_f = 780
        P_in = 104000
        P_out = 550000 + P_burner
        work = (P_out - P_in) / (0.35 * rho_f)
        return P_out, work
 
    # ------------------------------------------------------------
    # FULL CYCLE RUNNER
    # ------------------------------------------------------------
    def run_cycle(self):
 
        # 1. Inlet
        P2, T2 = self.inlet()
 
        # 2. Fan
        P13, T13, drag, W_fan = self.fan(P2, T2)
 
        # 3. Compressor
        P3, T3, W_comp = self.compressor(P13, T13)
 
        # 4. Burner
        T4, P4 = self.burner(P3, T3)
 
        # 5. Turbine (drives compressor)
        P5, T5 = self.turbine(P4, T4, W_comp)
 
        # 6. Turbine mixer
        P6, T6 = self.turbinemixer(P5, T5, T3)
 
        # 7. Afterburner
        P7, T7 = self.afterburner(P6, T6)
 
        # 8. Combined nozzle
        T_e, P_mix, u_e = self.combinednozzle(P5, T5, P13, T13)
 
        return {
            "Inlet": (P2, T2),
            "Fan": (P13, T13),
            "Compressor": (P3, T3),
            "Burner": (P4, T4),
            "Turbine": (P5, T5),
            "Mixer": (P6, T6),
            "Afterburner": (P7, T7),
            "Nozzle Exit Velocity": u_e,
            "Nozzle Exit Temp": T_e
        }
 
 
# ------------------------------------------------------------
# EXAMPLE USAGE
# ------------------------------------------------------------
if __name__ == "__main__":
    engine = Engine(
        T_a=220,       # K
        P_a=10000,     # Pa
        M=1.5,
        Prc=30,
        Prf=1.2,
        Beta=2,
        b=0.1,
        f=0.018,
        f_ab=0.01
    )
 
    results = engine.run_cycle()
    for stage, vals in results.items():
        print(stage, vals)
 
 
