import math

 
class Engine:
    """
    Turbofan parametric cycle model with:
    - Inlet
    - Fan
    - Compressor
    - Burner
    - Turbine
    - Turbine mixer
    - Fan turbine
    - Afterburner
    - Separate core / fan nozzles
    - Combined nozzle
    """
 
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
 
        self.R_uni = 8314.0
        self.R = self.R_uni / 28.8
        self.gamma_air = 1.4
 
        self.rho_amb = P_a / (T_a * self.R)
        self.c_inf = math.sqrt(self.gamma_air * self.R * T_a)
        self.v_inf = self.c_inf * M

        # Fuel pump parameters (from project spec)
        self.eta_pump   = 0.35          # pump efficiency
        self.rho_fuel   = 780.0         # kg/m^3
        self.p_f1       = 104e3         # Pa  (fuel tank pressure)
        self.deltaP_inj = 550e3         # Pa  (injector overpressure)
 
    # ------------------------------------------------------------
    # INLET
    # ------------------------------------------------------------
    def inlet(self):
        gamma = 1.4
        T_o_inlet = self.T_a * (1 + (gamma - 1)/2 * self.M**2)
 
        if self.M < 1:
            r_d = 1.0
        else:
            r_d = 1.0 - 0.075 * ((self.M - 1.0)**1.35)
 
        eff_d = 0.92
        P_o_inlet = r_d * self.P_a * (1 + eff_d*(gamma - 1)/2 * self.M**2)**(gamma/(gamma - 1))
 
        return P_o_inlet, T_o_inlet
 
    # ------------------------------------------------------------
    # FAN
    # ------------------------------------------------------------
    def fan(self, P_o, T_o):
        gamma = 1.4
 
        eff_fan = (self.Prf**((gamma - 1)/gamma) - 1.0) / (self.Prf**((gamma - 1)/(gamma*0.90)) - 1.0)
 
        # Specific drag constant C_beta_1 = 0.245 kN·s/kg
        C_beta_1 = 0.245
        delta_drag = C_beta_1 * self.M**2 * (self.P_a/101325.0) * self.Beta**1.5
 
        T_o_fan = T_o * (1.0 + (1.0/eff_fan) * (self.Prf**((gamma - 1)/gamma) - 1.0))
        P_o_fan = P_o * self.Prf
 
        Cp = gamma * self.R / (gamma - 1.0)
        work_c = Cp * (T_o_fan - T_o)
        work_fan = (1.0 + self.Beta) * work_c
 
        return P_o_fan, T_o_fan, delta_drag, work_fan
 
    # ------------------------------------------------------------
    # COMPRESSOR
    # ------------------------------------------------------------
    def compressor(self, P_o, T_o):
        gamma = 1.38
 
        eff_c = (self.Prc**((gamma - 1)/gamma) - 1.0) / \
                (self.Prc**((gamma - 1)/(gamma*0.90)) - 1.0)
 
        T_o_c = T_o * (1.0 + (1.0/eff_c) * (self.Prc**((gamma - 1)/gamma) - 1.0))
        P_o_c = P_o * self.Prc
 
        Cp = gamma * self.R / (gamma - 1.0)
        work = Cp * (T_o_c - T_o)
 
        return P_o_c, T_o_c, work
 
    # ------------------------------------------------------------
    # FUEL PUMP
    # ------------------------------------------------------------
    def fuel_pump(self, P3):
        """
        Fuel pump model.
        Input:
            P3 : combustor/burner inlet pressure (Pa)  ~ compressor exit
        Returns:
            P_f2   : fuel pump exit pressure (Pa)
            w_pump : pump power per unit core air mass flow (W per (kg/s) of air)
        """
        # Exit fuel pressure (must be ΔP_inj above combustor inlet)
        P_f2 = P3 + self.deltaP_inj

        # Actual pressure rise across the pump (from storage to injection manifold)
        deltaP_pump = P_f2 - self.p_f1

        # Work per unit core air mass flow
        fuel_ratio_total = self.f + self.f_ab
        w_pump = fuel_ratio_total * deltaP_pump / (self.eta_pump * self.rho_fuel)

        return P_f2, w_pump

    # ------------------------------------------------------------
    # BURNER
    # ------------------------------------------------------------
    def burner(self, P_o, T_o):
        gamma = 1.33
        eff_burner = 0.99
        Cp = gamma * self.R / (gamma - 1.0)
        Q = 45e6
 
        T_o_burner = (eff_burner * self.f * Q / Cp + (1.0 - self.b) * T_o) / (1.0 - self.b + self.f)
        P_o_burner = P_o * 0.98

        bmax = 0.12
        C_b_1 = 700
        T_max = 1300+C_b_1*math.sqrt(self.b/bmax)

        f_max = (1.0 - self.b)*(T_o-T_max)/(T_max-eff_burner*Q/Cp)
        withinTemp = T_o_burner<T_max
 
        return P_o_burner,T_o_burner, f_max, withinTemp
 
    # ------------------------------------------------------------
    # CORE TURBINE
    # ------------------------------------------------------------
    def turbine(self, P_o, T_o, work_c):
        gamma = 1.33
        Cp = gamma * self.R / (gamma - 1.0)
 
        T_o_turb = T_o - work_c / (Cp * (1.0 - self.b + self.f))
        Tr = T_o_turb / T_o
 
        exp = 0.92 * (gamma - 1.0) / gamma
        Pr = Tr**(1.0 / exp)
        P_o_turb = Pr * P_o
 
        return P_o_turb, T_o_turb
 
    # ------------------------------------------------------------
    # TURBINE MIXER
    # ------------------------------------------------------------
    def turbinemixer(self, P_t, T_t, T_comp):
        # Bleed air (b) is reintroduced at turbine exit
        gamma_mix = 1.34
        T_mix = T_t * (1.0 - self.b) + self.b * T_comp
        Cp_mix  = gamma_mix * self.R_uni/28.8 / (gamma_mix - 1)
        P_mix = P_t*(T_mix/T_t)**(Cp_mix/self.R)*(T_t/T_comp)**(Cp_mix/self.R*self.b)



        return P_mix, T_mix
 
    # ------------------------------------------------------------
    # FAN TURBINE
    # ------------------------------------------------------------
    def fanturbine(self, P_o, T_o, work_f):
        gamma = 1.33
        Cp = gamma * self.R / (gamma - 1.0)
 
        T_o_turb = T_o - work_f / (Cp * (1.0 + self.f))
        Tr = T_o_turb / T_o
 
    
        if(Tr<0):
            Pr =0.001
        exp = 0.92 * (gamma - 1.0) / gamma
        Pr = Tr**(1.0 / exp)
        P_o_turb = Pr * P_o
 
        return P_o_turb, T_o_turb
 
    # ------------------------------------------------------------
    # AFTERBURNER
    # ------------------------------------------------------------
    def afterburner(self, P_o, T_o):
        gamma = 1.32
        Q = 45e6
        Cp = gamma * self.R / (gamma - 1.0)
 
        T_o_ab = (0.96 * self.f_ab * Q / Cp + (1.0 + self.f) * T_o) / (1.0 + self.f + self.f_ab)
        P_o_ab = P_o * 0.97

        T_max = 2200

        f_max = (1.0 + self.f)*(T_o-T_max)/(T_max-0.96*Q/Cp)
        withinTemp = T_o_ab<T_max

        return P_o_ab, T_o_ab, f_max, withinTemp
 
    #---------------------------------
    # CORE NOZZLE (SEPARATE)
    #---------------------------------
    def corenozzle(self, P_o, T_o):
        gamma = 1.35
        Cp_cN = gamma * self.R_uni / 28.8 / (gamma - 1)
        # nozzle efficiency = 0.95
        T_e = T_o * (1 - 0.95 * (1 - (self.P_a / P_o)**((gamma - 1) / gamma)))
        u_e = math.sqrt(2 * Cp_cN * (T_o - T_e))
        # Project assumption: nozzle exit pressure = ambient
        return T_e, u_e, self.P_a
 
    #---------------------------------
    # FAN NOZZLE (SEPARATE)
    #---------------------------------
    def fannozzle(self, P_o, T_o):
        """
        Fan Nozzle Properties: Exit Temperature and Velocity
        """
        gamma = 1.4
        Cp_cN = gamma * self.R_uni / 28.8 / (gamma - 1)
        # nozzle efficiency = 0.97
        T_e = T_o * (1 - 0.97 * (1 - (self.P_a / P_o)**((gamma - 1) / gamma)))
        u_e = math.sqrt(2 * Cp_cN * (T_o - T_e))
        # Project assumption: nozzle exit pressure = ambient
        return T_e, u_e, self.P_a
 
    # ------------------------------------------------------------
    # COMBINED NOZZLE (MIXER + COMBINED NOZZLE)
    # ------------------------------------------------------------
    def combinednozzle(self, P_core, T_core, P_fan, T_fan):
        """
        Combined Nozzle (Nozzle Mixer + Combined Nozzle).
        We use a simple reversible mixing approximation + loss.
        """

 
        # Mixing temperature (approx mass-weighted)
        T_mix= (self.Beta*T_fan+(1+self.f+self.f_ab)*T_core) / (1+self.f+self.f_ab+self.Beta)
        gamma_mix = 1.44 - 1.39e-4*T_mix + 3.57e-8*(T_mix**2)
        Cp_mix    = gamma_mix * self.R_uni/28.8 / (gamma_mix - 1)
 
        #Derived Pressure Equation
        fan_ratio = self.Beta/(1+self.f+self.f_ab+self.Beta)
        P_mix_rev = P_core*(P_fan/P_core)**(fan_ratio)*(T_mix/T_core)**(Cp_mix/self.R)*(T_core/T_fan)**(Cp_mix/self.R*fan_ratio)
        P_mix = 0.80 * P_mix_rev
 
        # Combined nozzle expansion
        gamma_cn = 1.37
        eta_n    = 0.95
        Cp_cn    = gamma_cn * self.R_uni/28.8 / (gamma_cn - 1)

        T_e = T_mix * (1 - eta_n * (1 - (self.P_a / P_mix)**((gamma_cn - 1) / gamma_cn)))
        u_e = math.sqrt(2 * Cp_cn * (T_mix - T_e))
 
        return T_mix, gamma_mix, P_mix, T_e, u_e, self.P_a
 
    # ------------------------------------------------------------
    # SEPARATE NOZZLE PERFORMANCE (S_eff and TSFC)
    # ------------------------------------------------------------
    def separate_nozzle_performance(self, u_e_core, u_e_fan, drag):
        
        """
        Effective specific thrust and TSFC using SEPARATE nozzles.
        Uses the project-style momentum-only expression:
            τ/m_a = (1+f+f_ab)*u_ec + β*u_ef - (1+β)*V∞
        and assumes P_e = P_a (no pressure thrust).
        """

        Q = 45e6 

        V_inf = self.v_inf
 
        # Specific thrust (no drag), per unit core air mass (kN·s/kg)
        S_no_drag = ((1.0 + self.f + self.f_ab) * u_e_core
                     + self.Beta * u_e_fan
                     - (1.0 + self.Beta) * V_inf) / 1000.0
 
        # Subtract specific drag (already in kN·s/kg)
        S_eff = S_no_drag - drag
 
        # TSFC based on separate-nozzle effective specific thrust
        tsfc_val = (self.f + self.f_ab) / S_eff

        #Thermal Efficiency
        delta_KE = ((1.0+self.f+self.f_ab)*(u_e_core**2)+self.Beta*(u_e_fan**2)-(1.0 + self.Beta)*(V_inf**2))/2
        n_th = delta_KE/((self.f+self.f_ab)*Q)

        #Propulsive Efficiency
        n_p = S_eff*V_inf*1000/(delta_KE)

        #Overall Efficiency
        n_o = n_th*n_p

 
        return S_eff, tsfc_val, n_th, n_p, n_o
 
    # ------------------------------------------------------------
    # COMBINED NOZZLE PERFORMANCE (S_eff and TSFC)
    # ------------------------------------------------------------
    def combined_nozzle_performance(self, u_e_comb, drag):
        """
        Effective specific thrust and TSFC using COMBINED nozzle.
        Project-style expression:
            τ/m_a = (1+f+f_ab+β)*u_ec - (1+β)*V∞
        """
        Q = 45e6 
        V_inf = self.v_inf
 
        S_no_drag = ((1.0 + self.f + self.f_ab + self.Beta) * u_e_comb
                     - (1.0 + self.Beta) * V_inf) / 1000.0
 
        S_eff = S_no_drag - drag
        tsfc_val = (self.f + self.f_ab) / S_eff

        #Thermal Efficiency
        delta_KE = ((1+self.f+self.f_ab+self.Beta)*(u_e_comb**2)-(1.0 + self.Beta) * V_inf**2)/2
        n_th = delta_KE/((self.f+self.f_ab)*Q)

        #Propulsive Efficiency
        n_p = 1000*S_eff*V_inf/(delta_KE)

        #Overall Efficiency
        n_o = n_th*n_p
 
        return S_eff, tsfc_val, n_th, n_p, n_o
 
    # ------------------------------------------------------------
    # TSFC helper (if ever needed directly)
    # ------------------------------------------------------------
    def tsfc(self, S_eff):
        return (self.f + self.f_ab) / S_eff
 
    
    # ------------------------------------------------------------
    # Change engine cycle aspects
    # ------------------------------------------------------------
    def change(self, Prf, Prc, Beta, b, f, f_ab):
        self.Prf = Prf
        self.Prc = Prc
        self.Beta = Beta
        self.b = b
        self.f = f
        self.f_ab = f_ab
        return
    
    # ------------------------------------------------------------
    # Change freestream
    # ------------------------------------------------------------
    def changeFree(self,T_a,P_a,M):
        self.T_a = T_a
        self.P_a = P_a
        self.M = M
        return

    
    # ------------------------------------------------------------
    # Part 1 Iterator to minimize TSFC
    # ------------------------------------------------------------
    def tsfcCase(self):

         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        P_o6,  T_o6, f_max_ab, withinTemp_o6  = self.afterburner(P_o5_2, T_o5_2)

        # 2. Separate nozzles

        T_e, u_e, P_e = self.corenozzle(P_o6,  T_o6)
        T_ef,  u_ef, P_ef  = self.fannozzle(P_o3f, T_o3f)
 
        # 3. Combined nozzle
        T_o7, gammma_nm, P_o7, T_ec, u_ec, p_ec = self.combinednozzle(P_o6, T_o6, P_o3f, T_o3f)
 
        # 4. Thrust and TSFC for both nozzle configurations
        S_eff_sep,  tsfc_sep, n_th_sep, n_p_sep, n_o_sep  = self.separate_nozzle_performance(u_e, u_ef, drag)
        S_eff_comb, tsfc_comb, n_th_comb, n_p_comb, n_o_comb = self.combined_nozzle_performance(u_ec, drag)
        if tsfc_sep<0:
            return tsfc_comb
        elif tsfc_comb<0:
            return tsfc_sep
        else:
            return min(tsfc_sep,tsfc_comb)
    
    def turbineTemperatureContraint(self):

         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        return f_max-self.f
    
    def velocityConstraint(self):

         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        P_o6,  T_o6, f_max_ab, withinTemp_o6  = self.afterburner(P_o5_2, T_o5_2)

          # 2. Separate nozzles

        T_e, u_e, P_e = self.corenozzle(P_o6,  T_o6)
        T_ef,  u_ef, P_ef  = self.fannozzle(P_o3f, T_o3f)
 
        # 3. Combined nozzle
        T_o7, gammma_nm, P_o7, T_ec, u_ec, p_ec = self.combinednozzle(P_o6, T_o6, P_o3f, T_o3f)
 
        # 4. Thrust and TSFC for both nozzle configurations
        S_eff_sep,  tsfc_sep, n_th_sep, n_p_sep, n_o_sep  = self.separate_nozzle_performance(u_e, u_ef, drag)
        S_eff_comb, tsfc_comb, n_th_comb, n_p_comb, n_o_comb = self.combined_nozzle_performance(u_ec, drag)

        return min(u_ec,u_e)
        

    def abTemperatureContraint(self):

         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        P_o6,  T_o6, f_max_ab, withinTemp_o6  = self.afterburner(P_o5_2, T_o5_2)
        return f_max_ab-self.f_ab
    
    def withinFanTurbineWork(self, Prf, Prc, Beta, b, f, f_ab):
        self.change(Prf, Prc, Beta, b, f, f_ab)
         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        return T_o5_2

        
    
    def minimumThrust(self,thrustReq):

         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        P_o6,  T_o6, f_max_ab, withinTemp_o6  = self.afterburner(P_o5_2, T_o5_2)

          # 2. Separate nozzles

        T_e, u_e, P_e = self.corenozzle(P_o6,  T_o6)
        T_ef,  u_ef, P_ef  = self.fannozzle(P_o3f, T_o3f)
 
        # 3. Combined nozzle
        T_o7, gammma_nm, P_o7, T_ec, u_ec, p_ec = self.combinednozzle(P_o6, T_o6, P_o3f, T_o3f)
 
        # 4. Thrust and TSFC for both nozzle configurations
        S_eff_sep,  tsfc_sep, n_th_sep, n_p_sep, n_o_sep  = self.separate_nozzle_performance(u_e, u_ef, drag)
        S_eff_comb, tsfc_comb, n_th_comb, n_p_comb, n_o_comb = self.combined_nozzle_performance(u_ec, drag)

        return max(S_eff_comb,S_eff_sep)-thrustReq
    

    def maximizeThrust(self):

         # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        P_o6,  T_o6, f_max_ab, withinTemp_o6  = self.afterburner(P_o5_2, T_o5_2)

          # 2. Separate nozzles

        T_e, u_e, P_e = self.corenozzle(P_o6,  T_o6)
        T_ef,  u_ef, P_ef  = self.fannozzle(P_o3f, T_o3f)
 
        # 3. Combined nozzle
        T_o7, gammma_nm, P_o7, T_ec, u_ec, p_ec = self.combinednozzle(P_o6, T_o6, P_o3f, T_o3f)
 
        # 4. Thrust and TSFC for both nozzle configurations
        S_eff_sep,  tsfc_sep, n_th_sep, n_p_sep, n_o_sep  = self.separate_nozzle_performance(u_e, u_ef, drag)
        S_eff_comb, tsfc_comb, n_th_comb, n_p_comb, n_o_comb = self.combined_nozzle_performance(u_ec, drag)

        return S_eff_comb
        


 
 
    # ------------------------------------------------------------
    # RUN CYCLE
    # ------------------------------------------------------------
    def run_cycle(self):
        # 1. Upstream components
        P_o2,  T_o2  = self.inlet()
        P_o3f, T_o3f, drag, w_ft = self.fan(P_o2, T_o2)
        P_o3,  T_o3, w_c = self.compressor(P_o3f, T_o3f)
        P_o4, T_o4, f_max, withinTemp_o4  = self.burner(P_o3, T_o3)
        P_o5_1,  T_o5_1  = self.turbine(P_o4, T_o4, w_c)
        P_o5m,  T_o5m  = self.turbinemixer(P_o5_1, T_o5_1, T_o3)
        P_o5_2, T_o5_2 = self.fanturbine(P_o5m, T_o5m, w_ft)
        P_o6,  T_o6, f_max_ab, withinTemp_o6  = self.afterburner(P_o5_2, T_o5_2)

        # 1.2 Fuel Pump
        P_f2, w_p = self.fuel_pump(P_o3)
 
        # 2. Separate nozzles

        T_e, u_e, P_e = self.corenozzle(P_o6,  T_o6)
        T_ef,  u_ef, P_ef  = self.fannozzle(P_o3f, T_o3f)
 
        # 3. Combined nozzle
        T_o7, gammma_nm, P_o7, T_ec, u_ec, p_ec = self.combinednozzle(P_o6, T_o6, P_o3f, T_o3f)
 
        # 4. Thrust and TSFC for both nozzle configurations
        S_eff_sep,  tsfc_sep, n_th_sep, n_p_sep, n_o_sep  = self.separate_nozzle_performance(u_e, u_ef, drag)
        S_eff_comb, tsfc_comb, n_th_comb, n_p_comb, n_o_comb = self.combined_nozzle_performance(u_ec, drag)
 
        return {
            # Station states
            "Inlet (P_o2, T_o2)":                           (P_o2/1000, T_o2),
            "Fan exit (P_o3f, T_o3f)":                      (P_o3f/1000, T_o3f),
            "Compressor exit (P_o3, T_o3)":                 (P_o3/1000, T_o3),
            "Burner exit (P_o4, T_o4)":                     (P_o4/1000, T_o4),
            "Core turbine exit (P_o5_1, T_o5_1))":          (P_o5_1/1000, T_o5_1),
            "Turbine mixer exit (P_o5m, T_o5m)":            (P_o5m/1000, T_o5m),
            "Fan turbine exit (P_o5_2, T_o5_2)":            (P_o5_2/1000, T_o5_2),
            "Afterburner exit (P_o6,  T_o6)":               (P_o6/1000,  T_o6),
 
            # Fuel pump parameters
            "Fuel pump exit pressure (kPa)":        P_f2/1000,
            "Fuel pump work (kJ/kg)":   w_p/1000,

            # Nozzle exit conditions
            "Core nozzle sep (T_e, P_e)":      (T_e, P_e/1000),
            "Fan nozzle sep (T_ef, P_ef)":       (T_ef,  P_ef/1000),
            "Combined nozzles (T_o7, gammma_nm, P_o7, T_ec)": (T_o7, gammma_nm, P_o7/1000, T_ec),
 
            # Performance: separate nozzles
            "Separate Nozzle Speeds (u_e (m/s), u_ef (m/s))": (u_e, u_ef),
            "Separate Nozzle Performance (T/ma (kNs/kg), TSFC (kg/kNs), eff_th (%), eff_p(%), eff_o(%))": (S_eff_sep, tsfc_sep, n_th_sep*100, n_p_sep*100, n_o_sep*100), 

            # Work Required
            "Work Required in kJ/kg (w_c, w_p, w_ft)": (w_c/1000, w_p/1000, w_ft/1000),

            # Max Fuel/Air Ratios 
            "Max F/A Ratios (f_max, f_max_ab)": (f_max, f_max_ab),

            # Performance: combined nozzle
            "Combined Nozzle Performance (u_ec, T/ma, TSFC, eff_th, eff_p, eff_o)": (u_ec, S_eff_comb, tsfc_comb, n_th_comb*100, n_p_comb*100, n_o_comb*100),


        }
 
 

