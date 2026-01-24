import math

class AISC_F7_Check:
    def __init__(self, B, H, t_nom, Fy, Lb, E=29000, Cb=1.0):
        """
        B, H: Outside Width, Depth (in)
        t_nom: Nominal wall thickness (in)
        Fy: Yield Strength (ksi)
        Lb: Unbraced Length (in) - Critical for LTB check
        Cb: Moment gradient factor (default 1.0 for conservative check)
        """
        # 1. Geometry Setup
        self.t = 0.93 * t_nom  # Design thickness (A500)
        self.b = B - 3 * self.t # Clear width (Flange)
        self.h = H - 3 * self.t # Clear depth (Web)
        self.Fy = Fy
        self.E = E
        self.Lb = Lb
        self.Cb = Cb
        
        # 2. Calculate Section Properties (Simplified for Box)
        # In a real app, calculate these precisely or pull from a DB
        I_outer_x = (B * H**3) / 12
        I_inner_x = ((B - 2*self.t) * (H - 2*self.t)**3) / 12
        self.Ix = I_outer_x - I_inner_x
        self.Sx = self.Ix / (H / 2)
        
        # Plastic Modulus (Zx)
        self.Zx = (B*H**2)/4 - ((B-2*self.t)*(H-2*self.t)**2)/4
        
        # Radius of Gyration about Y-axis (ry) - Needed for LTB
        I_outer_y = (H * B**3) / 12
        I_inner_y = ((H - 2*self.t) * (B - 2*self.t)**3) / 12
        Iy = I_outer_y - I_inner_y
        self.Ag = B*H - (B - 2*self.t)*(H - 2*self.t)
        self.ry = math.sqrt(Iy / self.Ag)
        
        # Torsional Constant (J) - Approx for thin-walled tube
        # J approx = 4 * A_enclosed^2 * t / perimeter_midline
        Am = (B - self.t) * (H - self.t)
        Pm = 2 * ((B - self.t) + (H - self.t))
        self.J = (4 * Am**2 * self.t) / Pm

    def calculate_nominal_strength(self):
        # Calculate individual limit states
        Mn_yield = self._check_yielding()
        Mn_flb = self._check_flange_local_buckling()
        Mn_wlb = self._check_web_local_buckling()
        Mn_ltb = self._check_lateral_torsional_buckling()
        
        # Final Mn is the lowest value
        Mn = min(Mn_yield, Mn_flb, Mn_wlb, Mn_ltb)
        
        return Mn

    def _check_yielding(self):
        # Eq F7-1
        return self.Fy * self.Zx

    def _check_flange_local_buckling(self):
        # Check Slenderness
        lambda_f = self.b / self.t
        sqrt_E_Fy = math.sqrt(self.E / self.Fy)
        
        # Limits (from Table B4.1b, referenced in F7)
        lambda_p = 1.12 * sqrt_E_Fy
        lambda_r = 1.40 * sqrt_E_Fy

        # 1. Compact Flange (No FLB limit state)
        if lambda_f <= lambda_p:
            return float('inf') # Does not control

        # 2. Non-Compact Flange (Eq F7-2)
        elif lambda_f <= lambda_r:
            Mp = self.Fy * self.Zx
            term = 3.57 * (self.b / self.t) * math.sqrt(self.Fy / self.E) - 4.0
            Mn = Mp - (Mp - self.Fy * self.Sx) * term
            return min(Mn, Mp)

        # 3. Slender Flange (Eq F7-3 & F7-4)
        else:
            # Effective Width (be)
            term1 = 0.38 * math.sqrt(self.E / self.Fy)
            be = 1.92 * self.t * math.sqrt(self.E / self.Fy) * (1 - term1 / (self.b/self.t))
            be = min(be, self.b)
            
            # Effective Section Modulus (Se)
            # Simplified: Scale Sx by lost width ratio (approximate)
            # For exact structural engine, recalculate I with reduced flange width
            effective_Sx = self.Sx * (be / self.b) 
            return self.Fy * effective_Sx

    def _check_web_local_buckling(self):
        # Check Web Slenderness
        lambda_w = self.h / self.t
        sqrt_E_Fy = math.sqrt(self.E / self.Fy)
        
        lambda_pw = 2.42 * sqrt_E_Fy
        lambda_rw = 5.70 * sqrt_E_Fy
        
        # 1. Compact Web
        if lambda_w <= lambda_pw:
            return float('inf') # Does not control

        # 2. Non-Compact Web (Eq F7-6)
        elif lambda_w <= lambda_rw:
            Mp = self.Fy * self.Zx
            # Note: The formula in image uses h/tw specifically
            term = 0.305 * (self.h / self.t) * math.sqrt(self.Fy / self.E) - 0.738
            Mn = Mp - (Mp - self.Fy * self.Sx) * term
            return min(Mn, Mp)
            
        else:
            # AISC User Note: "There are no HSS with slender webs"
            # For custom box sections, this would require Eq F7-7 to F7-9
            return 0.0 # Placeholder for "Design not supported for Slender Webs"

    def _check_lateral_torsional_buckling(self):
        # Eq F7-12 (Lp) & F7-13 (Lr)
        Mp = self.Fy * self.Zx
        
        # Lp Calculation
        Lp = 0.13 * self.E * self.ry * math.sqrt(self.J * self.Ag) / Mp
        
        # Lr Calculation
        Lr = 2 * self.E * self.ry * (math.sqrt(self.J * self.Ag) / (0.7 * self.Fy * self.Sx))

        # 1. Compact Brace (No LTB)
        if self.Lb <= Lp:
            return float('inf') # Does not control (Section 4a)

        # 2. Inelastic LTB (Eq F7-10)
        elif self.Lb <= Lr:
            term = (self.Lb - Lp) / (Lr - Lp)
            Mn = self.Cb * (Mp - (Mp - 0.7 * self.Fy * self.Sx) * term)
            return min(Mn, Mp)

        # 3. Elastic LTB (Eq F7-11)
        else:
            Mn = 2 * self.E * self.Cb * (math.sqrt(self.J * self.Ag) / (self.Lb / self.ry))
            return min(Mn, Mp)