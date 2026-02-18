import math

def flexural_strength_f9(
    Fy,
    E,
    Sx,
    Zx,
    Iy,
    J,
    d,
    tw,                # Stem/web thickness
    bf,                # Flange width
    tf,                # Flange thickness
    Lb,
    My, Mz,
    Pu,                # Axial force (+ = tension, − = compression)
    section,
    method
):
    """
    AISC 360-22 | Chapter F9
    Yielding + Lateral-Torsional Buckling + Stem Local Buckling + Flange Local Buckling
    Tees and Double Angles Loaded in the Plane of Symmetry
    """

    # -------------------------------------------------
    # 1. Applicability
    # -------------------------------------------------
    if section.type not in ["SectionType.TEE", "SectionType.DOUBLE_ANGLE"]:
        raise ValueError("Chapter F9 applies only to TEES and DOUBLE ANGLES")

    # -------------------------------------------------
    # 2. Bending Axis & Geometry
    # -------------------------------------------------
    # Assume classify_bending_axis is a helper function defined elsewhere
    axis_enum, _, _ = classify_bending_axis(section=section, My=My, Mz=Mz)
    bending_axis = axis_enum.value

    # -------------------------------------------------
    # 3. Tension / Compression from Axial Force
    # -------------------------------------------------
    # In F9, "Tension" refers to the Stem/Web leg being in tension.
    if Pu > 0:
        stem_in_compression = False
        force_case = "Stem in Tension"
    else:
        stem_in_compression = True
        force_case = "Stem in Compression"

    # -------------------------------------------------
    # 4. Yield Moment My and Plastic Moment Mp (F9.1)
    # -------------------------------------------------
    My_val = Fy * Sx

    if section.type == "SectionType.TEE":
        if not stem_in_compression:
            Mp = min(Fy * Zx, 1.6 * My_val)   # F9-2
            yielding_case = "Tee stem in tension (F9-2)"
        else:
            Mp = My_val                      # F9-4
            yielding_case = "Tee stem in compression (F9-4)"

    elif section.type == "SectionType.DOUBLE_ANGLE":
        if not stem_in_compression:
            Mp = My_val                      # F9-3
            yielding_case = "Double-angle web legs in tension"
        else:
            Mp = 1.5 * My_val                # F9-5
            yielding_case = "Double-angle web legs in compression (F9-5)"

    # -------------------------------------------------
    # 5. Lateral-Torsional Buckling (F9.2)
    # -------------------------------------------------
    ry = math.sqrt(Iy / section.A)
    Lp = 1.76 * ry * math.sqrt(E / Fy)
    Mcr = 0.0 

    if not stem_in_compression:
        # Stem in tension: B and Mcr (F9-10, F9-11)
        B = 2.3 * (d / Lb) * math.sqrt(Iy / J)
        Mcr = (1.95 * E / Lb) * math.sqrt(Iy * J) * (B + math.sqrt(1 + B**2))
        
        if Lb <= Lp:
            Mn_LTB = Mp
            ltb_case = "Lb <= Lp (F9-6)"
        else:
            Lr = 1.95 * (E / Fy) * math.sqrt(Iy / Sx) * math.sqrt(2.36 * (Fy / E) * (d * Sx / J) + 1)
            if Lb <= Lr:
                Mn_LTB = Mp - (Mp - My_val) * ((Lb - Lp) / (Lr - Lp))
                ltb_case = "Inelastic LTB (F9-6)"
            else:
                Mn_LTB = Mcr
                ltb_case = "Elastic LTB (F9-7)"
    else:
        # Stem in compression: B and Mcr (F9-10, F9-12)
        B = -2.3 * (d / Lb) * math.sqrt(Iy / J)
        Mcr = (1.95 * E / Lb) * math.sqrt(Iy * J) * (B + math.sqrt(1 + B**2))
        
        if section.type == "SectionType.TEE":
            Mn_LTB = min(Mcr, My_val) # F9-13
            ltb_case = "Tee stem compression (F9-13)"
        else:
            # Double angles per F10.2/F10.3
            ratio = My_val / Mcr
            Mn_LTB = min((1.92 - 1.17 * math.sqrt(ratio)) * My_val, 1.5 * My_val) if ratio <= 1.0 else (0.92 - 0.17 * (Mcr / My_val)) * Mcr
            ltb_case = "Double angle compression (refer F10)"

    # -------------------------------------------------
    # 6. Local Buckling of Stems/Web Legs (F9.3)
    # -------------------------------------------------
    Mn_LB_stem = float('inf')
    stem_lb_case = "N/A - Stem in tension"

    if stem_in_compression:
        if section.type == "SectionType.TEE":
            # For tee stems
            slenderness = d / tw
            limit_p = 0.84 * math.sqrt(E / Fy)
            limit_r = 1.52 * math.sqrt(E / Fy)

            if slenderness <= limit_p:
                Fcr = Fy  # F9-17
                stem_lb_case = "Stem Compact (F9-17)"
            elif slenderness <= limit_r:
                # F9-18
                Fcr = (1.43 - 0.515 * (slenderness) * math.sqrt(Fy / E)) * Fy
                stem_lb_case = "Stem Noncompact (F9-18)"
            else:
                # F9-19
                Fcr = (1.52 * E) / (slenderness**2)
                stem_lb_case = "Stem Slender (F9-19)"
            
            Mn_LB_stem = Fcr * Sx  # F9-16
            
        elif section.type == "SectionType.DOUBLE_ANGLE":
            # Per F9.3(b), refer to Section F10.3 with Sc
            # (F10 logic usually results in LTB governing or similar reductions)
            Mn_LB_stem = Mn_LTB 
            stem_lb_case = "Double-angle web legs (refer F10.3)"

    # -------------------------------------------------
    # 7. Flange Local Buckling (F9.4)
    # -------------------------------------------------
    Mn_FLB = float('inf')
    flb_case = "N/A - Flange in tension"

    if not stem_in_compression:
        # Flange is in compression when stem is in tension
        lamb = bf / (2 * tf)
        lamb_pf = 0.38 * math.sqrt(E / Fy)
        lamb_rf = 1.0 * math.sqrt(E / Fy)
        Sxc = Sx 

        if section.type == "SectionType.TEE":
            if lamb <= lamb_pf:
                Mn_FLB = Mp
                flb_case = "Compact Flange"
            elif lamb <= lamb_rf:
                Mn_FLB = Mp - (Mp - 0.7 * Fy * Sxc) * ((lamb - lamb_pf) / (lamb_rf - lamb_pf))
                Mn_FLB = min(Mn_FLB, 1.6 * My_val)
                flb_case = "Noncompact Flange (F9-14)"
            else:
                Mn_FLB = (0.7 * E * Sxc) / (lamb**2)
                flb_case = "Slender Flange (F9-15)"
        else:
            Mn_FLB = Mn_LTB 
            flb_case = "Double-angle (refer F10.3)"

    # -------------------------------------------------
    # 8. Governing Nominal Strength
    # -------------------------------------------------
    Mn = min(Mp, Mn_LTB, Mn_FLB, Mn_LB_stem)

    return {
        "Mn": Mn,
        "Mp": Mp,
        "Mn_LTB": Mn_LTB,
        "Mn_LB_stem": Mn_LB_stem if Mn_LB_stem != float('inf') else "N/A",
        "Mn_FLB": Mn_FLB if Mn_FLB != float('inf') else "N/A",
        "cases": {
            "yielding": yielding_case,
            "ltb": ltb_case,
            "stem_lb": stem_lb_case,
            "flb": flb_case
        }
    }