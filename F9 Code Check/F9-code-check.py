import math
from typing import Dict, Any, List

# Core imports (same as F7)
from core.section import Section
from core.materials import Material
from core.enums import SectionType, BendingAxis
from design_code_checks.aisc_asd.utils import DEFAULTS
from global_utils.signed_max import signed_max

# Constants
OMEGA_B = DEFAULTS["ASD"]["Omega_flexure"]
PHI_B   = DEFAULTS["LRFD"]["phi_flexure"]


def flexural_strength_f9(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    Lb: float = 1.0,
    forces: List[float],
    bending_axis: BendingAxis,
    compactness: Dict[str, Any]
) -> Dict[str, Any]:
    """
    AISC 360-22 | Chapter F9
    Flexural strength of Tees and Double Angles loaded in the
    plane of symmetry.

    Applicable Limit States:
    1. Yielding                          (F9-1)
    2. Lateral-Torsional Buckling        (F9-6, F9-7, F9-13)
    3. Flange Local Buckling             (F9-14, F9-15)
    4. Local Buckling of Tee Stems /
       Double-Angle Web Legs             (F9-16 … F9-19)

    Sign convention for moment:
        Positive Mx  →  stem / web legs in TENSION
        Negative Mx  →  stem / web legs in COMPRESSION
    """

    # ==================================================================
    # 1. APPLICABILITY CHECK
    # ==================================================================
    if section.type not in [SectionType.TEE, SectionType.DOUBLE_ANGLE]:
        raise ValueError(
            f"Chapter F9 applies only to Tee and Double-Angle sections. "
            f"Got: {section.type}"
        )

    # ------------------------------------------------------------------
    # 2. MATERIAL & SECTION PROPERTIES
    # ------------------------------------------------------------------
    Fy = material.Fy
    E  = material.E

    Sx = section.Sx          # Elastic section modulus (about x)
    Zx = section.Zx          # Plastic section modulus (about x)
    Iy = section.Iy          # Moment of inertia about y
    ry = section.ry          # Radius of gyration about y
    J  = section.J           # Torsional constant
    Ag = section.A           # Gross area

    d  = section.dim_z       # Full depth of tee / double-angle
    tw = section.tw          # Stem / web thickness
    bf = section.dim_y       # Flange width
    tf = section.tf          # Flange thickness

    # ------------------------------------------------------------------
    # 3. DETERMINE STEM STRESS CONDITION
    # ------------------------------------------------------------------
    #    Positive demand moment → stem in tension
    #    Negative demand moment → stem in compression
    if bending_axis == BendingAxis.MAJOR:
        M_raw = signed_max(forces[5], forces[11])
    else:
        M_raw = signed_max(forces[4], forces[10])

    stem_in_tension = (M_raw >= 0)

    # ==================================================================
    # 4. LIMIT STATE 1 — YIELDING  (F9.1)
    # ==================================================================
    #   F9-3:  My = Fy · Sx
    My_val = Fy * Sx                                        # Eq. F9-3

    if section.type == SectionType.TEE:
        if stem_in_tension:
            Mp = min(Fy * Zx, 1.6 * My_val)                # Eq. F9-2
            yielding_case = "Tee stem in tension (F9-2)"
        else:
            Mp = My_val                                     # Eq. F9-4
            yielding_case = "Tee stem in compression (F9-4)"

    else:  # SectionType.DOUBLE_ANGLE
        if stem_in_tension:
            Mp = min(Fy * Zx, 1.6 * My_val)                # Eq. F9-2
            yielding_case = "Double-angle web legs in tension (F9-2)"
        else:
            Mp = 1.5 * My_val                               # Eq. F9-5
            yielding_case = "Double-angle web legs in compression (F9-5)"

    Mn_Yield = Mp                                           # Eq. F9-1

    # ==================================================================
    # 5. LIMIT STATE 2 — LATERAL-TORSIONAL BUCKLING  (F9.2)
    # ==================================================================
    Mn_LTB = None
    Mcr    = None
    B_val  = None
    Lp_val = None
    Lr_val = None
    ltb_case = "N/A"

    Lp_val = 1.76 * ry * math.sqrt(E / Fy)                 # Eq. F9-8

    if stem_in_tension:
        # ---- stems / web legs in TENSION ----
        B_val = 2.3 * (d / Lb) * math.sqrt(Iy / J)         # Eq. F9-11

        Mcr = (1.95 * E / Lb) * math.sqrt(Iy * J) * (
            B_val + math.sqrt(1 + B_val ** 2)
        )                                                    # Eq. F9-10

        Lr_val = 1.95 * (E / Fy) * math.sqrt(Iy * J) / Sx * math.sqrt(
            2.36 * (Fy / E) * (d * Sx / J) + 1
        )                                                    # Eq. F9-9

        if Lb <= Lp_val:
            # Plastic range — yielding governs
            Mn_LTB = None
            ltb_case = "Lb ≤ Lp — No LTB"
        elif Lb <= Lr_val:
            Mn_LTB = Mp - (Mp - My_val) * (
                (Lb - Lp_val) / (Lr_val - Lp_val)
            )                                                # Eq. F9-6
            Mn_LTB = min(Mn_LTB, Mp)
            ltb_case = "Inelastic LTB (F9-6)"
        else:
            Mn_LTB = Mcr                                    # Eq. F9-7
            Mn_LTB = min(Mn_LTB, Mp)
            ltb_case = "Elastic LTB (F9-7)"

    else:
        # ---- stems / web legs in COMPRESSION ----
        B_val = -2.3 * (d / Lb) * math.sqrt(Iy / J)        # Eq. F9-12

        Mcr = (1.95 * E / Lb) * math.sqrt(Iy * J) * (
            B_val + math.sqrt(1 + B_val ** 2)
        )                                                    # Eq. F9-10

        if section.type == SectionType.DOUBLE_ANGLE:
            # Double-angle web legs in compression:
            # Use Equations F10-2 and F10-3 with
            #   Mcr from F9-10 (computed above)
            #   My  from F9-3  (My_val)
            ratio_My_Mcr = My_val / Mcr

            if ratio_My_Mcr <= 1.0:
                # Inelastic LTB  — Eq. F10-2
                Mn_LTB = (1.92 - 1.17 * math.sqrt(ratio_My_Mcr)) * My_val
                Mn_LTB = min(Mn_LTB, 1.5 * My_val)
                ltb_case = "Double-angle web legs in compression LTB (F10-2 w/ F9-10, F9-3)"
            else:
                # Elastic LTB    — Eq. F10-3
                Mn_LTB = (0.92 - 0.17 * (Mcr / My_val)) * Mcr
                ltb_case = "Double-angle web legs in compression LTB (F10-3 w/ F9-10, F9-3)"

        else:
            # Tee stem in compression — Eq. F9-13
            Mn_LTB = min(Mcr, My_val)                       # Eq. F9-13
            ltb_case = "Tee stem in compression LTB (F9-13)"

    # ==================================================================
    # 6. LIMIT STATE 3 — FLANGE LOCAL BUCKLING  (F9.4)
    # ==================================================================
    Mn_FLB  = None
    Sxc     = Sx             # Elastic section modulus in compression at flange
    flb_case = "N/A"

    # Flange slenderness
    lambda_f  = bf / (2 * tf)
    lambda_pf = 0.38 * math.sqrt(E / Fy)
    lambda_rf = 1.0  * math.sqrt(E / Fy)

    flange_class = compactness["elements"]["flange"]["classification"]

    if flange_class == "Compact":
        # No FLB
        Mn_FLB = None
        flb_case = "Compact flange — No FLB"

    elif flange_class == "Noncompact":
        Mn_FLB = Mp - (Mp - 0.7 * Fy * Sxc) * (
            (lambda_f - lambda_pf) / (lambda_rf - lambda_pf)
        )                                                    # Eq. F9-14
        Mn_FLB = min(Mn_FLB, 1.6 * My_val)
        flb_case = "Noncompact flange (F9-14)"

    elif flange_class == "Slender":
        Mn_FLB = (0.7 * E * Sxc) / (lambda_f ** 2)          # Eq. F9-15
        flb_case = "Slender flange (F9-15)"

    #Need to add condition for double angle flange legs

    # ==================================================================
    # 7. LIMIT STATE 4 — LOCAL BUCKLING OF TEE STEMS /
    #                     DOUBLE-ANGLE WEB LEGS  (F9.3)
    # ==================================================================
    Mn_SLB  = None          # Stem / web-leg local buckling
    Fcr_stem = None
    slb_case = "N/A"

    if not stem_in_tension:
        # Stem / web legs are in compression → check local buckling
        lambda_s = d / tw
        lambda_ps = 0.84 * math.sqrt(E / Fy)      # Table B4.1b compact limit
        lambda_rs = 1.52 * math.sqrt(E / Fy)      # Table B4.1b noncompact limit

        stem_class = compactness["elements"]["web"]["classification"]

        if stem_class == "Compact":
            Fcr_stem = Fy                                    # Eq. F9-17
            slb_case = "Compact stem (F9-17)"

        elif stem_class == "Noncompact":
            Fcr_stem = (
                1.43 - 0.515 * lambda_s * math.sqrt(Fy / E)
            ) * Fy                                           # Eq. F9-18
            slb_case = "Noncompact stem (F9-18)"

        elif stem_class == "Slender":
            Fcr_stem = 1.52 * E / (lambda_s ** 2)            # Eq. F9-19
            slb_case = "Slender stem (F9-19)"

        if Fcr_stem is not None:
            Mn_SLB = Fcr_stem * Sx                           # Eq. F9-16

    # ==================================================================
    # 8. EQUATIONS REGISTER  (Chapter F9 — all referenced formulas)
    # ==================================================================
    # Helper flags for readability
    is_tee    = section.type == SectionType.TEE
    is_dbl    = section.type == SectionType.DOUBLE_ANGLE
    in_comp   = not stem_in_tension

    equations = {
        "F9-1": {
            "id":          "F9-1",
            "description": "Yielding — Mn = Mp",
            "value":       Mn_Yield,
            "applicable":  True,
        },
        "F9-2": {
            "id":          "F9-2",
            "description": "Plastic Moment — stems/web legs in tension",
            "value":       Mp if stem_in_tension else None,
            "applicable":  stem_in_tension,
        },
        "F9-3": {
            "id":          "F9-3",
            "description": "Yield Moment — My = Fy·Sx",
            "value":       My_val,
            "applicable":  True,
        },
        "F9-4": {
            "id":          "F9-4",
            "description": "Plastic Moment — Tee stem in compression",
            "value":       Mp if (is_tee and in_comp) else None,
            "applicable":  is_tee and in_comp,
        },
        "F9-5": {
            "id":          "F9-5",
            "description": "Plastic Moment — Double-angle web legs in compression",
            "value":       Mp if (is_dbl and in_comp) else None,
            "applicable":  is_dbl and in_comp,
        },
        "F9-6": {
            "id":          "F9-6",
            "description": "Inelastic LTB",
            "value":       Mn_LTB if "F9-6" in ltb_case else None,
            "applicable":  "F9-6" in ltb_case,
        },
        "F9-7": {
            "id":          "F9-7",
            "description": "Elastic LTB — Mn = Mcr",
            "value":       Mn_LTB if "F9-7" in ltb_case else None,
            "applicable":  "F9-7" in ltb_case,
        },
        "F9-8": {
            "id":          "F9-8",
            "description": "Lp = 1.76·ry·√(E/Fy)",
            "value":       Lp_val,
            "applicable":  True,
        },
        "F9-9": {
            "id":          "F9-9",
            "description": "Limiting unbraced length — Lr",
            "value":       Lr_val,
            "applicable":  Lr_val is not None,
        },
        "F9-10": {
            "id":          "F9-10",
            "description": "Critical elastic LTB moment — Mcr",
            "value":       Mcr,
            "applicable":  Mcr is not None,
        },
        "F9-11": {
            "id":          "F9-11",
            "description": "LTB parameter B (positive, stem in tension)",
            "value":       B_val if stem_in_tension else None,
            "applicable":  stem_in_tension,
        },
        "F9-12": {
            "id":          "F9-12",
            "description": "LTB parameter B (negative, stem in compression)",
            "value":       B_val if in_comp else None,
            "applicable":  in_comp,
        },
        "F9-13": {
            "id":          "F9-13",
            "description": "LTB Tee stem in compression — Mn = Mcr ≤ My",
            "value":       Mn_LTB if "F9-13" in ltb_case else None,
            "applicable":  "F9-13" in ltb_case,
        },
        "F10-2": {
            "id":          "F10-2",
            "description": "Inelastic LTB — Double-angle web legs in compression",
            "value":       Mn_LTB if "F10-2" in ltb_case else None,
            "applicable":  "F10-2" in ltb_case,
        },
        "F10-3": {
            "id":          "F10-3",
            "description": "Elastic LTB — Double-angle web legs in compression",
            "value":       Mn_LTB if "F10-3" in ltb_case else None,
            "applicable":  "F10-3" in ltb_case,
        },
        "F9-14": {
            "id":          "F9-14",
            "description": "Flange Local Buckling — Noncompact",
            "value":       Mn_FLB if flange_class == "Noncompact" else None,
            "applicable":  flange_class == "Noncompact",
        },
        "F9-15": {
            "id":          "F9-15",
            "description": "Flange Local Buckling — Slender",
            "value":       Mn_FLB if flange_class == "Slender" else None,
            "applicable":  flange_class == "Slender",
        },
        "F9-16": {
            "id":          "F9-16",
            "description": "Stem/Web-Leg Local Buckling — Mn = Fcr·Sx",
            "value":       Mn_SLB,
            "applicable":  Mn_SLB is not None,
        },
        "F9-17": {
            "id":          "F9-17",
            "description": "Stem Compact — Fcr = Fy",
            "value":       Fcr_stem if slb_case == "Compact stem (F9-17)" else None,
            "applicable":  slb_case == "Compact stem (F9-17)",
        },
        "F9-18": {
            "id":          "F9-18",
            "description": "Stem Noncompact",
            "value":       Fcr_stem if slb_case == "Noncompact stem (F9-18)" else None,
            "applicable":  slb_case == "Noncompact stem (F9-18)",
        },
        "F9-19": {
            "id":          "F9-19",
            "description": "Stem Slender — Fcr = 1.52·E / (d/tw)²",
            "value":       Fcr_stem if slb_case == "Slender stem (F9-19)" else None,
            "applicable":  slb_case == "Slender stem (F9-19)",
        },
    }

    # ==================================================================
    # 9. GOVERNING NOMINAL FLEXURAL STRENGTH
    # ==================================================================
    candidates = {
        "Yielding":                Mn_Yield,
        "Lateral-Torsional Buckling": Mn_LTB,
        "Flange Local Buckling":   Mn_FLB,
        "Stem Local Buckling":     Mn_SLB,
    }

    # Filter out Nones
    applicable = {k: v for k, v in candidates.items() if v is not None}

    if not applicable:
        governing_limit_state = "Unknown"
        Mn = 0.0
    else:
        governing_limit_state, Mn = min(
            applicable.items(), key=lambda item: item[1]
        )

    # ==================================================================
    # 10. DESIGN STRENGTH & UTILIZATION
    # ==================================================================
    if method.upper() == "ASD":
        factor = OMEGA_B
        Md = Mn / factor
    elif method.upper() == "LRFD":
        factor = PHI_B
        Md = factor * Mn
    else:
        raise ValueError(f"Unknown design method: {method}")

    # Demand
    if bending_axis == BendingAxis.MAJOR:
        M_demand = abs(signed_max(forces[5], forces[11]))
    else:
        M_demand = abs(signed_max(forces[4], forces[10]))

    # Unity / Utilization Ratio
    if Md <= 1e-6:
        UR = 999.0
    else:
        UR = M_demand / Md

    pass_check = UR <= 1.0

    # ==================================================================
    # 11. RETURN RESULTS
    # ==================================================================
    return {
        "element_id": element_id,
        "book": "AISC 360-22",
        "chapter": "F9",
        "check": "Flexural Strength",
        "method": method,
        "bending_axis":bending_axis.name if hasattr(bending_axis, "name") else str(bending_axis),
        "stem_condition": "Tension" if stem_in_tension else "Compression",
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Mp": Mp,
            "My": My_val,
            "Mn_LTB": Mn_LTB,
            "Mn_FLB": Mn_FLB,
            "Mn_SLB": Mn_SLB,
            "Mcr": Mcr,
            "B": B_val,
            "Lp": Lp_val,
            "Lr": Lr_val,
            "Lb": Lb,
            "Fcr_stem": Fcr_stem,
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if pass_check else "FAIL",
    }