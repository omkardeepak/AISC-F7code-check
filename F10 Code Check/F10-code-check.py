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


def flexural_strength_f10(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    Lb: float = 1.0,
    Cb: float = 1.0,
    forces: List[float],
    bending_axis: BendingAxis,
    compactness: Dict[str, Any],
    axis_type: str = "geometric",
    toe_condition: str = "compression",
    point_restraint: bool = False,
) -> Dict[str, Any]:
    """
    AISC 360-22 | Chapter F10
    Flexural strength of Single Angles.

    Applicable Limit States:
    1. Yielding                          (F10-1)
    2. Lateral-Torsional Buckling        (F10-2, F10-3)
    3. Leg Local Buckling                (F10-6, F10-7, F10-8)

    Parameters:
        axis_type:        "principal" or "geometric"
        toe_condition:    "compression" or "tension"
                          (only used when axis_type == "geometric")
        point_restraint:  True if lateral-torsional restraint is
                          provided only at the point of maximum moment
                          (only used when axis_type == "geometric")

    Mcr is determined from:
        Principal axis bending → Eq. F10-4
        Geometric axis bending → Eq. F10-5a / F10-5b
    """

    # ==================================================================
    # 1. APPLICABILITY CHECK
    # ==================================================================
    if section.type not in [SectionType.SINGLE_ANGLE]:
        raise ValueError(
            f"Chapter F10 applies only to Single Angle sections. "
            f"Got: {section.type}"
        )

    # ------------------------------------------------------------------
    # 2. MATERIAL & SECTION PROPERTIES
    # ------------------------------------------------------------------
    Fy = material.Fy
    E  = material.E

    Sx = section.Sx          # Elastic section modulus (about x)
    Sy = section.Sy          # Elastic section modulus (about y)
    Ag = section.A           # Gross area
    ry = section.ry          # Radius of gyration about y-axis
    rz = section.rz          # Radius of gyration about minor principal axis (z)
    J  = section.J           # Torsional constant

    b  = section.dim_y       # Leg width (long leg for unequal angles)
    t  = section.tf          # Leg thickness
    Bw = section.Bw          # Section property for single-angle flexure (±)

    # ------------------------------------------------------------------
    # 3. DETERMINE BENDING AXIS & DEMAND
    # ------------------------------------------------------------------
    if bending_axis == BendingAxis.MAJOR:
        S = Sx
        M_demand = abs(signed_max(forces[5], forces[11]))
    else:
        S = Sy
        M_demand = abs(signed_max(forces[4], forces[10]))

    My_val = Fy * S                                          # Yield moment

    # ==================================================================
    # 4. LIMIT STATE 1 — YIELDING  (F10.1)
    # ==================================================================
    Mp = 1.5 * My_val                                        # Eq. F10-1
    Mn_Yield = Mp

    # ==================================================================
    # 5. LIMIT STATE 2 — LATERAL-TORSIONAL BUCKLING  (F10.2)
    # ==================================================================
    Mcr    = None
    Mn_LTB = None
    ltb_case = "N/A"

    if axis_type == "principal":
        # ---- PRINCIPAL AXIS BENDING 
        term = (Bw * rz) / (Lb * t)

        Mcr = (9 * E * Ag * rz * t * Cb) / (8 * Lb) * (
            math.sqrt(1 + (4.4 * term) ** 2) + 4.4 * term
        )                                                    # Eq. F10-4

        mcr_case = "Principal axis (F10-4)"

    else:
        # ---- GEOMETRIC AXIS BENDING 
        base   = (0.58 * E * b ** 4 * Cb) / (Lb ** 2)
        inside = math.sqrt(1 + 0.88 * (Lb * t / b ** 2) ** 2)

        if toe_condition == "compression":
            Mcr = base * (inside - 1)                        # Eq. F10-5a
            mcr_case = "Geometric axis, toe in compression (F10-5a)"
        else:
            Mcr = base * (inside + 1)                        # Eq. F10-5b
            mcr_case = "Geometric axis, toe in tension (F10-5b)"

        # (ii) If lateral-torsional restraint is at the point of
        #      maximum moment only, Mcr may be increased by 25%
        if point_restraint:
            Mcr *= 1.25
            mcr_case += " [×1.25 point restraint]"

        # For geometric axis bending
        My_val = 0.80 * My_val                               # F10-5 note

    # ---- Determine Mn_LTB using F10-2 / F10-3 ----
    ratio = My_val / Mcr

    if ratio <= 1.0:
        # Inelastic LTB — Eq. F10-2
        Mn_LTB = (1.92 - 1.17 * math.sqrt(ratio)) * My_val
        Mn_LTB = min(Mn_LTB, 1.5 * My_val)                  # Eq. F10-2
        ltb_case = "Inelastic LTB (F10-2)"
    else:
        # Elastic LTB — Eq. F10-3
        Mn_LTB = (0.92 - 0.17 * (Mcr / My_val)) * Mcr       # Eq. F10-3
        ltb_case = "Elastic LTB (F10-3)"

    # ==================================================================
    # 6. LIMIT STATE 3 — LEG LOCAL BUCKLING  (F10.3)
    # ==================================================================
    Mn_LLB   = None
    Fcr_leg  = None
    llb_case = "N/A"

    lambda_leg = b / t
    leg_class  = compactness["elements"]["leg"]["classification"]

    if leg_class == "Compact":
        Mn_LLB  = None
        llb_case = "Compact leg — No LLB"

    elif leg_class == "Noncompact":
        # Eq. F10-6: 
        Sc = S       # Elastic section modulus to toe in compression
        Mn_LLB = Fy * Sc * (2.43 - 1.72 * lambda_leg * math.sqrt(Fy / E))
        llb_case = "Noncompact leg (F10-6)"

    elif leg_class == "Slender":
        # Eq. F10-7:
        Fcr_leg = 0.71 * E / (lambda_leg ** 2)               # Eq. F10-7

        # Eq. F10-8:  Mn = Fcr · Sc
        Sc = S
        Mn_LLB = Fcr_leg * Sc                                # Eq. F10-8
        llb_case = "Slender leg (F10-7 / F10-8)"

    # ==================================================================
    # 7. EQUATIONS REGISTER  (Chapter F10 — all referenced formulas)
    # ==================================================================
    equations = {
        "F10-1": {
            "id":          "F10-1",
            "description": "Yielding — Mn = 1.5·My",
            "value":       Mn_Yield,
            "applicable":  True,
        },
        "F10-2": {
            "id":          "F10-2",
            "description": "Inelastic LTB",
            "value":       Mn_LTB if "F10-2" in ltb_case else None,
            "applicable":  "F10-2" in ltb_case,
        },
        "F10-3": {
            "id":          "F10-3",
            "description": "Elastic LTB",
            "value":       Mn_LTB if "F10-3" in ltb_case else None,
            "applicable":  "F10-3" in ltb_case,
        },
        "F10-4": {
            "id":          "F10-4",
            "description": "Mcr — Principal axis bending",
            "value":       Mcr if axis_type == "principal" else None,
            "applicable":  axis_type == "principal",
        },
        "F10-5a": {
            "id":          "F10-5a",
            "description": "Mcr — Geometric axis, toe in compression",
            "value":       Mcr if (axis_type == "geometric" and toe_condition == "compression") else None,
            "applicable":  axis_type == "geometric" and toe_condition == "compression",
        },
        "F10-5b": {
            "id":          "F10-5b",
            "description": "Mcr — Geometric axis, toe in tension",
            "value":       Mcr if (axis_type == "geometric" and toe_condition == "tension") else None,
            "applicable":  axis_type == "geometric" and toe_condition == "tension",
        },
        "F10-6": {
            "id":          "F10-6",
            "description": "Leg Local Buckling — Noncompact",
            "value":       Mn_LLB if leg_class == "Noncompact" else None,
            "applicable":  leg_class == "Noncompact",
        },
        "F10-7": {
            "id":          "F10-7",
            "description": "Critical Buckling Stress — Slender Leg",
            "value":       Fcr_leg if leg_class == "Slender" else None,
            "applicable":  leg_class == "Slender",
        },
        "F10-8": {
            "id":          "F10-8",
            "description": "Leg Local Buckling — Slender",
            "value":       Mn_LLB if leg_class == "Slender" else None,
            "applicable":  leg_class == "Slender",
        },
    }

    # ==================================================================
    # 8. GOVERNING NOMINAL FLEXURAL STRENGTH
    # ==================================================================
    candidates = {
        "Yielding":                Mn_Yield,
        "Lateral-Torsional Buckling": Mn_LTB,
        "Leg Local Buckling":      Mn_LLB,
    }

    applicable = {k: v for k, v in candidates.items() if v is not None}

    if not applicable:
        governing_limit_state = "Unknown"
        Mn = 0.0
    else:
        governing_limit_state, Mn = min(
            applicable.items(), key=lambda item: item[1]
        )

    # ==================================================================
    # 9. DESIGN STRENGTH & UTILIZATION
    # ==================================================================
    if method.upper() == "ASD":
        factor = OMEGA_B
        Md = Mn / factor
    elif method.upper() == "LRFD":
        factor = PHI_B
        Md = factor * Mn
    else:
        raise ValueError(f"Unknown design method: {method}")

    if Md <= 1e-6:
        UR = 999.0
    else:
        UR = M_demand / Md

    pass_check = UR <= 1.0

    # ==================================================================
    # 10. RETURN RESULTS
    # ==================================================================
    return {
        "element_id": element_id,
        "book": "AISC 360-22",
        "chapter": "F10",
        "check": "Flexural Strength",
        "method": method,
        "bending_axis": bending_axis.name if hasattr(bending_axis, "name") else str(bending_axis),
        "toe_condition": toe_condition if axis_type == "geometric" else "N/A",
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Mp": Mp,
            "My": My_val,
            "Mn_LTB": Mn_LTB,
            "Mn_LLB": Mn_LLB,
            "Mcr": Mcr,
            "Cb": Cb,
            "Lb": Lb,
            "Fcr_leg": Fcr_leg,
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if pass_check else "FAIL",
    }
