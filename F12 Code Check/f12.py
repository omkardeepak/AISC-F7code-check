import math
from typing import Dict, Any, List, Optional

# Core imports (same as F7)
from core.section import Section
from core.materials import Material
from core.enums import SectionType, BendingAxis
from design_code_checks.aisc_asd.utils import DEFAULTS
from global_utils.signed_max import signed_max

# Constants
OMEGA_B = DEFAULTS["ASD"]["Omega_flexure"]
PHI_B   = DEFAULTS["LRFD"]["phi_flexure"]


def flexural_strength_f12(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    forces: List[float],
    bending_axis: BendingAxis,
    Fcr_ltb: Optional[float] = None,
    Fcr_lb: Optional[float] = None,
    Smin: Optional[float] = None,
) -> Dict[str, Any]:
    """
    AISC 360-22 | Chapter F12
    Flexural strength of Unsymmetrical Shapes.

    This section applies to all unsymmetrical shapes, except
    single angles (covered by Section F10), that are not covered
    by Sections F1 through F11.

    The nominal flexural strength, Mn, shall be the lowest value
    obtained according to the limit states of yielding (yield
    moment), lateral-torsional buckling, and local buckling where:
        Mn = Fn · Smin                                   (F12-1)

    where
        Smin = minimum elastic section modulus relative to
               the axis of bending, in.³ (mm³)

    Applicable Limit States:
    1. Yielding                                          (F12-2)
       Fn = Fy

    2. Lateral-Torsional Buckling                        (F12-3)
       Fn = Fcr ≤ Fy
       where Fcr = lateral-torsional buckling stress for the
                   section as determined by analysis, ksi (MPa)

    3. Local Buckling                                    (F12-4)
       Fn = Fcr ≤ Fy
       where Fcr = local buckling stress for the section as
                   determined by analysis, ksi (MPa)

    User Note:
        In the case of Z-shaped members, it is recommended that
        Fcr be taken as 0.5·Fcr of a channel with the same flange
        and web properties.

    Parameters:
        Smin:     Minimum elastic section modulus relative to the
                  axis of bending, in.³ (mm³).  For unsymmetric
                  shapes the neutral axis is NOT at the mid-depth,
                  so the section modulus differs at each extreme
                  fiber.  The SMALLER value shall be used.
                  If None, falls back to section.Sx / section.Sy.
        Fcr_ltb:  Lateral-torsional buckling stress determined
                  by analysis, ksi (MPa).
                  If None, LTB is skipped.
        Fcr_lb:   Local buckling stress determined by analysis
                  or from applicable provisions in Table B4.1b,
                  ksi (MPa).
                  If None, local buckling is skipped.
    """

    # ==================================================================
    # 1. APPLICABILITY CHECK
    # ==================================================================
    if section.type == SectionType.SINGLE_ANGLE:
        raise ValueError(
            f"Chapter F12 does not apply to Single Angles "
            f"(use Chapter F10 instead). Got: {section.type}"
        )

    # ------------------------------------------------------------------
    # 2. MATERIAL & SECTION PROPERTIES
    # ------------------------------------------------------------------
    Fy = material.Fy
    E  = material.E

    # Smin — minimum elastic section modulus about the axis of bending.
    # For unsymmetric shapes the NA is not at mid-depth, so S differs
    # at the two extreme fibers; F12 requires the SMALLER value.
    if Smin is not None:
        S = Smin
    elif bending_axis == BendingAxis.MAJOR:
        S = section.Sx
    else:
        S = section.Sy

    # ==================================================================
    # 3. LIMIT STATE 1 — YIELDING  (F12.1)
    #    Fn = Fy                                            Eq. F12-2
    #    Mn = Fn · Smin = Fy · Smin                         Eq. F12-1
    # ==================================================================
    Fn_yield = Fy                                            # Eq. F12-2
    Mn_Yield = Fn_yield * S                                  # Eq. F12-1

    # ==================================================================
    # 4. LIMIT STATE 2 — LATERAL-TORSIONAL BUCKLING  (F12.2)
    #    Fn = Fcr ≤ Fy                                      Eq. F12-3
    #    Mn = Fn · Smin                                     Eq. F12-1
    # ==================================================================
    Mn_LTB   = None
    Fn_ltb   = None
    ltb_case = "N/A"

    if Fcr_ltb is not None:
        Fn_ltb = min(Fcr_ltb, Fy)                            # Eq. F12-3
        Mn_LTB = Fn_ltb * S                                  # Eq. F12-1

        if Fcr_ltb >= Fy:
            ltb_case = "LTB does not govern (Fcr ≥ Fy)"
            Mn_LTB = None
        else:
            ltb_case = "Lateral-Torsional Buckling (F12-3)"
    else:
        ltb_case = "Fcr_ltb not provided — LTB not checked"

    # ==================================================================
    # 5. LIMIT STATE 3 — LOCAL BUCKLING  (F12.3)
    #    Fn = Fcr ≤ Fy                                      Eq. F12-4
    #    Mn = Fn · Smin                                     Eq. F12-1
    # ==================================================================
    Mn_LB    = None
    Fn_lb    = None
    lb_case  = "N/A"

    if Fcr_lb is not None:
        Fn_lb = min(Fcr_lb, Fy)                              # Eq. F12-4
        Mn_LB = Fn_lb * S                                    # Eq. F12-1

        if Fcr_lb >= Fy:
            lb_case = "Local Buckling does not govern (Fcr ≥ Fy)"
            Mn_LB = None
        else:
            lb_case = "Local Buckling (F12-4)"
    else:
        lb_case = "Fcr_lb not provided — LB not checked"

    # ==================================================================
    # 6. EQUATIONS REGISTER  (Chapter F12 — all referenced formulas)
    # ==================================================================
    equations = {
        "F12-1": {
            "id":          "F12-1",
            "description": "Nominal Flexural Strength ",
            "value":       None,    # populated below after governing Mn
            "applicable":  True,
        },
        "F12-2": {
            "id":          "F12-2",
            "description": "Yielding — Fn = Fy",
            "value":       Fn_yield,
            "applicable":  True,
        },
        "F12-3": {
            "id":          "F12-3",
            "description": "Lateral-Torsional Buckling ",
            "value":       Fn_ltb,
            "applicable":  Fn_ltb is not None and Fn_ltb < Fy,
        },
        "F12-4": {
            "id":          "F12-4",
            "description": "Local Buckling p",
            "value":       Fn_lb,
            "applicable":  Fn_lb is not None and Fn_lb < Fy,
        },
    }

    # ==================================================================
    # 7. GOVERNING NOMINAL FLEXURAL STRENGTH
    # ==================================================================
    candidates = {
        "Yielding":                   Mn_Yield,
        "Lateral-Torsional Buckling": Mn_LTB,
        "Local Buckling":             Mn_LB,
    }

    applicable = {k: v for k, v in candidates.items() if v is not None}

    if not applicable:
        governing_limit_state = "Unknown"
        Mn = 0.0
    else:
        governing_limit_state, Mn = min(
            applicable.items(), key=lambda item: item[1]
        )

    # Back-fill F12-1 with the governing Mn
    equations["F12-1"]["value"] = Mn

    # ==================================================================
    # 8. DESIGN STRENGTH & UTILIZATION
    # ==================================================================
    if method.upper() == "ASD":
        factor = OMEGA_B
        Md = Mn / factor
    elif method.upper() == "LRFD":
        factor = PHI_B
        Md = factor * Mn
    else:
        raise ValueError(f"Unknown design method: {method}")

    if bending_axis == BendingAxis.MAJOR:
        M_demand = abs(signed_max(forces[5], forces[11]))
    else:
        M_demand = abs(signed_max(forces[4], forces[10]))

    if Md <= 1e-6:
        UR = 999.0
    else:
        UR = M_demand / Md

    pass_check = UR <= 1.0

    # ==================================================================
    # 9. RETURN RESULTS
    # ==================================================================
    return {
        "element_id": element_id,
        "book": "AISC 360-22",
        "chapter": "F12",
        "check": "Flexural Strength",
        "method": method,
        "bending_axis": bending_axis.name if hasattr(bending_axis, "name") else str(bending_axis),
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Smin": S,
            "Fy": Fy,
            "Fn_yield": Fn_yield,
            "Fn_ltb": Fn_ltb,
            "Fn_lb": Fn_lb,
            "Fcr_ltb": Fcr_ltb,
            "Fcr_lb": Fcr_lb,
            "Mn_Yield": Mn_Yield,
            "Mn_LTB": Mn_LTB,
            "Mn_LB": Mn_LB,
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if pass_check else "FAIL",
    }
