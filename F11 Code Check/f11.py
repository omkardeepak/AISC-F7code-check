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


def flexural_strength_f11(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    Lb: float = 1.0,
    Cb: float = 1.0,
    forces: List[float],
    bending_axis: BendingAxis,
) -> Dict[str, Any]:
    """
    AISC 360-22 | Chapter F11
    Flexural strength of Rectangular Bars and Rounds.

    Applicable Limit States:
    1. Yielding                          (F11-1)
    2. Lateral-Torsional Buckling        (F11-2, F11-3)

    Notes:
        - LTB does NOT apply to:
            • Rounds
            • Square bars
            • Rectangular bars bent about their minor axis
        - LTB applies only to rectangular bars (d ≠ t)
          bent about the major axis.
    """

    # ==================================================================
    # 1. APPLICABILITY CHECK
    # ==================================================================
    if section.type not in [SectionType.RECT_BAR, SectionType.ROUND_BAR]:
        raise ValueError(
            f"Chapter F11 applies only to Rectangular Bars and Rounds. "
            f"Got: {section.type}"
        )

    # ------------------------------------------------------------------
    # 2. MATERIAL & SECTION PROPERTIES
    # ------------------------------------------------------------------
    Fy = material.Fy
    E  = material.E

    is_round = section.type == SectionType.ROUND_BAR

    if is_round:
        S = section.Sx
        Z = section.Zx
        d = section.dim_z          # Diameter
        t = d                      # For rounds, t = d (no LTB applies)
    else:
        # Rectangular bar
        if bending_axis == BendingAxis.MAJOR:
            S = section.Sx
            Z = section.Zx
            d = section.dim_z      # Depth (larger dimension)
            t = section.dim_y      # Width / thickness (smaller dimension)
        else:
            S = section.Sy
            Z = section.Zy
            d = section.dim_y      # Width becomes "depth" for minor axis
            t = section.dim_z      # Depth becomes "width" for minor axis

    My_val = Fy * S                                          # Yield moment

    # ==================================================================
    # 3. LIMIT STATE 1 — YIELDING  (F11.1)
    # ==================================================================
    Mp = min(Fy * Z, 1.6 * My_val)                           # Eq. F11-1
    Mn_Yield = Mp

    # ==================================================================
    # 4. LIMIT STATE 2 — LATERAL-TORSIONAL BUCKLING  (F11.2)
    # ==================================================================
    Mn_LTB  = None
    Fcr     = None
    ltb_case = "N/A"

    # LTB applies only to rectangular bars bent about the major axis
    # It does NOT apply to rounds, square bars, or minor-axis bending
    is_square  = (not is_round) and math.isclose(d, t, rel_tol=1e-9)
    ltb_applies = (
        not is_round
        and not is_square
        and bending_axis == BendingAxis.MAJOR
    )

    if ltb_applies:
        # LTB parameter
        Lbd_t2 = Lb * d / (t ** 2)

        limit_compact    = 0.08 * E / Fy
        limit_noncompact = 1.9  * E / Fy

        if Lbd_t2 <= limit_compact:
            # Plastic range — yielding governs, no LTB
            Mn_LTB = None
            ltb_case = "Lb·d/t² ≤ 0.08·E/Fy — No LTB"

        elif Lbd_t2 <= limit_noncompact:
            # Inelastic LTB — Eq. F11-2
            Mn_LTB = Cb * (1.52 - 0.274 * Lbd_t2 * (Fy / E)) * My_val
            Mn_LTB = min(Mn_LTB, Mp)                         # Eq. F11-2
            ltb_case = "Inelastic LTB (F11-2)"

        else:
            # Elastic LTB — Eq. F11-3, F11-4
            Fcr = 1.9 * E * Cb / Lbd_t2                      # Eq. F11-4
            Mn_LTB = Fcr * S                                  # Eq. F11-3
            Mn_LTB = min(Mn_LTB, Mp)
            ltb_case = "Elastic LTB (F11-3)"

    else:
        if is_round:
            ltb_case = "Round — LTB does not apply"
        elif is_square:
            ltb_case = "Square bar — LTB does not apply"
        else:
            ltb_case = "Minor axis bending — LTB does not apply"

    # ==================================================================
    # 5. EQUATIONS REGISTER  (Chapter F11 — all referenced formulas)
    # ==================================================================
    equations = {
        "F11-1": {
            "id":          "F11-1",
            "description": "Yielding — Mn = Mp = Fy·Z ≤ 1.6·My",
            "value":       Mn_Yield,
            "applicable":  True,
        },
        "F11-2": {
            "id":          "F11-2",
            "description": "Inelastic LTB",
            "value":       Mn_LTB if "F11-2" in ltb_case else None,
            "applicable":  "F11-2" in ltb_case,
        },
        "F11-3": {
            "id":          "F11-3",
            "description": "Elastic LTB — Mn = Fcr·Sx",
            "value":       Mn_LTB if "F11-3" in ltb_case else None,
            "applicable":  "F11-3" in ltb_case,
        },
        "F11-4": {
            "id":          "F11-4",
            "description": "Critical Stress — Fcr = 1.9·E·Cb / (Lb·d/t²)",
            "value":       Fcr,
            "applicable":  Fcr is not None,
        },
    }

    # ==================================================================
    # 6. GOVERNING NOMINAL FLEXURAL STRENGTH
    # ==================================================================
    candidates = {
        "Yielding":                   Mn_Yield,
        "Lateral-Torsional Buckling": Mn_LTB,
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
    # 7. DESIGN STRENGTH & UTILIZATION
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
    # 8. RETURN RESULTS
    # ==================================================================
    return {
        "element_id": element_id,
        "book": "AISC 360-22",
        "chapter": "F11",
        "check": "Flexural Strength",
        "method": method,
        "bending_axis": bending_axis.name if hasattr(bending_axis, "name") else str(bending_axis),
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Mp": Mp,
            "My": My_val,
            "Mn_LTB": Mn_LTB,
            "Fcr": Fcr,
            "Cb": Cb,
            "Lb": Lb,
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if pass_check else "FAIL",
    }
