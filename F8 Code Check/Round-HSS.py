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

def flexural_strength_hss_f8(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    forces: List[float],
    bending_axis: BendingAxis,
    compactness: Dict[str, Any]
) -> Dict[str, Any]:
    """
    AISC 360-22 | Chapter F8
    Flexural strength of Round HSS (Hollow Structural Sections).

    Applicable Limit States:
    1. Yielding              (F8-1)
    2. Local Buckling         (F8-2 Noncompact, F8-3 Slender)

    Notes:
    - Lateral-Torsional Buckling does NOT apply to Round HSS.
    - Round HSS are symmetric about all bending axes, so the same
      section properties (S, Z) govern regardless of axis.
    - Applicability limit: D/t <= 0.45 * E / Fy
    """

    # ------------------------------------------------------------------
    # 1. APPLICABILITY CHECK
    # ------------------------------------------------------------------
    if section.type != SectionType.HSS:
        raise ValueError(
            f"Chapter F8 applies only to Round HSS sections. "
            f"Got: {section.type}"
        )

    # Material & Section Properties
    Fy = material.Fy
    E  = material.E

    D  = section.dim_z          # Outside diameter
    t  = section.tw             # Wall thickness

    # Round HSS: identical properties about both axes
    S = section.Sx              # Elastic section modulus
    Z = section.Zx              # Plastic section modulus

    # Diameter-to-thickness ratio
    Dt = D / t

    # Upper applicability limit per F8
    if Dt > 0.45 * E / Fy:
        raise ValueError(
            f"D/t = {Dt:.2f} exceeds the Chapter F8 applicability "
            f"limit of 0.45·E/Fy = {0.45 * E / Fy:.2f}"
        )

    # ------------------------------------------------------------------
    # 2. COMPACTNESS CLASSIFICATION  (Table B4.1b, Case 20)
    # ------------------------------------------------------------------
    #   Compact:     D/t <= 0.07 E/Fy        (λp)
    #   Noncompact:  0.07 E/Fy < D/t <= 0.31 E/Fy  (λr)
    #   Slender:     0.31 E/Fy < D/t <= 0.45 E/Fy
    # ------------------------------------------------------------------
    overall_class = compactness["overall"].capitalize()

    # ------------------------------------------------------------------
    # 3. LIMIT STATE 1 — Yielding  (Eq. F8-1)
    # ------------------------------------------------------------------
    Mp = Fy * Z
    Mn_Yield = Mp

    # ------------------------------------------------------------------
    # 4. LIMIT STATE 2 — Local Buckling  (Eq. F8-2, F8-3)
    # ------------------------------------------------------------------
    Mn_LB = None          # None ➜ not applicable (compact section)
    Fcr   = None          # Critical buckling stress (slender only)

    if overall_class == "Compact":
        # No local buckling; yielding governs
        Mn_LB = None
        local_case = "Compact – No Local Buckling"

    elif overall_class == "Noncompact":
        # Eq. F8-2:  Mn = ( 0.021·E / (D/t)  +  Fy ) · S
        Mn_LB = (0.021 * E / Dt + Fy) * S
        local_case = "Noncompact (Eq. F8-2)"

    elif overall_class == "Slender":
        # Eq. F8-3:  Fcr = 0.33·E / (D/t)
        # Eq. F8-4:  Mn  = Fcr · S
        Fcr = 0.33 * E / Dt
        Mn_LB = Fcr * S
        local_case = "Slender (Eq. F8-3 / F8-4)"

    else:
        raise ValueError(
            f"Invalid compactness classification: '{overall_class}'"
        )

    # ------------------------------------------------------------------
    # 4b. EQUATIONS REGISTER  (Chapter F8 — all referenced formulas)
    # ------------------------------------------------------------------
    equations = {
        "F8-1": {
            "id":          "F8-1",
            "description": "Yielding — Plastic Moment",
            "value":       Mn_Yield,
            "applicable":  True,           # always computed
        },
        "F8-2": {
            "id":          "F8-2",
            "description": "Local Buckling — Noncompact Section",
            "value":       Mn_LB if overall_class == "Noncompact" else None,
            "applicable":  overall_class == "Noncompact",
        },
        "F8-3": {
            "id":          "F8-3",
            "description": "Critical Buckling Stress — Slender Section",
            "value":       Fcr if overall_class == "Slender" else None,
            "applicable":  overall_class == "Slender",
        },
        "F8-4": {
            "id":          "F8-4",
            "description": "Local Buckling — Slender Section",
            "value":       Mn_LB if overall_class == "Slender" else None,
            "applicable":  overall_class == "Slender",
        },
    }

    # ------------------------------------------------------------------
    # 5. GOVERNING NOMINAL FLEXURAL STRENGTH
    # ------------------------------------------------------------------
    candidates = {
        "Yielding": Mn_Yield,
        "Local Buckling": Mn_LB
    }

    # Filter out Nones (compact → LB not applicable)
    applicable = {k: v for k, v in candidates.items() if v is not None}

    if not applicable:
        governing_limit_state = "Unknown"
        Mn = 0.0
    else:
        governing_limit_state, Mn = min(
            applicable.items(), key=lambda item: item[1]
        )

    # ------------------------------------------------------------------
    # 6. DESIGN STRENGTH & UTILIZATION
    # ------------------------------------------------------------------
    if method.upper() == "ASD":
        factor = OMEGA_B
        Md = Mn / factor
    elif method.upper() == "LRFD":
        factor = PHI_B
        Md = factor * Mn
    else:
        raise ValueError(f"Unknown design method: {method}")

    # Demand — absolute maximum moment from start / end forces
    #   Index 5 = Mz (Major), Index 11 = Mz_end
    #   Index 4 = My (Minor), Index 10 = My_end
    if bending_axis == BendingAxis.MAJOR:
        M_demand = abs(signed_max(forces[5], forces[11]))
    else:
        M_demand = abs(signed_max(forces[4], forces[10]))

    # Unity / Utilization Ratio
    if Md <= 1e-6:
        UR = 999.0          # Fail-safe for zero capacity
    else:
        UR = M_demand / Md

    pass_check = UR <= 1.0

    # ------------------------------------------------------------------
    # 7. RETURN RESULTS
    # ------------------------------------------------------------------
    return {
        "element_id": element_id,
        "book": "AISC 360-22",
        "chapter": "F8",
        "check": "Flexural Strength",
        "method": method,
        "bending_axis": (
            bending_axis.name
            if hasattr(bending_axis, "name")
            else str(bending_axis)
        ),
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Mp": Mp,
            "Mn_LB": Mn_LB,
            "local_buckling_case": local_case,
            "D": D,
            "t": t,
            "D_over_t": Dt,
            "compactness": overall_class,
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if pass_check else "FAIL",
    }
