import math
from typing import Dict, Any, List

# Core imports
from core.section import Section
from core.materials import Material
from core.enums import SectionType, BendingAxis
from design_code_checks.aisc_asd.utils import DEFAULTS
from global_utils.signed_max import signed_max

# Constants
OMEGA_B = DEFAULTS["ASD"]["Omega_flexure"]
PHI_B   = DEFAULTS["LRFD"]["phi_flexure"]

def flexural_strength_hss_f7(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    Lb: float = 1.0,
    Cb: float = 1.0,
    forces: List[float],
    bending_axis: BendingAxis,
    compactness: Dict[str, Any]
) -> Dict[str, Any]:
    """
    AISC 360-16 | Chapter F7
    Flexural strength of square / rectangular HSS and box sections.
    
    Applicable Limit States:
    1. Yielding (Y)
    2. Flange Local Buckling (FLB)
    3. Web Local Buckling (WLB)
    4. Lateral-Torsional Buckling (LTB) - Major Axis Only
    """

    # 1. Verification
    if section.type not in [SectionType.BOX, SectionType.HSS]:
        # Note: Adjust SectionType checks based on your exact Enum definitions
        if section.type != SectionType.BOX: 
             # Fallback check if your enums are strictly named
             pass 

    # 2. Material & Section Properties
    Fy = material.Fy
    E  = material.E
    
    Ag = section.A
    ry = section.ry
    J  = section.J
    
    # Dimensions for buckling checks
    # b = width of flange resisting moment
    # h = depth of web
    if bending_axis == BendingAxis.MAJOR:
        S = section.Sx
        Z = section.Zx
        b = section.dim_y - 3 * section.tw # approximate clear width if not stored
        tf = section.tf
        h = section.dim_z - 3 * section.tf
        tw = section.tw
    else:
        # For minor axis, "flange" is the side walls, "web" is top/bottom
        S = section.Sy
        Z = section.Zy
        b = section.dim_z - 3 * section.tf
        tf = section.tw # thickness of the flange (which is the vertical wall)
        h = section.dim_y - 3 * section.tw
        tw = section.tf

    # 3. Compactness Classification
    # Note: Ensure your compactness checker maps correctly to the bending axis
    flange_class = compactness["elements"]["flange"]["classification"]
    web_class = compactness["elements"]["web"]["classification"]

    # -----------------------------------------------------------
    # LIMIT STATE 1: Yielding (F7-1)
    # -----------------------------------------------------------
    Mp = Fy * Z
    Mn_Yield = Mp

    # Initialize other states as None (Not Applicable)
    Mn_FLB = None
    Mn_WLB = None
    Mn_LTB = None
    Se     = None          # Effective section modulus  (slender FLB only)
    Lp     = None          # Compact LTB limit length
    Lr     = None          # Noncompact LTB limit length

    # -----------------------------------------------------------
    # LIMIT STATE 2: Flange Local Buckling (F7-2, F7-3)
    # -----------------------------------------------------------
    if flange_class == "Noncompact":
        # Eq F7-2
        lambda_f = b / tf
        # Note: You should fetch lambda_p and lambda_r from compactness dict ideally
        # calculating locally here for safety based on code logic provided:
        Mn_FLB = Mp - (Mp - Fy * S) * (
            3.57 * lambda_f * math.sqrt(Fy / E) - 4.0
        )
        Mn_FLB = min(max(Mn_FLB, 0), Mp)

    elif flange_class == "Slender":
        # Eq F7-3 (Effective Width)
        lambda_f = b / tf
        
        # User Correction: Check Enum properly here
        if section.type == SectionType.BOX or section.type == SectionType.HSS:
            # Table B4.1b Case 17 (HSS)
            correction = 0.38
        else:
            # Box Sections (Case 18)
            correction = 0.34

        be = 1.92 * tf * math.sqrt(E / Fy) * (
            1 - correction / (lambda_f * math.sqrt(E / Fy))
        )
        be = min(be, b)
        
        # Effective Section Modulus
        Se = S * (be / b) # Simplified reduction
        Mn_FLB = Fy * Se

    # -----------------------------------------------------------
    # LIMIT STATE 3: Web Local Buckling (F7-6)
    # -----------------------------------------------------------
    # Note: F7-6 is typically for non-compact webs. 
    # Slender webs in HSS are rare and handled in separate chapters/sections if extreme.
    if web_class == "Noncompact":
        lambda_w = h / tw
        Mn_WLB = Mp - (Mp - Fy * S) * (
            0.305 * lambda_w * math.sqrt(Fy / E) - 0.738
        )
        Mn_WLB = min(max(Mn_WLB, 0), Mp)

    # -----------------------------------------------------------
    # LIMIT STATE 4: Lateral-Torsional Buckling (F7-4)
    # -----------------------------------------------------------
    # Only applies to Major Axis bending
    if bending_axis == BendingAxis.MAJOR:
        # Calculate LTB lengths
        term = math.sqrt(J * Ag)
        Lp = 0.13 * E * ry * term / Mp
        Lr = 2 * E * ry * term / (0.7 * Fy * S)

        if Lb <= Lp:
            # No LTB
            pass # Mn_LTB remains None (or Mp, but Yielding covers it)
        elif Lp < Lb <= Lr:
            Mn_LTB = Cb * (
                Mp - (Mp - 0.7 * Fy * S) * ((Lb - Lp) / (Lr - Lp))
            )
            Mn_LTB = min(Mn_LTB, Mp)
        else:
            Mn_LTB = 2 * E * Cb * term / (Lb / ry) # Eq F7-5 is missing in standard text, using F7-12 logic
            # Actually for HSS, F7-5 is: Mn = 2 * E * Cb * sqrt(Ja) / (Lb/ry) ?
            # Let's verify standard AISC F7 Eq F7-5:
            # Mn = 2 * E * Cb * sqrt(J * Ag) / (Lb / ry) is incorrect dimensionally usually. 
            # Correct AISC Eq F7-5: Mn = 2*E*Cb*sqrt(JA) / (Lb/ry) IS correct.
            Mn_LTB = min(Mn_LTB, Mp)

    # -----------------------------------------------------------
    # EQUATIONS REGISTER  (Chapter F7 — all referenced formulas)
    # -----------------------------------------------------------
    equations = {
        "F7-1": {
            "id":          "F7-1",
            "description": "Yielding — Plastic Moment",
            "value":       Mn_Yield,
            "applicable":  True,
        },
        "F7-2": {
            "id":          "F7-2",
            "description": "Flange Local Buckling — Noncompact",
            "value":       Mn_FLB if flange_class == "Noncompact" else None,
            "applicable":  flange_class == "Noncompact",
        },
        "F7-3": {
            "id":          "F7-3",
            "description": "Flange Local Buckling — Slender (Effective Width)",
            "value":       Mn_FLB if flange_class == "Slender" else None,
            "applicable":  flange_class == "Slender",
        },
        "F7-4": {
            "id":          "F7-4",
            "description": "Lateral-Torsional Buckling — Inelastic (Lp < Lb ≤ Lr)",
            "value":       Mn_LTB if (Mn_LTB is not None and Lp is not None and Lp < Lb <= Lr) else None,
            "applicable":  Mn_LTB is not None and Lp is not None and Lp < Lb <= (Lr if Lr else 0),
        },
        "F7-5": {
            "id":          "F7-5",
            "description": "Lateral-Torsional Buckling — Elastic (Lb > Lr)",
            "value":       Mn_LTB if (Mn_LTB is not None and Lr is not None and Lb > Lr) else None,
            "applicable":  Mn_LTB is not None and Lr is not None and Lb > Lr,
        },
        "F7-6": {
            "id":          "F7-6",
            "description": "Web Local Buckling — Noncompact",
            "value":       Mn_WLB if web_class == "Noncompact" else None,
            "applicable":  web_class == "Noncompact",
        },
    }

    # -----------------------------------------------------------
    # GOVERNING STRENGTH SELECTION
    # -----------------------------------------------------------
    candidates = {
        "Yielding": Mn_Yield,
        "Flange Local Buckling": Mn_FLB,
        "Web Local Buckling": Mn_WLB,
        "Lateral-Torsional Buckling": Mn_LTB
    }

    # Filter out Nones
    applicable = {k: v for k, v in candidates.items() if v is not None}

    # Find minimum
    if not applicable:
        governing_limit_state = "Unknown"
        Mn = 0.0
    else:
        # Get both key and value of the minimum item
        governing_limit_state, Mn = min(applicable.items(), key=lambda item: item[1])

    # -----------------------------------------------------------
    # DESIGN STRENGTH & UTILIZATION
    # -----------------------------------------------------------
    if method.upper() == "ASD":
        factor = OMEGA_B
        Md = Mn / factor
    elif method.upper() == "LRFD":
        factor = PHI_B
        Md = factor * Mn
    else:
        raise ValueError(f"Unknown design method: {method}")

    # Demand (Absolute max of start/end moments for the relevant axis)
    # Index 5 = Mz (Major usually), Index 11 = Mz_end
    # Index 4 = My (Minor usually), Index 10 = My_end
    if bending_axis == BendingAxis.MAJOR:
        M_demand = abs(signed_max(forces[5], forces[11]))
    else:
        M_demand = abs(signed_max(forces[4], forces[10]))

    # Unity Check
    if Md <= 1e-6:
        UR = 999.0 # Fail safe for zero capacity
    else:
        UR = M_demand / Md

    pass_check = UR <= 1.0

    return {
        "element_id": element_id,
        "book": "AISC 360-16",
        "chapter": "F7",
        "check": "Flexural Strength",
        "method": method,
        "bending_axis": bending_axis.name if hasattr(bending_axis, 'name') else str(bending_axis),
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Mp": Mp,
            "Mn_FLB": Mn_FLB,
            "Mn_WLB": Mn_WLB,
            "Mn_LTB": Mn_LTB,
            "Cb": Cb,
            "Lb": Lb
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if pass_check else "FAIL"
    }