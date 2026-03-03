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


def flexural_strength_f13(
    *,
    element_id: str,
    section: Section,
    material: Material,
    method: str = "ASD",
    forces: List[float],
    bending_axis: BendingAxis,
    # --- F13.1: Bolt Holes in Tension Flange ---
    Afg: Optional[float] = None,
    Afn: Optional[float] = None,
    Mn_chapter: Optional[float] = None,
    # --- F13.2: Proportioning Limits (I-Shaped) ---
    symmetry: str = "doubly",
    web_class: str = "compact",
    a: Optional[float] = None,
    # --- F13.3: Cover Plates ---
    has_cover_plate: bool = False,
    w_cover: Optional[float] = None,
    end_weld_condition: Optional[str] = None,
    t_plate: Optional[float] = None,
    t_weld_end: Optional[float] = None,
) -> Dict[str, Any]:
    """
    AISC 360-22 | Chapter F13
    Proportions of Beams and Girders.

    Applicable Limit States / Checks:
    1. Tensile Rupture of Tension Flange (Bolt Holes)    (F13-1)
    2. Proportioning Limits — Iyc/Iy (Singly Symmetric)  (F13-2)
    3. Proportioning Limits — Slender Web h/tw            (F13-3, F13-4)
    4. Cover Plate Weld Termination Length                 (F13-5, F13-6, F13-7)
    """

    # ==================================================================
    # 1. MATERIAL & SECTION PROPERTIES
    # ==================================================================
    Fy = material.Fy
    Fu = material.Fu
    E  = material.E

    Sx = section.Sx

    h  = section.dim_z - 2 * section.tf
    tw = section.tw
    Iy = section.Iy
    h_tw = h / tw

    equations = {}
    prop_checks = []

    # ==================================================================
    # 2. F13.1 — BOLT HOLES IN TENSION FLANGE
    # ==================================================================
    Mn_F13   = None
    Yt       = None
    f13_1_case = "N/A — No bolt hole data provided"

    if Afg is not None and Afn is not None:
        # Determine Yt
        if Fy / Fu <= 0.8:
            Yt = 1.0
        else:
            Yt = 1.1

        Fu_Afn    = Fu * Afn
        Yt_Fy_Afg = Yt * Fy * Afg

        if Fu_Afn >= Yt_Fy_Afg:
            # (a) Tensile rupture does not apply
            f13_1_case = "Fu·Afn ≥ Yt·Fy·Afg — Tensile rupture does not apply"
            Mn_F13 = None
        else:
            # (b) Mn capped by Eq. F13-1
            Mn_F13 = (Fu * Afn / Afg) * Sx                  # Eq. F13-1
            f13_1_case = "Fu·Afn < Yt·Fy·Afg — Tensile rupture governs (F13-1)"

        equations["F13-1"] = {
            "id":          "F13-1",
            "description": "Tensile Rupture — Mn = (Fu·Afn / Afg) · Sx",
            "value":       Mn_F13,
            "applicable":  Mn_F13 is not None,
        }

    # ==================================================================
    # 3. GOVERNING NOMINAL FLEXURAL STRENGTH
    # ==================================================================
    candidates = {}

    if Mn_chapter is not None:
        candidates["Chapter Flexural Strength"] = Mn_chapter
    if Mn_F13 is not None:
        candidates["Tensile Rupture of Tension Flange"] = Mn_F13

    applicable_mn = {k: v for k, v in candidates.items() if v is not None}

    if not applicable_mn:
        governing_limit_state = "N/A"
        Mn = Mn_chapter if Mn_chapter is not None else 0.0
    else:
        governing_limit_state, Mn = min(
            applicable_mn.items(), key=lambda item: item[1]
        )

    # ==================================================================
    # 4. DESIGN STRENGTH & UTILIZATION
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
    # 5. F13.2 — PROPORTIONING LIMITS FOR I-SHAPED MEMBERS
    # ==================================================================
    Iyc_ratio    = None
    h_tw_max     = None
    aw           = None
    f13_2_pass   = True
    slender_pass = True
    aw_pass      = True

    # --- Iyc / Iy check (singly symmetric only) --- Eq. F13-2
    if symmetry == "singly":
        bfc = section.bfc if hasattr(section, "bfc") else section.dim_y
        tfc = section.tfc if hasattr(section, "tfc") else section.tf

        Iyc = (bfc ** 3 * tfc) / 12
        Iyc_ratio = Iyc / Iy
        f13_2_pass = 0.1 <= Iyc_ratio <= 0.9

        prop_checks.append({
            "id":     "F13-2",
            "check":  "0.1 ≤ Iyc/Iy ≤ 0.9",
            "Iyc":    Iyc,
            "Iy":     Iy,
            "ratio":  Iyc_ratio,
            "status": "PASS" if f13_2_pass else "FAIL",
        })

        equations["F13-2"] = {
            "id":          "F13-2",
            "description": "Singly symmetric — 0.1 ≤ Iyc/Iy ≤ 0.9",
            "value":       Iyc_ratio,
            "applicable":  True,
        }

    # --- Slender web limits --- Eqs. F13-3, F13-4
    slender_eq = "N/A"

    if web_class == "slender":
        if a is not None:
            a_h = a / h
            if a_h <= 1.5:
                h_tw_max   = 12.0 * math.sqrt(E / Fy)       # Eq. F13-3
                slender_eq = "F13-3"
            else:
                h_tw_max   = 0.40 * E / Fy                   # Eq. F13-4
                slender_eq = "F13-4"
        else:
            # Unstiffened girder
            h_tw_max   = 260.0
            slender_eq = "Unstiffened (h/tw ≤ 260)"

        slender_pass = h_tw <= h_tw_max

        prop_checks.append({
            "id":       slender_eq,
            "check":    "h/tw ≤ (h/tw)_max",
            "h_tw":     h_tw,
            "h_tw_max": h_tw_max,
            "a":        a,
            "a_h":      a / h if a is not None else None,
            "status":   "PASS" if slender_pass else "FAIL",
        })

        equations["F13-3"] = {
            "id":          "F13-3",
            "description": "Slender web, a/h ≤ 1.5 — (h/tw)_max = 12·√(E/Fy)",
            "value":       h_tw_max if "F13-3" in slender_eq else None,
            "applicable":  "F13-3" in slender_eq,
        }
        equations["F13-4"] = {
            "id":          "F13-4",
            "description": "Slender web, a/h > 1.5 — (h/tw)_max = 0.40·E/Fy",
            "value":       h_tw_max if "F13-4" in slender_eq else None,
            "applicable":  "F13-4" in slender_eq,
        }

    # --- aw ≤ 10 (per Eq. F4-12) ---
    bfc_aw = section.bfc if hasattr(section, "bfc") else section.dim_y
    tfc_aw = section.tfc if hasattr(section, "tfc") else section.tf
    aw     = (h * tw) / (bfc_aw * tfc_aw)
    aw_pass = aw <= 10.0

    prop_checks.append({
        "id":     "F4-12 (aw limit)",
        "check":  "aw ≤ 10",
        "aw":     aw,
        "status": "PASS" if aw_pass else "FAIL",
    })

    prop_all_pass = f13_2_pass and slender_pass and aw_pass

    # ==================================================================
    # 6. F13.3 — COVER PLATES
    # ==================================================================
    a_prime          = None
    cover_eq_used    = None
    cover_condition  = None

    if has_cover_plate and w_cover is not None and end_weld_condition is not None:
        if end_weld_condition == "full":
            # (1) Weld ≥ ¾·t across end
            a_prime       = w_cover                          # Eq. F13-5
            cover_eq_used = "F13-5"
            cover_condition = "Weld ≥ ¾·t across end of plate"

        elif end_weld_condition == "partial":
            # (2) Weld < ¾·t across end
            a_prime       = 1.5 * w_cover                    # Eq. F13-6
            cover_eq_used = "F13-6"
            cover_condition = "Weld < ¾·t across end of plate"

        elif end_weld_condition == "none":
            # (3) No weld across end
            a_prime       = 2.0 * w_cover                    # Eq. F13-7
            cover_eq_used = "F13-7"
            cover_condition = "No weld across end of plate"

        else:
            raise ValueError(
                f"Unknown end_weld_condition: '{end_weld_condition}'. "
                f"Must be 'full', 'partial', or 'none'."
            )

        equations["F13-5"] = {
            "id":          "F13-5",
            "description": "Weld ≥ ¾·t across end — a' = w",
            "value":       a_prime if cover_eq_used == "F13-5" else None,
            "applicable":  cover_eq_used == "F13-5",
        }
        equations["F13-6"] = {
            "id":          "F13-6",
            "description": "Weld < ¾·t across end — a' = 1.5w",
            "value":       a_prime if cover_eq_used == "F13-6" else None,
            "applicable":  cover_eq_used == "F13-6",
        }
        equations["F13-7"] = {
            "id":          "F13-7",
            "description": "No weld across end — a' = 2w",
            "value":       a_prime if cover_eq_used == "F13-7" else None,
            "applicable":  cover_eq_used == "F13-7",
        }

    # ==================================================================
    # 7. RETURN RESULTS
    # ==================================================================
    return {
        "element_id": element_id,
        "book": "AISC 360-22",
        "chapter": "F13",
        "check": "Proportions of Beams and Girders",
        "method": method,
        "bending_axis": bending_axis.name if hasattr(bending_axis, "name") else str(bending_axis),
        "M_demand": M_demand,
        "Mn": Mn,
        "Md": Md,
        "governing_limit_state": governing_limit_state,
        "details": {
            "Fy": Fy,
            "Fu": Fu,
            "Sx": Sx,
            "h": h,
            "tw": tw,
            "h_tw": h_tw,
            "h_tw_max": h_tw_max,
            "prop_all_pass": prop_all_pass,
            "a_prime": a_prime,
            "w_cover": w_cover,
        },
        "equations": equations,
        "candidates": candidates,
        "UR": UR,
        "status": "PASS" if (pass_check and prop_all_pass) else "FAIL",
    }
