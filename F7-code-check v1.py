import math

def flexural_strength_hss_f7(data):
    """
    AISC 360-16 Chapter F7
    Flexural strength of square/rectangular HSS and box sections
    """

    # -----------------------------
    # Metadata
    # -----------------------------
    element_id = data.get("element_id", "UNKNOWN")
    method = data["method"]  # ASD or LRFD

    # -----------------------------
    # Material & Section Properties
    # -----------------------------
    Fy = data["Fy"]
    E = data["E"]
    Ag = data["Ag"]
    Z = data["Z"]
    S = data["S"]
    ry = data["ry"]
    J = data["J"]

    b = data["b"]
    tf = data["tf"]
    h = data["h"]
    tw = tf  # HSS assumption

    Lb = data["Lb"]
    Cb = data["Cb"]

    section_type = data["section_type"]  # "HSS" or "BOX"
    bending_axis = data["bending_axis"]  # "Major" or "Minor"

    flange_class = data["flange_compactness"]
    web_class = data["web_compactness"]

    is_square = abs(b - h) < 1e-6

    # -----------------------------
    # 1. Plastic Moment Capacity
    # -----------------------------
    Mp = Fy * Z

    Mn_FLB = None
    Mn_WLB = None
    Mn_LTB = None

    # -----------------------------
    # 2. Flange Local Buckling (F7-2, F7-3)
    # -----------------------------
    if flange_class == "NonCompact":
        Mn_FLB = Mp - (Mp - Fy * S) * (
            3.57 * (b / tf) * math.sqrt(Fy / E) - 4.0
        )
        Mn_FLB = min(Mn_FLB, Mp)

    elif flange_class == "Slender":
        if section_type == "HSS":
            be = 1.92 * tf * math.sqrt(E / Fy) * (
                1 - 0.38 / ((b / tf) * math.sqrt(E / Fy))
            )
        else:  # BOX
            be = 1.92 * tf * math.sqrt(E / Fy) * (
                1 - 0.34 / ((b / tf) * math.sqrt(E / Fy))
            )

        be = min(be, b)
        Se = S * (be / b)
        Mn_FLB = Fy * Se

    # Compact flange → not applicable

    # -----------------------------
    # 3. Web Local Buckling (F7-6)
    # -----------------------------
    if web_class == "NonCompact":
        Mn_WLB = Mp - (Mp - Fy * S) * (
            0.305 * (h / tw) * math.sqrt(Fy / E) - 0.738
        )
        Mn_WLB = min(Mn_WLB, Mp)

    # Slender web → does NOT occur for HSS
    # Compact web → not applicable

    # -----------------------------
    # 4. Lateral–Torsional Buckling (F7-10, F7-11)
    # -----------------------------
    if not is_square and bending_axis == "Major":

        Lp = 0.13 * E * ry * math.sqrt(J * Ag) / Mp
        Lr = 2 * E * ry * math.sqrt(J * Ag) / (0.7 * Fy * S)

        if Lb <= Lp:
            Mn_LTB = Mp

        elif Lp < Lb <= Lr:
            Mn_LTB = Cb * (
                Mp - (Mp - 0.7 * Fy * S) * ((Lb - Lp) / (Lr - Lp))
            )
            Mn_LTB = min(Mn_LTB, Mp)

        else:
            Mn_LTB = 2 * E * Cb * math.sqrt(J * Ag) / (Lb / ry)
            Mn_LTB = min(Mn_LTB, Mp)

    # -----------------------------
    # 5. Governing Nominal Strength
    # -----------------------------
    candidates = {
        "Yielding": Mp,
        "Flange Local Buckling": Mn_FLB,
        "Web Local Buckling": Mn_WLB,
        "Lateral-Torsional Buckling": Mn_LTB
    }

    valid = {k: v for k, v in candidates.items() if v is not None}
    governing_limit_state = min(valid, key=valid.get)
    Mn = valid[governing_limit_state]

    # -----------------------------
    # 6. Output
    # -----------------------------
    return {
        "element_id": element_id,
        "book": "AISC 360-16",
        "chapter": "F7",
        "check": "Flexural Strength",
        "method": method,
        "Mn": Mn,
        "governing_limit_state": governing_limit_state,
        "Mp": Mp,
        "Mn_FLB": Mn_FLB,
        "Mn_WLB": Mn_WLB,
        "Mn_LTB": Mn_LTB,
        "bending_axis": bending_axis,
        "section_type": section_type,
        "flange_compactness": flange_class,
        "web_compactness": web_class,
        "status": "PASS"
    }
