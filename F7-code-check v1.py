import math

def flexural_strength_hss_f7(
    Fy, E, Ag,
    Sx, Sy,
    ry, J,
    b, tf, h, tw,
    Lb, Cb,
    My, Mz
):
    """
    AISC 360-16 | Chapter F7
    Flexural strength of square / rectangular HSS and box sections
    """

    # -------------------------------------------------
    # 1. Verify Applicability
    # -------------------------------------------------
    if section.type != "SectionType.HSS":
        raise ValueError("Chapter F7 applies only to HSS / BOX sections")

    section_type = section.type
    is_square = abs(b - h) < 1e-6

    # -------------------------------------------------
    # 2. Identify Bending Axis
    # -------------------------------------------------
    axis_enum, _, _ = classify_bending_axis(section=section, My=My, Mz=Mz)
    bending_axis = axis_enum.value  # "major" or "minor"

    if bending_axis == "major":
        S = Sx
        Z = section.Zx
    else:
        S = Sy
        Z = section.Zy

    # -------------------------------------------------
    # 3. Compactness Classification (B4.1)
    # -------------------------------------------------
    compactness = classify_section(section=section, material=material)

    flange_class = compactness["elements"]["flange"]["classification"].capitalize()
    web_class = compactness["elements"]["web"]["classification"].capitalize()

    # -------------------------------------------------
    # 4. Plastic Moment Capacity (Yielding)
    # -------------------------------------------------
    Mp = Fy * Z

    Mn_FLB = None
    Mn_WLB = None
    Mn_LTB = None

    # -------------------------------------------------
    # 5. Flange Local Buckling (F7-2, F7-3)
    # -------------------------------------------------
    if flange_class == "Noncompact":
        Mn_FLB = Mp - (Mp - Fy * S) * (
            3.57 * (b / tf) * math.sqrt(Fy / E) - 4.0
        )
        Mn_FLB = min(Mn_FLB, Mp)

    elif flange_class == "Slender":
        if section_type == "SectionType.HSS":
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

    # Compact flange → FLB not applicable

    # -------------------------------------------------
    # 6. Web Local Buckling (F7-6)
    # -------------------------------------------------
    if web_class == "Noncompact":
        Mn_WLB = Mp - (Mp - Fy * S) * (
            0.305 * (h / tw) * math.sqrt(Fy / E) - 0.738
        )
        Mn_WLB = min(Mn_WLB, Mp)

    # Slender web does not occur for HSS per AISC note

    # -------------------------------------------------
    # 7. Lateral–Torsional Buckling (F7-10, F7-11)
    # -------------------------------------------------
    if not is_square and bending_axis == "major":

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

    # -------------------------------------------------
    # 8. Governing Nominal Flexural Strength
    # -------------------------------------------------
    candidates = {
        "Yielding": Mp,
        "Flange Local Buckling": Mn_FLB,
        "Web Local Buckling": Mn_WLB,
        "Lateral-Torsional Buckling": Mn_LTB
    }

    applicable = {k: v for k, v in candidates.items() if v is not None}
    governing_limit_state = min(applicable, key=applicable.get)
    Mn = applicable[governing_limit_state]

    # -------------------------------------------------
    # 9. Output (As per your required schema)
    # -------------------------------------------------
    return {
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
        "bending_axis": bending_axis.capitalize(),
        "section_type": section_type,
        "flange_compactness": flange_class,
        "web_compactness": web_class,
        "status": "PASS"
    }
