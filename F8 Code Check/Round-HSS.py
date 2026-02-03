import math

def flexural_strength_hss_f8_round(
    Fy, E,
    D, t,
    S, Z,
    My, Mz
):
    """
    AISC 360-16 | Chapter F8
    Flexural strength of ROUND HSS
    """

    # 1. Applicability
    if section.type != "SectionType.HSS":
        raise ValueError("Chapter F8 applies only to ROUND HSS sections")

    Dt = D / t

    if Dt >= 0.45 * E / Fy:
        raise ValueError("D/t exceeds applicability limit for Chapter F8")

    # 2. Bending Axis (required by framework)
    axis_enum, _, _ = classify_bending_axis(
        section=section,
        My=My,
        Mz=Mz
    )
    bending_axis = axis_enum.value  # 'major' or 'minor'
    # (Same properties for round HSS)

    # 3. Compactness Classification
    compactness = classify_section(
        section=section,
        material=material
    )

    overall_class = compactness["overall"].capitalize()

    # 4. Yielding (F8-1)
    Mn_yielding = Fy * Z

    # 5. Local Buckling (F8-2, F8-3)
    if overall_class == "Compact":
        Mn_local = Mn_yielding
        local_case = "Compact (No Local Buckling)"

    elif overall_class == "Noncompact":
        Mn_local = (0.021 * E / Dt + Fy) * S
        local_case = "Noncompact (F8-2)"

    elif overall_class == "Slender":
        Fcr = 0.33 * E / Dt
        Mn_local = Fcr * S
        local_case = "Slender (F8-3)"

    else:
        raise ValueError("Invalid compactness classification")

    # 6. Governing Nominal Strength
    Mn = min(Mn_yielding, Mn_local)

    governing_limit_state = (
        "Yielding"
        if Mn == Mn_yielding
        else f"Local Buckling – {local_case}"
    )

    # 7. Output (matches your schema)
    return {
        "book": "AISC 360-16",
        "chapter": "F8",
        "check": "Flexural Strength",
        "section_type": "Round HSS",
        "method": method,
        "Mn": Mn,
        "Mn_yielding": Mn_yielding,
        "Mn_local_buckling": Mn_local,
        "D_over_t": Dt,
        "compactness": overall_class,
        "governing_limit_state": governing_limit_state,
        "bending_axis": bending_axis.capitalize(),
        "status": "PASS"
    }
