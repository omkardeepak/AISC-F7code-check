import math

def flexural_strength_hss_f7(
    Fy, E, Ag,
    Sx, Sy,
    ry, J,
    b, tf, h, tw,
    Lb, Cb,
    Bw,
    My, Mz,
    axis_type="principal",        # "principal" or "geometric"
    toe_condition="compression", # "compression" or "tension"
    point_restraint=False        # True if restraint only at max moment
):

    t = tf
    rz = ry

    # ----------------------------------
    # Axis selection
    # ----------------------------------
    if abs(My) >= abs(Mz):
        bending_axis = "major"
        M = abs(My)
    else:
        bending_axis = "minor"
        M = abs(Mz)

    # ----------------------------------
    # Yielding (F10-1)
    # ----------------------------------
    Mp = 1.5 * M
    Mn_yield = Mp

    # ==================================
    # PRINCIPAL AXIS → F10-4
    # ==================================
    if axis_type == "principal":

        term = (Bw * rz) / (Lb * t)

        Mcr = (9 * E * Ag * rz * t * Cb) / (8 * Lb) * (
            math.sqrt(1 + (4.4 * term) ** 2) + 4.4 * term
        )

    # ==================================
    # GEOMETRIC AXIS → F10-5a / F10-5b
    # ==================================
    else:

        base = (0.58 * E * b**4 * Cb) / (Lb**2)
        inside = math.sqrt(1 + 0.88 * (Lb * t / b**2)**2)

        # F10-5a
        if toe_condition == "compression":
            Mcr = base * (inside - 1)

        # F10-5b
        else:
            Mcr = base * (inside + 1)

        # F10-5(ii)
        if point_restraint:
            Mcr *= 1.25

        # My modification per F10-5
        M = 0.80 * M

    # ----------------------------------
    # Nominal Flexural Strength
    # ----------------------------------
    ratio = M / Mcr

    if ratio <= 1.0:
        Mn_LTB = (1.92 - 1.17 * math.sqrt(ratio)) * M
        Mn_LTB = min(Mn_LTB, 1.5 * M)
        governing_ltb = "Inelastic LTB (F10-2)"
    else:
        Mn_LTB = (0.92 - 0.17 * (Mcr / M)) * Mcr
        governing_ltb = "Elastic LTB (F10-3)"

    # ----------------------------------
    # Governing Mn
    # ----------------------------------
    Mn = min(Mn_yield, Mn_LTB)

    if Mn == Mn_yield:
        governing_limit_state = "Yielding (F10-1)"
    else:
        governing_limit_state = governing_ltb

    return {
        "book": "AISC 360-22",
        "chapter": "F10",
        "check": "Flexural Strength",
        "method": "Single Angle",
        "Mn": Mn,
        "governing_limit_state": governing_limit_state,
        "Mp": Mp,
        "Mn_FLB": None,
        "Mn_WLB": None,
        "Mn_LTB": Mn_LTB,
        "bending_axis": bending_axis.capitalize(),
        "section_type": "Single Angle",
        "flange_compactness": None,
        "web_compactness": None,
        "status": "OK"
    }
