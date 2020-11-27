"""Moduł obliczeniowy, który przetwarza dane i sprawdza dobrany przekrój."""

from math import ceil, exp, pi, sqrt, atan, sin, cos, degrees as deg, radians as rad

# Przeliczanie jednostek układu SI do podstawowych
m = 1
cm = 1e-2
mm = 1e-3
cm2 = 1e-4
mm2 = 1e-6
N = 1
kN = 1e3
Pa = N / m ** 2
kPa = kN / m ** 2
MPa = N / mm ** 2
GPa = 1e9

A_s = lambda fi: pi * (fi / 2) ** 2


def wymiarowanie(
    g_k: float,
    q_k: float,
    kat: str,
    l: float,
    b: float,
    h: float,
    s: str,
    n_1: int,
    n_2: int,
    n_3: int,
    fi_1: float,
    fi_2: float,
    fi_3: float,
    fi_r: float,
    n_k: int,
    h_k: float,
    fi_d: float,
    fi_k: float,
    fi_g: float,
    bet: str,
    c_min_dur: float,
    delta_c_dev: float,
    b_w: float,
    h_st: float,
) -> (str, str):
    """Wymiarowanie zbrojenia do schematu belki jednoprzęsłowej, swobodnie podpartej."""
    warn = ""

    # Otulina, przyjęte zbrojenie i wysokość użyteczna (p. 4.4 [2])
    c_min_b = max(fi_1, fi_2) - fi_r
    c_min = max(c_min_b, c_min_dur, 10 * mm)
    c_nom = c_min + delta_c_dev
    l_p = (ceil(round(l + 12 * cm, 3) / cm / 10)) * 10  # długość płyty w cm
    b_p = round(b / cm)  # szerokość płyty w cm
    h_p = max(45 * mm, c_nom + 2 * fi_1 + fi_r, c_nom + fi_2 + 10 * mm + fi_r)
    A_s_1 = n_1 * A_s(fi_1)
    A_s_2 = n_2 * A_s(fi_2)
    A_s_k = n_k * 2 * A_s(fi_d)
    A_s_3 = n_3 * A_s(fi_3)
    A_s_prov = A_s_1 + A_s_2 + A_s_k + A_s_3
    a_1 = c_nom + fi_1 / 2 + fi_r
    a_2 = c_nom + fi_2 / 2 + fi_r
    a_k = c_nom + fi_d / 2 + fi_r
    a_3 = h_p + fi_3 / 2
    a_mean = (A_s_1 * a_1 + A_s_2 * a_2 + A_s_k * a_k + A_s_3 * a_3) / A_s_prov
    d = h - a_mean

    # Wkłady styropianowe (zastępcza szerokość z równości momentów bezwładności [7])
    if s == "true":
        b_st = 5 * cm  # odstęp pomiędzy krawędzią płyty i licem wkładu styropianowego
        if b_w < 70 * mm + 2 * c_nom:
            raise Exception("Za mała szerokość żeber usztywniających.")
        if h - h_p - h_st < 50 * mm:
            raise Exception("Za mała grubość nadbetonu nad wkładami styropianowymi.")
        if n_k == 1 and (b - b_w - 2 * b_st) / 2 < max(h_st, 85 * mm):
            raise Exception("Nie można stosować wkładów do podanej szerokości płyty.")
        A_c = b * h
        y_c = h / 2
        J_c = b * h ** 3 / 12
        A_st = (b - n_k * b_w - 2 * b_st) * h_st
        y_st = h_p + h_st / 2
        J_st = (b - n_k * b_w - 2 * b_st) * h_st ** 3 / 12
        y = (A_c * y_c - A_st * y_st) / (A_c - A_st)
        J_z = J_c - J_st + A_c * (y_c - y) ** 2 - A_st * (y_st - y) ** 2
        b = 12 * J_z / h ** 3

    # Materiały (p. 3 oraz załącznik B [2])
    t = 50 * 365  # wiek betonu w rozważanej chwili w dniach
    tp = 7  # wiek betonu na końcu okresu produkcji i montażu w dniach
    ts = 5 + tp  # wiek betonu na końcu okresu pielęgnacji w dniach
    t0 = 28 + tp  # wiek betonu w chwili przyłożenia obciążenia
    RH = 80  # wilgotność powietrza zewnętrznego w procentach
    f_ck = int(bet) * MPa
    f_cd = f_ck / 1.4
    f_cm = f_ck + 8 * MPa
    f_ctm = 0.3 * (f_ck / MPa) ** (2 / 3) * MPa
    f_ctk_05 = 0.7 * f_ctm
    f_ctd = f_ctk_05 / 1.4
    f_bd = 2.25 * f_ctd
    E_cm = 22 * (0.1 * f_cm / MPa) ** 0.3 * GPa
    h_0 = h / mm
    fi_RH = 1 + (1 - RH / 100) / (0.1 * (h_0) ** (1 / 3))
    beta_fcm = 16.8 / sqrt(f_cm / MPa)
    beta_t0 = 1 / (0.1 + t0 ** 0.2)
    fi_0 = fi_RH * beta_fcm * beta_t0
    beta_H = min(1.5 * (1 + (0.012 * RH) ** 18) * h_0 + 250, 1500)
    beta_c_t_t0 = ((t - t0) / (beta_H + t - t0)) ** 0.3
    fi_t_t0 = fi_0 * beta_c_t_t0
    E_c_eff = E_cm / (1 + fi_t_t0)
    f_yk = 500 * MPa  # stal zbrojeniowa klasy C (A-IIIN)
    f_yd = f_yk / 1.15
    E_s = 200 * GPa
    alfa_e = E_s / E_c_eff
    ro = A_s_prov / (b * d)
    if alfa_e * ro <= 0.06:
        z = 0.9 * d
    elif alfa_e * ro <= 0.18:
        z = 0.85 * d
    else:
        z = 0.8 * d

    # Statyka
    p_k = (g_k + q_k) / kPa  # całkowite obciążenie char. ponad ciężar własny w kN/m2
    g_k, q_k = b * g_k, b * q_k  # przeliczenie obciążenia na metr bieżący przekroju
    psi_0 = {  # tab. A 1.1 [1]
        **dict.fromkeys(["A", "B", "C", "D", "F", "G", "S1"], 0.7),
        **{"E": 1.0, "H": 0, "S2": 0.5},
    }
    psi_2 = {  # tab. A 1.1 [1]
        **dict.fromkeys(["A", "B"], 0.3),
        **dict.fromkeys(["C", "D", "F"], 0.6),
        **{"E": 0.8, "G": 0.3, "H": 0, "S1": 0.2, "S2": 0.2},
    }
    g_k += 24.5 * kPa * h * b  # dodanie ciężaru własnego do obciążeń stałych
    if s == "true":
        gamma_s = max((1 - (2 * 0.5 * m + max(l // (2 * m), 1) * 0.25 * m) / l), 0)
        g_k -= gamma_s * 24.5 * kPa * ((b - n_k * b_w - 2 * b_st) * h_st)
    p = max(  # 6.10a/b [1]
        g_k * 1.35 + q_k * 1.5 * psi_0[kat], g_k * 1.35 * 0.85 + q_k * 1.5
    )
    p_q = g_k + q_k * psi_2[kat]  # 6.16b [1]
    l_eff = (l_p * cm + l) / 2
    M_Ed = 0.125 * p * l_eff ** 2
    V_Ed = 0.5 * p * l_eff
    l_eff = l  # p. 8 [3]
    M_Ed_q = 0.125 * p_q * l_eff ** 2

    # Warunek ULS (SGN) - Zginanie (tab. 7.2 i p. 7.1.1 [5])
    mi = M_Ed / (b * d ** 2 * f_cd)
    if mi > 0.371:  # tab. 7.2 [5]
        warn += "<font color=red>Przekroczno wartość graniczną mi.<br>"
    omega = 0.9731 - sqrt(0.9469 - 1.946 * mi)
    A_s_req = max(
        omega * b * d * (f_cd / f_yd), 0.26 * (f_ctm / f_yk) * b * d, 0.0013 * b * d
    )
    if A_s_prov < A_s_req:
        warn += f"<font color=red>Zbyt mały stopień zbrojenia przekroju = {ro:.2%}.<br>"
    A_c = A_c - A_st if s == "true" else b * d
    if A_s_prov > 0.04 * A_c or A_s_prov > 0.5 * (f_cd / f_yd) * b * d:
        warn += f"<font color=red>Zbyt duży stopień zbrojenia przekroju = {ro:.2%}.<br>"
    M_Rd = A_s_prov * f_yd * (d - 0.5138 * ((A_s_prov * f_yd) / (b * f_cd)))
    if M_Ed > M_Rd:
        warn += "<font color=red>Warunek nośności na zginanie niespełniony.<br>"
    if A_s_2 != 0 and A_s_3 == 0:
        M_Rd_2 = (
            (A_s_prov - A_s_2)
            * f_yd
            * (d - 0.5138 * (((A_s_prov - A_s_2) * f_yd) / (b * f_cd)))
        )
        sigma_sd = M_Ed / (z * A_s_prov)
        l_b_rqd = (fi_2 / 4) * (sigma_sd / f_bd)
        l_bd = max(l_b_rqd, 10 * fi_2, 100 * mm)
        x = (V_Ed - sqrt(2) * sqrt(-M_Rd_2 * p + 0.5 * V_Ed ** 2)) / p - l_bd
        l_2 = f"na {round(l_p - 2 * x / cm, -1):.0f} cm"
    else:
        l_2 = ""

    # Warunek ULS (SGN) - Ścinanie (tab. 10.2 [5])
    k = min(1 + sqrt(20 / (d / mm)), 2.0)
    ro_v = (A_s_prov - A_s_2) / (b * d)
    v_Rd_c = (0.18 / 1.4) * k * (100 * ro_v * (f_ck / MPa)) ** (1 / 3) * MPa
    v_min = 0.035 * sqrt(k ** 3 * f_ck / MPa) * MPa
    v_Rd_c = max(v_Rd_c, v_min)
    V_Rd_c = v_Rd_c * b * d
    if V_Ed > V_Rd_c and h >= 20 * cm:
        warn += "<font color=orange>Element wymagający zbrojenie na ścinanie.<br>"
    elif V_Ed > V_Rd_c and h < 20 * cm:
        warn += "<font color=red>Warunek nośności na ścinanie niespełniony.<br>"

    # Warunek ULS (SGN) - Rozwarstwienie (p. 6.2.5 [2])
    beta = 1 if f_yd * A_s_prov / (f_cd * b) < (h - h_p) else (h - h_p) / h
    b_i = n_k * b_w if s == "true" else b
    v_Ed_i = beta * V_Ed / (z * b_i)
    c, mi = 0.4, 0.7  # powierzchnie szorstkie, grabione
    sigma_n = min(p_q / b_i, 0.6 * f_cd)
    A_s_i = n_k * 2 * A_s(fi_k)
    s_k = 200 * mm  # rozstaw krzyżulców kratownicy
    A_i = b_i * s_k
    a = deg(atan(s_k / 2 / h_k))
    v = 0.6 * (1 - f_ck / MPa / 250)
    v_Rd_i = min(
        c * f_ctd
        + mi * sigma_n
        + A_s_i / A_i * f_yd * (mi * sin(rad(a)) + cos(rad(a))),
        0.5 * v * f_cd,
    )
    if v_Ed_i > v_Rd_i:
        warn += "<font color=red>Warunek nośności na rozwarstwienie niespełniony.<br>"

    # Ugięcia w stanie zarysowania (p. 7.4.3 [2])
    x_I = (0.5 * b * h ** 2 + alfa_e * A_s_prov * d) / (b * h + alfa_e * A_s_prov)
    J = b * h ** 3 / 12
    J_I = J + b * h * (x_I - h / 2) ** 2 + alfa_e * A_s_prov * (d - x_I) ** 2
    alfa_k = 1 / 10  # p. 5.8.8.2(4) [2]
    alfa_I = alfa_k * (M_Ed_q * l_eff ** 2) / (E_c_eff * J_I)
    x_II = d * ((alfa_e ** 2 * ro ** 2 + 2 * alfa_e * ro) ** 0.5 - alfa_e * ro)
    J_II = b * x_II ** 3 / 3 + alfa_e * ro * b * d * (d - x_II) ** 2
    alfa_II = alfa_k * (M_Ed_q * l_eff ** 2) / (E_c_eff * J_II)
    M_cr = f_ctm * J_I / (h - x_I)
    if M_cr > M_Ed_q:
        dzeta = 0
    else:
        dzeta = 1 - 0.5 * (M_cr / M_Ed_q) ** 2
    alfa = dzeta * alfa_II + (1 - dzeta) * alfa_I

    # Ugięcia od skurczu (p. 3.1.4 i p. 7.4.3 [2] oraz całe [6])
    h_0 = 2 * h  # wysychanie płyty może zachodzić tylko z jednej strony
    beta_ds_t_ts = (t - ts) / ((t - ts) + 0.04 * sqrt(h_0 ** 3))
    if h_0 < 200 * mm:
        k_h = ((0.85 - 1) * h_0 + 0.2 * 1 - 0.1 * 0.85) / (0.2 - 0.1)
    elif h_0 < 300 * mm:
        k_h = ((0.75 - 0.85) * h_0 + 0.3 * 0.85 - 0.2 * 0.75) / (0.3 - 0.2)
    elif h_0 < 500 * mm:
        k_h = ((0.7 - 0.75) * h_0 + 0.5 * 0.75 - 0.3 * 0.7) / (0.5 - 0.3)
    else:
        k_h = 0.7
    beta_RH = 1.55 * (1 - (RH / 100) ** 3)
    f_cm0 = 10 * MPa
    alfa_ds1, alfa_ds2 = 4, 0.12  # cement klasy N
    epsilon_cd_0 = (
        0.85
        * (220 + 110 * alfa_ds1)
        * exp(-alfa_ds2 * (f_cm / f_cm0))
        * beta_RH
        * 10 ** -6
    )
    epsilon_cd_t = beta_ds_t_ts * k_h * epsilon_cd_0
    beta_as_t = 1 - exp(-0.2 * t ** 0.5)
    epsilon_ca_inf = 2.5 * (f_ck / MPa - 10) * 10 ** -6
    epsilon_ca_t = beta_as_t * epsilon_ca_inf
    epsilon_cs = epsilon_cd_t + epsilon_ca_t
    S_I = A_s_prov * (d - x_I)
    S_II = A_s_prov * (d - x_II)
    k_cs_I = epsilon_cs * alfa_e * S_I / J_I
    k_cs_II = epsilon_cs * alfa_e * S_II / J_II
    k_cs_m = dzeta * k_cs_II + (1 - dzeta) * k_cs_I
    alfa_cs = k_cs_m * l_eff ** 2 / 8

    # Wpływ kratownicy na zmniejszenie ugięcia (p. 8 [3])
    y_g = h_k - fi_g / 2
    y_d = (2 * fi_d + n_2 * fi_2) / (2 + n_2)
    y = (n_k * A_s(fi_g) * y_g + (A_s_2 + A_s_k) * y_d) / (
        n_k * A_s(fi_g) + (A_s_2 + A_s_k)
    )
    I_g = pi * sqrt(n_k * A_s(fi_g) / pi) ** 4 / 64
    I_d = pi * sqrt((A_s_2 + A_s_k) / pi) ** 4 / 64
    I_k = (
        I_g + I_d + n_k * A_s(fi_g) * (y_g - y) ** 2 + (A_s_2 + A_s_k) * (y_d - y) ** 2
    )
    gamma_k = max(1 - 0.9 * E_s * I_k / (E_c_eff * J), 0.85)

    # Warunek SLS (SGU) - Sprawdzenie ugięć (p. 7.4.1 [2])
    alfa_0 = l_eff / 300  # odwrotna strzałka ugięcia (p. 8 [3])
    alfa_fin = max(gamma_k * (alfa + alfa_cs) - alfa_0, 0)
    alfa_lim = l_eff / 250  # p. 7.4.1(4) [2]
    if alfa_fin > alfa_lim:
        warn += "<font color=orange>Dopuszczalne ugięcie przekroczone.<br>"

    # Warunek SLS (SGU) - Sprawdzenie rys (p. 7.3 [2] i tab. 14.2 [5])
    alfa_e = E_s / E_cm
    h_c_ef = min(2.5 * (h - d), (h - x_I) / 3)
    A_c_eff = h_c_ef * b
    ro_p_eff = A_s_prov / A_c_eff
    sigma_s = M_Ed_q / (z * A_s_prov)
    k_t = 0.4  # dla obciążeń długotrwałych
    epsilon_sm_cm = max(
        (sigma_s - k_t * (f_ctm / ro_p_eff) * (1 + alfa_e * ro_p_eff)) / E_s,
        0.6 * sigma_s / E_s,
    )
    s_r_max = 1.3 * (h - x_I)
    w_k = s_r_max * epsilon_sm_cm
    w_max = 0.4 * mm  # tab. 7.2N [2]
    if w_k > w_max:
        warn += "<font color=orange>Dopuszczalne rysy przekroczone.<br>"

    # Notka
    s_r = min(3 * h, 40 * cm, m / (0.2 * A_s_req / (b_p * cm) / A_s(fi_r)))
    s_r = 5 * cm * round(s_r / (5 * cm))
    siatki = [188, 283]
    A_s_s = min(s for s in siatki if s > ((0.25 * A_s_req) / (b_p * cm) / mm2))
    return (
        (
            f"<h4><mark>PŁYTA {h / cm:.0f}{'s' if s == 'true' else ''} L={l_p} W={b_p} "
            f"({p_k:.1f} kN/m2)<br>zbroj. {f'{n_k}x' if n_k > 1 else ''}"
            f"{A_s_prov / n_k / cm2:.2f} cm2<br></mark></h4>"
            f"{n_k}<small>x krat.</small> E-{h_k / cm:.0f}-"
            f"{int(fi_d / mm):02d}{fi_k / mm:.0f}{int(fi_g / mm):02d}<br>"
            f"{n_1}#{fi_1 / mm:.0f} <small>na całej długości w płycie</small><br>"
            f"{f'{n_2}#{fi_2 / mm:.0f} {l_2} ' if n_2 != 0 else ''}"
            f"{'<small>dospawane do kratownicy</small><br>' if n_2 != 0 else ''}"
            f"{f'{n_3}#{fi_3 / mm:.0f} ' if n_3 != 0 else ''}"
            f"{'<small>dozbrojenie na płycie</small><br>' if n_3 != 0 else ''}"
            f"#{fi_r / mm:.0f} co {s_r / cm:.0f} cm "
            f"<small>rozdzielcze w płycie</small><br>"
            f"{f'Q{A_s_s} szer. {ceil(l / cm / 7 / 5) * 5 + 20} cm ' if l >= 4 else ''}"
            f"{'<small>nadpodporowe</small>' if l >= 4 else ''}"
            f"<p><h4>WYMIAROWANIE WG EUROKODÓW</h4><br>"
            f"A<sub>s</sub> = {A_s_prov / cm2:.2f} cm&#178; "
            f"({A_s_prov / (b_p * cm) / mm2:.0f} mm&#178;/m) &rho; = {ro:.2%}<br>"
            f"M<sub>Ed</sub> = {M_Ed / kN:.1f} kNm "
            f"{'<' if M_Ed < M_Rd else '>'} "
            f"M<sub>Rd</sub> = {M_Rd / kN:.1f} kNm ({float(M_Ed/M_Rd):.1%})<br>"
            f"V<sub>Ed</sub> = {V_Ed / kN:.1f} kN "
            f"{'<' if V_Ed < V_Rd_c else '>'} "
            f"V<sub>Rd,c</sub> = {V_Rd_c / kN:.1f} kN ({float(V_Ed/V_Rd_c):.1%})<br>"
            f"v<sub>Edi</sub> = {v_Ed_i / MPa:.2f} MPa "
            f"{'<' if v_Ed_i < v_Rd_i else '>'} "
            f"v<sub>Rdi</sub> = {v_Rd_i / MPa:.2f} MPa ({float(v_Ed_i/v_Rd_i):.1%})<br>"
            f"&alpha;<sub>fin</sub> = {alfa_fin / mm:.1f} mm "
            f"{'<' if alfa_fin < alfa_lim else '>'} "
            f"&alpha;<sub>lim</sub> = {alfa_lim / mm:.1f} mm "
            f"({float(alfa_fin / alfa_lim):.1%})<br>"
            f"w<sub>k</sub> = {w_k / mm:.2f} mm "
            f"{'<' if w_k < w_max else '>'} "
            f"w<sub>max</sub> = {w_max / mm:.1f} mm ({float(w_k/w_max):.1%})</p>"
        ),
        warn if warn else "<font color=green>Warunki stanów granicznych spełnione.<br>",
    )
