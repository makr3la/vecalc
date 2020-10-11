"""Moduł obliczeniowy, który przetwarza dane i sprawdza dobrany przekrój."""

from math import ceil, exp, pi, sqrt

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

####################### Założenia Stropy Małro #######################

# Beton konstrukcyjny prefabrykatów i nadbetonu klasy C20/25 (B25)
f_ck = 20 * MPa

# Stal zbrojeniowa klasy C (A-IIIN) – gatunek B500SP EPSTAL
f_yk = 500 * MPa
mi_lim = 0.371  # tab. 7.2 [5]

# Cement klasy N
alfa_ds1 = 4
alfa_ds2 = 0.12

# Klasa konstrukcji S4 i klasa ekspozycji XC1
c_min_dur = 15 * mm  # otulenie ze względu na warunki środowiska (tab. 4.4N [2])
delta_c_dev = 0 * mm  # odchyłka otulenia elementów prefabrykowanych (p. 4.4.1.3 [2])

# Ugięcia od skurczu
t = 50 * 365  # wiek betonu w rozważanej chwili w dniach
ts = 5  # wiek betonu na końcu okresu pielęgnacji w dniach
t0 = 28  # wiek betonu w chwili przyłożenia obciążenia
RH = 80  # wilgotność powietrza zewnętrznego w procentach

# Geometria i wkłady styropianowe
h_p = 45 * mm  # grubość płyty prefabrykowanej
b_w = 16 * cm  # szerokość żebra usztywniającego (odstęp pomiędzy licami wkładów)
b_st = 5 * cm  # odstęp pomiędzy krawędzią płyty i licem wkładu styropianowego


def h_st(h: float) -> float:
    """Zwraca wysokość wkładu styropianowego w zależności od grubości stropu."""
    if h >= 20 * cm:
        return 10 * cm
    elif h >= 23 * cm:
        return 12 * cm
    else:
        raise Exception("Nie zdefiniowano wysokości wkładu do podanej grubości stropu.")


# Kratownice stalowe FILIGRAN typ E
h_k = lambda h: (14 * cm if h >= 20 * cm else 10 * cm)
n_k = lambda b: (int(ceil(b / cm / 60)))
fi_g = 10 * mm
fi_k = 5 * mm
fi_d = 5 * mm

# Odwrotna strzałka ugięcia (p. 8 [3])
alfa_0 = lambda l_eff: (l_eff / 300 if l_eff >= 4 else 0)

######################################################################


def oblicz(
    g_k: float,
    q_k: float,
    l: float,
    b: float,
    h: float,
    n_1: int,
    n_2: int,
    fi_1: float,
    fi_2: float,
    fi_r: float,
    s: bool,
) -> (str, str):
    """Wymiarowanie zbrojenia do schematu belki jednoprzęsłowej, swobodnie podpartej."""
    b_p = round(b / cm)
    warn = ""
    if n_2 not in [0, n_k(b) * 2]:
        raise Exception("Dopuszczalne 0 lub 2 pręty dospawane do jednej kratownicy.")

    # Statyka
    g_k, q_k = b * g_k, b * q_k  # przeliczenie obciążenia na metr bieżący przekroju
    if s == "true":  # dodanie ciężaru własnego do obciążeń stałych
        g_k += 24.5 * kPa * (h * b - (b - n_k(b) * b_w - 2 * b_st) * h_st(h))
    else:
        g_k += 24.5 * kPa * h * b
    p = max(g_k * 1.35 + q_k * 1.5 * 0.7, g_k * 1.35 * 0.85 + q_k * 1.5)  # 6.10a/b [1]
    p_q = g_k + q_k * 0.3  # 6.16b [1]
    l_eff = l + 8 * cm  # (p. 8 [3])
    M_Ed = 0.125 * p * l_eff ** 2
    V_Ed = 0.5 * p * l_eff
    l_eff = l  # (p. 8 [3])
    M_Ed_q = 0.125 * p_q * l_eff ** 2

    # Otulina i wysokość użyteczna (p. 4.4 [2])
    c_min_b = max(fi_1, fi_2)
    c_min = max(c_min_b, c_min_dur, 10 * mm)
    c_nom = c_min + delta_c_dev
    a_1 = c_nom + 0.5 * max(fi_1, fi_2) + fi_r
    d = h - a_1

    # Materiały (p. 3 oraz załącznik B [2])
    f_cd = f_ck / 1.4
    f_cm = f_ck + 8 * MPa
    f_ctm = 0.3 * (f_ck / MPa) ** (2 / 3) * MPa
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
    f_yd = f_yk / 1.15
    E_s = 200 * GPa

    # Wkłady styropianowe (zastępcza szerokość z równości momentów bezwładności [7])
    if s == "true":
        if n_k(b) == 1 and (b - b_w - 2 * b_st) / 2 < max(h_st(h), 85 * mm):
            raise Exception("Nie można stosować wkładów do podanej grubości stropu.")
        A_c = b * h
        y_c = h / 2
        J_c = b * h ** 3 / 12
        A_st = (b - n_k(b) * b_w - 2 * b_st) * h_st(h)
        y_st = h_p + h_st(h) / 2
        J_st = (b - n_k(b) * b_w - 2 * b_st) * h_st(h) ** 3 / 12
        y = (A_c * y_c - A_st * y_st) / (A_c - A_st)
        J_z = J_c - J_st + A_c * (y_c - y) ** 2 - A_st * (y_st - y) ** 2
        b = 12 * J_z / h ** 3

    # Warunek ULS (SGN) - Zginanie (tab. 7.2 i p. 7.1.1 [5])
    mi = M_Ed / (b * d ** 2 * f_cd)
    if mi > mi_lim:
        warn += "<font color=red>Przekroczno wartość graniczną mi.<br>"
    omega = 0.9731 - sqrt(0.9469 - 1.946 * mi)
    A_c = A_c - A_st if s == "true" else b * d
    A_s_req = max(
        omega * A_c * (f_cd / f_yd), 0.26 * f_ctm / f_yk * A_c, 0.0013 * A_c
    ) - n_k(b) * 2 * A_s(fi_d)
    A_s_prov = n_1 * A_s(fi_1) + n_2 * A_s(fi_2) + n_k(b) * 2 * A_s(fi_d)
    ro = A_s_prov / A_c
    if A_s_prov < A_s_req:
        warn += f"<font color=red>Zbyt mały stopień zbrojenia przekroju = {ro:.2%}.<br>"
    if A_s_prov > 0.5 * f_cd / f_yd * A_c or A_s_prov > 0.04 * A_c:
        warn += f"<font color=red>Zbyt duży stopień zbrojenia przekroju = {ro:.2%}.<br>"
    M_Rd = A_s_prov * f_yd * (d - 0.5138 * ((A_s_prov * f_yd) / (b * f_cd)))
    if M_Ed > M_Rd:
        warn += "<font color=red>Warunek nośności na zginanie niespełniony.<br>"

    # Warunek ULS (SGN) - Ścinanie (tab. 10.2 [5])
    k = min(1 + sqrt(20 / (d / mm)), 2.0)
    v_Rd_c = (0.18 / 1.4) * k * (100 * ro * (f_ck / MPa)) ** (1 / 3) * MPa
    v_min = 0.035 * sqrt(k ** 3 * f_ck / MPa) * MPa
    v_Rd_c = max(v_Rd_c, v_min)
    V_Rd_c = v_Rd_c * b * d
    if V_Ed > V_Rd_c:
        warn += "<font color=red>Element wymagający zbrojenie na ścinanie.<br>"

    # Ugięcia w stanie zarysowania (p. 7.4.3 [2])
    alfa_e = E_s / E_c_eff
    x_I = (0.5 * b * h ** 2 + alfa_e * A_s_prov * d) / (b * h + alfa_e * A_s_prov)
    J = b * h ** 3 / 12
    J_I = J + b * h * (x_I - h / 2) ** 2 + alfa_e * A_s_prov * (d - x_I) ** 2
    alfa_k = 5 / 48
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
    beta_ds_t_ts = (t - ts) / ((t - ts) + 0.04 * sqrt(h ** 3))
    if h < 200 * mm:
        k_h = ((0.85 - 1) * h + 0.2 * 1 - 0.1 * 0.85) / (0.2 - 0.1)
    elif h < 300 * mm:
        k_h = ((0.75 - 0.85) * h + 0.3 * 0.85 - 0.2 * 0.75) / (0.3 - 0.2)
    elif h < 500 * mm:
        k_h = ((0.7 - 0.75) * h + 0.5 * 0.75 - 0.3 * 0.7) / (0.5 - 0.3)
    else:
        k_h = 0.7
    beta_RH = 1.55 * (1 - (RH / 100) ** 3)
    f_cm0 = 10 * MPa
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
    alfa_ks = 1 / 8
    alfa_cs = alfa_ks * k_cs_m * l_eff ** 2

    # Wpływ kratownicy na zmniejszenie ugięcia (p. 8 [3])
    y_g = h_k(h) - fi_g / 2
    y_d = (fi_d + fi_2) / 4
    A_sd = n_k(b) * 2 * A_s(fi_d) + n_2 * A_s(fi_2)
    y = (n_k(b) * A_s(fi_g) * y_g + A_sd * y_d) / (n_k(b) * A_s(fi_g) + A_sd)
    I_g = pi * sqrt(n_k(b) * A_s(fi_g) / pi) ** 4 / 64
    I_d = pi * sqrt(A_sd / pi) ** 4 / 64
    I_k = I_g + I_d + n_k(b) * A_s(fi_g) * (y_g - y) ** 2 + A_sd * (y_d - y) ** 2
    gamma_k = max(1 - 0.9 * E_s * I_k / (E_c_eff * (b * h ** 3 / 12)), 0.85)

    # Warunek SLS (SGU) - Sprawdzenie ugięć (p. 7.4.1 [2])
    alfa_fin = gamma_k * (alfa + alfa_cs) - alfa_0(l_eff)
    alfa_lim = l_eff / 250
    if alfa_fin > alfa_lim:
        warn += "<font color=orange>Dopuszczalne ugięcie przekroczone.<br>"

    # Notka
    l_p = int(ceil((l + 12 * cm) / cm / 10)) * 10
    s_r = min(40 * cm, m / (0.2 * A_s_req / (b_p * cm) / A_s(fi_r)))
    return (
        (
            f"<h4>VECTOR {h / cm:.0f}{'s' if s == 'true' else ''} L={l_p} W={b_p}<br>"
            f"zbroj. {n_1}#{fi_1 / mm:.0f} "
            f"{f'+ {n_2}#{fi_2 / mm:.0f}' if n_2 != 0 else ''} + {n_k(b)} krat. "
            f"E-{h_k(h) / cm:.0f}-0{fi_d / mm:.0f}{fi_k / mm:.0f}{fi_g / mm:.0f}<br>"
            f"rozdzielcze #{fi_r / mm:.0f} co {s_r / cm:.0f} cm</h4>"
            f"<p>Przyjęto: A<sub>s</sub> = {A_s_prov / cm2:.2f} cm&#178; "
            f"({A_s_prov / (b_p * cm) / mm2:.0f} mm&#178;/m)<br>"
            f"Zginanie: M<sub>Ed</sub> = {M_Ed / kN:.1f} kNm "
            f"{'<' if M_Ed < M_Rd else '>'} "
            f"M<sub>Rd</sub> = {M_Rd / kN:.1f} kNm ({float(M_Ed/M_Rd):.1%})<br>"
            f"Ścinanie: V<sub>Ed</sub> = {V_Ed / kN:.1f} kN "
            f"{'<' if V_Ed < V_Rd_c else '>'} "
            f"V<sub>Rd,c</sub> = {V_Rd_c / kN:.1f} kN ({float(V_Ed/V_Rd_c):.1%})<br>"
            f"Ugięcie: &alpha;<sub>fin</sub> = {alfa_fin / mm:.1f} mm "
            f"{'<' if alfa_fin < alfa_lim else '>'} "
            f"&alpha;<sub>lim</sub> = {alfa_lim / mm:.1f} mm "
            f"({float(alfa_fin / alfa_lim):.1%})</p>"
        ),
        warn if warn else "<font color=green>Warunki stanów granicznych spełnione.<br>",
    )
