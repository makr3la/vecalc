from itertools import product
from math import ceil

from flask import jsonify, render_template, request

from . import app, processing as p


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/_dobierz")
def _dobierz():
    """Dobór zbrojenia na podstawie obciążeń i geometrii z założeniami Stropy Małro."""
    b = float(request.args.get("b")) * p.cm
    h = float(request.args.get("h")) * p.cm
    b_p = round(b / p.cm)
    n_k = ceil(b_p / 60)
    n_1 = ceil(((b_p - n_k * 6) / 18 - n_k) / 2) * 2
    n_2 = [0, n_k * 2, n_k * 3]
    types = [
        (fi_1 * p.mm, fi_2 * p.mm)
        for (fi_1, fi_2) in [
            (8, 0),
            (10, 0),
            (8, 8),
            (10, 8),
            (10, 10),
            (10, 12),
            (10, 14),
        ]
    ]
    try:
        for n_1, (fi_1, fi_2), n_2 in product([n_1, 2 * n_1], types, n_2):
            result = p.wymiarowanie(
                g_k=float(request.args.get("g_k")) * p.kPa,
                q_k=float(request.args.get("q_k")) * p.kPa,
                kat=request.args.get("kat"),
                l=float(request.args.get("l")) * p.cm,
                b=b,
                h=h,
                s=request.args.get("s"),
                n_1=n_1,
                n_2=n_2,
                n_3=0,
                fi_1=fi_1,
                fi_2=fi_2,
                fi_3=0,
                fi_r=6 * p.mm,
                n_k=n_k,
                h_k=14 * p.cm if h >= 20 * p.cm else 10 * p.cm,
                fi_d=5 * p.mm,
                fi_k=5 * p.mm,
                fi_g=10 * p.mm,
                bet="20",
                c_min_dur=10 * p.mm,
                delta_c_dev=5 * p.mm,
            )
            if result[1].startswith("<font color=green>"):
                return jsonify(result=result)
        return jsonify(result="<font color=orange>Dobierz zbrojenie ręcznie.")
    except Exception as e:
        return jsonify(result=str(e))


@app.route("/_oblicz")
def _oblicz():
    """Wymiarowanie zbrojenia na podstawie przyjętych parametrów."""
    try:
        return jsonify(
            result=p.wymiarowanie(
                g_k=float(request.args.get("g_k")) * p.kPa,
                q_k=float(request.args.get("q_k")) * p.kPa,
                kat=request.args.get("kat"),
                l=float(request.args.get("l")) * p.cm,
                b=float(request.args.get("b")) * p.cm,
                h=float(request.args.get("h")) * p.cm,
                s=request.args.get("s"),
                n_1=int(float(request.args.get("n_1"))),
                n_2=int(float(request.args.get("n_2"))),
                n_3=int(float(request.args.get("n_3"))),
                fi_1=int(float(request.args.get("fi_1"))) * p.mm,
                fi_2=int(float(request.args.get("fi_2"))) * p.mm,
                fi_3=int(float(request.args.get("fi_3"))) * p.mm,
                fi_r=float(request.args.get("fi_r")) * p.mm,
                n_k=int(float(request.args.get("n_k"))),
                h_k=float(request.args.get("h_k")) * p.cm,
                fi_d=float(request.args.get("fi_d")) * p.mm,
                fi_k=float(request.args.get("fi_k")) * p.mm,
                fi_g=float(request.args.get("fi_g")) * p.mm,
                bet=request.args.get("bet"),
                c_min_dur=float(request.args.get("c_min_dur")) * p.mm,
                delta_c_dev=float(request.args.get("delta_c_dev")) * p.mm,
            )
        )
    except Exception as e:
        return jsonify(result=str(e))
