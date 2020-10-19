from itertools import product
from math import ceil

from flask import jsonify, render_template, request

from . import app, processing as p


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/_dobierz")
def _dobierz():
    g_k = float(request.args.get("g_k")) * p.kPa
    q_k = float(request.args.get("q_k")) * p.kPa
    kat = request.args.get("kat")
    l = float(request.args.get("l")) * p.cm
    b = float(request.args.get("b")) * p.cm
    h = float(request.args.get("h")) * p.cm
    s = request.args.get("s")
    b_p = round(b / p.cm)
    n_1 = ceil(((b_p - p.n_k(b_p) * 6) / 18 - p.n_k(b_p)) / 2) * 2
    n_2 = [0, p.n_k(b_p) * 2]
    types = [
        (fi_1 * p.mm, fi_2 * p.mm)
        for (fi_1, fi_2) in [
            (8, 0),
            (10, 0),
            (12, 0),
            (10, 8),
            (10, 10),
            (12, 10),
            (12, 12),
            (14, 12),
            (14, 14),
        ]
    ]
    try:
        for (fi_1, fi_2), n_2 in product(types, n_2):
            result = p.wymiarowanie(
                g_k, q_k, kat, l, b, h, s, n_1, n_2, 0, fi_1, fi_2, 0, 6 * p.mm
            )
            if result[1].startswith("<font color=green>"):
                return jsonify(result=result)
        return jsonify(result="<font color=orange>Dobierz zbrojenie ręcznie.")
    except Exception as e:
        return jsonify(result=str(e))


@app.route("/_oblicz")
def _oblicz():
    g_k = float(request.args.get("g_k")) * p.kPa
    q_k = float(request.args.get("q_k")) * p.kPa
    kat = request.args.get("kat")
    l = float(request.args.get("l")) * p.cm
    b = float(request.args.get("b")) * p.cm
    h = float(request.args.get("h")) * p.cm
    s = request.args.get("s")
    n_1 = int(float(request.args.get("n_1")))
    n_2 = int(float(request.args.get("n_2")))
    n_3 = int(float(request.args.get("n_3")))
    fi_1 = int(float(request.args.get("fi_1"))) * p.mm
    fi_2 = int(float(request.args.get("fi_2"))) * p.mm
    fi_3 = int(float(request.args.get("fi_3"))) * p.mm
    fi_r = float(request.args.get("fi_r")) * p.mm
    try:
        return jsonify(
            result=p.wymiarowanie(
                g_k, q_k, kat, l, b, h, s, n_1, n_2, n_3, fi_1, fi_2, fi_3, fi_r
            )
        )
    except Exception as e:
        return jsonify(result=str(e))
