"""Module with all the view (route) functions."""

from flask import jsonify, render_template, request

from . import app, processing as p


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/_process")
def _process():
    g_k = float(request.args.get("g_k")) * p.kPa
    q_k = float(request.args.get("q_k")) * p.kPa
    l = float(request.args.get("l"))
    h = float(request.args.get("h")) * p.cm
    n_1 = int(request.args.get("n_1"))
    n_2 = int(request.args.get("n_2"))
    fi_1 = float(request.args.get("fi_1")) * p.mm
    fi_2 = float(request.args.get("fi_2")) * p.mm
    fi_r = float(request.args.get("fi_r")) * p.mm
    s = request.args.get("s")
    try:
        return jsonify(result=p.oblicz(g_k, q_k, l, h, n_1, n_2, fi_1, fi_2, fi_r, s))
    except Exception as e:
        return jsonify(result=str(e))
