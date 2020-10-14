from flask import jsonify, render_template, request

from . import app, processing as p


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/_process")
def _process():
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
            result=p.oblicz(
                g_k, q_k, kat, l, b, h, s, n_1, n_2, n_3, fi_1, fi_2, fi_3, fi_r
            )
        )
    except Exception as e:
        return jsonify(result=str(e))
