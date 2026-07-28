#
#  ISC License
#
#  Copyright (c) 2026, Autonomous Vehicle Systems Lab, University of Colorado at Boulder
#
#  Permission to use, copy, modify, and/or distribute this software for any
#  purpose with or without fee is hereby granted, provided that the above
#  copyright notice and this permission notice appear in all copies.
#
#  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
#  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
#  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
#  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
#  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
#  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
#  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
#

r"""
Integrator-step sweep for :ref:`scenarioCompareOrbit`.

Repeats the two-orbit Keplerian comparison over a ladder of RK4 steps to separate the
two error sources the base scenario reports at a single ``dt``:

#. the per-engine error against the analytic Kepler solution, expected to shrink as
   :math:`dt^4` (RK4 truncation); and
#. the cross-engine BSM-vs-MuJoCo difference, expected to be ``dt``-independent
   (formulation/round-off seeded, not time-stepping).

Results are written to ``results/sweepOrbitDt.json``.
"""

import json
import os

import numpy as np

import scenarioCompareOrbit as sco

from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody

resultsPath = os.path.join(os.path.dirname(__file__), "results")


def run():
    mass = 750.0  # [kg]
    mu = simIncludeGravBody.BODY_DATA["earth"].mu  # [m^3/s^2]
    recordDt = 60.0  # [s]

    rN, vN, oe0 = sco.initialOrbitState(mu)
    orbitPeriod = 2.0*np.pi*np.sqrt(oe0.a**3/mu)  # [s]
    tf = 2.0*orbitPeriod  # [s]

    rows = []
    for dt in (40.0, 20.0, 10.0, 5.0, 2.5):
        bsmRec = sco.runBSM(mass, mu, dt, tf, recordDt)
        mjRec = sco.runMujoco(mu, dt, tf, recordDt)
        times = np.array(bsmRec.times())*macros.NANO2SEC
        posBSM = np.array(bsmRec.r_BN_N)
        posMujoco = np.array(mjRec.r_BN_N)
        nSamples = min(len(posBSM), len(posMujoco))
        times, posBSM, posMujoco = times[:nSamples], posBSM[:nSamples], posMujoco[:nSamples]
        truth = sco.keplerTruth(mu, oe0.a, oe0, times)
        rows.append({
            "dt": dt,  # [s]
            "bsmVsAnalyticMax": float(np.max(
                np.linalg.norm(truth - posBSM, axis=1))),  # [m]
            "mujocoVsAnalyticMax": float(np.max(
                np.linalg.norm(truth - posMujoco, axis=1))),  # [m]
            "crossParadigmPosMax": float(np.max(
                np.linalg.norm(posBSM - posMujoco, axis=1))),  # [m]
        })
        print("dt {:5.1f} s: BSM vs Kepler {:.3e} m, cross-engine {:.3e} m".format(
            dt, rows[-1]["bsmVsAnalyticMax"], rows[-1]["crossParadigmPosMax"]))

    # Fitted truncation order between successive rungs of the analytic-error ladder.
    orders = []
    for lo, hi in zip(rows[1:], rows[:-1]):
        orders.append(np.log(hi["bsmVsAnalyticMax"]/lo["bsmVsAnalyticMax"])
                      / np.log(hi["dt"]/lo["dt"]))
    print("pairwise fitted truncation orders:", ["{:.2f}".format(p) for p in orders])

    os.makedirs(resultsPath, exist_ok=True)
    outFile = os.path.join(resultsPath, "sweepOrbitDt.json")
    with open(outFile, "w") as f:
        json.dump({
            "scenario": "sweepOrbitDt",
            "configuration": {"tf": tf, "recordDt": recordDt,
                              "orbit": "same elements as scenarioCompareOrbit, 2 orbits"},
            "rows": rows,
            "pairwiseTruncationOrders": [float(p) for p in orders],
        }, f, indent=2)
    print("Wrote " + outFile)


if __name__ == "__main__":
    run()
