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
Cross-engine slosh-displacement metric for :ref:`scenarioCompareVariableMass`.

The base scenario plots the spring-mass-damper slosh displacements but does not
write their cross-engine difference to JSON. This driver runs the uncompensated
comparison in both the deep-space and in-orbit configurations and records the
maximum absolute displacement difference over the burn.

Results are written to ``results/sweepSloshDisplacement.json``.
"""

import json
import os

import numpy as np

import scenarioCompareVariableMass as scv

from Basilisk.utilities import macros

resultsPath = os.path.join(os.path.dirname(__file__), "results")


def maxSloshDifference(inOrbit, tf=900.0):
    """Max abs cross-engine SMD displacement difference [m] for one configuration."""
    dt = scv.timeStep()
    mu = scv.earthMu()

    bsmSim, bsmRec, _ = scv.buildBSM(dt, True, True, inOrbit)
    bsmSim.ConfigureStopTime(macros.sec2nano(tf))
    bsmSim.ExecuteSimulation()
    bsm = scv.pullBSM(bsmRec, mu)

    initialState = {"r_BN_N": bsm["r_BN_N"][0], "v_BN_N": bsm["v_BN_N"][0],
                    "sigma_BN": bsm["sigma_BN"][0], "omega_BN_B": bsm["omega_BN_B"][0]}
    mjSim, mjRec, _ = scv.buildMujoco(dt, True, initialState, True, inOrbit)
    mjSim.ConfigureStopTime(macros.sec2nano(tf))
    mjSim.ExecuteSimulation()
    mj = scv.pullMujoco(mjRec, mu)

    nSamples = min(len(bsm["rho"]), len(mj["rho"]))
    return float(np.max(np.abs(bsm["rho"][:nSamples] - mj["rho"][:nSamples])))


def run():
    out = {}
    for inOrbit in (False, True):
        key = "inOrbit" if inOrbit else "deepSpace"
        out[key] = maxSloshDifference(inOrbit)
        print("{}: max slosh displacement diff {:.3e} m".format(key, out[key]))

    os.makedirs(resultsPath, exist_ok=True)
    outFile = os.path.join(resultsPath, "sweepSloshDisplacement.json")
    with open(outFile, "w") as f:
        json.dump({
            "scenario": "sweepSloshDisplacement",
            "maxAbsRhoDiff_m": out,
            "configuration": {"dt": scv.timeStep(), "tf": 900.0,
                              "note": "SMD rho1..3 BSM vs MuJoCo, uncompensated"},
        }, f, indent=2)
    print("Wrote " + outFile)


if __name__ == "__main__":
    run()
