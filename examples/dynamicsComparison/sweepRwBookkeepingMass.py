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
Bookkeeping-mass sweep for :ref:`scenarioCompareRwPanels`.

MuJoCo requires every wheel body to carry a positive mass, while the BSM
balanced-wheel effector is ideally massless; the scenario uses a 1e-6 kg
bookkeeping mass. This sweep reruns the comparison with that mass reduced to
confirm it is what sets the ~1e-10 rad cross-engine agreement floor.

Results are written to ``results/sweepRwBookkeepingMass.json``.
"""

import json
import os

import numpy as np

import scenarioCompareRwPanels as srp

resultsPath = os.path.join(os.path.dirname(__file__), "results")


def attitudeFloor(rwMass, dt=0.02, tf=120.0, recordDt=0.5):
    """Max cross-engine hub-attitude principal angle with the given wheel mass [kg]."""
    srp.RW_MASS = rwMass
    bsmState, _, _, _ = srp.runBSM(dt, tf, recordDt)
    mjState, _, _, _ = srp.runMujoco(dt, tf, recordDt)
    sigmaBSM = np.array(bsmState.sigma_BN)
    sigmaMujoco = np.array(mjState.sigma_BN)
    nSamples = min(len(sigmaBSM), len(sigmaMujoco))
    return float(np.max(srp.relativePrincipalAngle(sigmaBSM[:nSamples], sigmaMujoco[:nSamples])))


def run():
    rows = []
    for rwMass in (1.0e-6, 1.0e-8, 1.0e-10):
        floor = attitudeFloor(rwMass)
        rows.append({"rwMass": rwMass, "hubAttitudePrincipalAngleMax": floor})
        print("RW_MASS {:.0e} kg -> attitude floor {:.3e} rad".format(rwMass, floor))

    os.makedirs(resultsPath, exist_ok=True)
    outFile = os.path.join(resultsPath, "sweepRwBookkeepingMass.json")
    with open(outFile, "w") as f:
        json.dump({
            "scenario": "sweepRwBookkeepingMass",
            "configuration": {"dt": 0.02, "tf": 120.0,
                              "note": "scenarioCompareRwPanels with swept MuJoCo wheel bookkeeping mass"},
            "rows": rows,
        }, f, indent=2)
    print("Wrote " + outFile)


if __name__ == "__main__":
    run()
