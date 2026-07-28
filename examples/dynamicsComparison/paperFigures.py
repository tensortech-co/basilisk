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
Print-quality figures for the dynamics-engine comparison paper.

The scenario scripts emit documentation figures sized for web pages; this script
re-renders the figures placed in the paper from the stored ``results/*.json``
metrics, with fonts and legends sized for two-column print inclusion and one
consistent color scheme (Paul Tol high-contrast: BSM blue, MuJoCo red, third
series yellow).

Run after ``runAllComparisons.py`` and the sweep drivers::

    python3 paperFigures.py [outputDir]

``outputDir`` defaults to ``results/paper`` next to this script.
"""

import json
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt

thisFolder = os.path.dirname(os.path.abspath(__file__))
resultsPath = os.path.join(thisFolder, "results")

BLUE = "#004488"    # BSM
RED = "#BB5566"     # MuJoCo
YELLOW = "#DDAA33"  # third series (spinningBodyNDOF, at-rest cross-engine, ...)
GRAY = "#666666"

STYLE = {
    "font.size": 9,
    "axes.labelsize": 9,
    "axes.titlesize": 9,
    "legend.fontsize": 7.0,
    "xtick.labelsize": 8,
    "ytick.labelsize": 8,
    "lines.linewidth": 1.3,
    "lines.markersize": 4.0,
    "legend.framealpha": 0.9,
    "figure.constrained_layout.use": True,
}

# Marker per integrator family, shared by both Pareto panels and stated in the
# paper's figure caption -- keep the two in sync.
INTEGRATOR_MARKERS = (
    ("Euler", "v"),
    ("RK2", "s"),
    ("RK4", "o"),
    ("RKF45", "D"),
    ("RKF78", "^"),
)


def loadJson(name):
    with open(os.path.join(resultsPath, name + ".json")) as f:
        return json.load(f)


def markerFor(integratorName):
    """Marker for an svIntegrator name; RKF45 checked before RK4 by order below."""
    for key, marker in reversed(INTEGRATOR_MARKERS):
        if key in integratorName:
            return marker
    return "x"


def paretoPanel(data, errorKey, errorLabel, floorKey, outFile):
    """One work-precision panel: error vs wall-clock, both engines, marker per integrator."""
    fig, ax = plt.subplots(figsize=(3.5, 2.7))

    for engine, color in (("bsm", BLUE), ("mujoco", RED)):
        rows = data[engine]
        families = {}
        for row in rows:
            families.setdefault(row["integrator"], []).append(row)
        for integratorName, familyRows in families.items():
            familyRows.sort(key=lambda r: r["wall"])
            adaptive = "RKF" in integratorName
            ax.loglog([r["wall"] for r in familyRows],
                      [max(r[errorKey], 1e-18) for r in familyRows],
                      linestyle="-" if adaptive else "--",
                      marker=markerFor(integratorName), color=color,
                      markerfacecolor=color if adaptive else "white",
                      markeredgecolor=color, linewidth=1.0)

    ax.axhline(data[floorKey], color=GRAY, linestyle=":", linewidth=1.0)
    ax.text(0.98, data[floorKey]*1.6, "reference floor", color=GRAY, fontsize=6.5,
            ha="right", transform=ax.get_yaxis_transform())

    # Compact custom legend: engine colors + integrator markers.
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], color=BLUE, label="BSM"),
               Line2D([], [], color=RED, label="MuJoCo")]
    present = {r["integrator"] for r in data["bsm"]}
    for key, marker in INTEGRATOR_MARKERS:
        if any(key in name for name in present):
            handles.append(Line2D([], [], color=GRAY, linestyle="none",
                                  marker=marker, label=key))
    ax.legend(handles=handles, ncol=2, loc="best", handlelength=1.4,
              columnspacing=0.8, borderpad=0.4)

    ax.set_xlabel("wall-clock time [s]")
    ax.set_ylabel(errorLabel)
    ax.grid(True, which="both", alpha=0.2)
    fig.savefig(outFile)
    plt.close(fig)


def paretoFigures(outputDir):
    for study, jsonName in (("RwPanels", "scenarioCompareParetoRwPanels"),
                            ("FlexPanels", "scenarioCompareParetoFlexPanels")):
        data = loadJson(jsonName)
        paretoPanel(data, "error", "final attitude error [rad]", "referenceFloor",
                    os.path.join(outputDir, jsonName + "_frontier.pdf"))
        paretoPanel(data, "positionError", "final position error [m]",
                    "positionReferenceFloor",
                    os.path.join(outputDir, jsonName + "_frontierPosition.pdf"))


def scalingFigure(outputDir):
    """Wall-clock vs DOF for the three implementations, with BSM/MuJoCo ratio labels."""
    rows = loadJson("scenarioCompareFlexPanels")["rows"]
    dof = np.array([r["dof"] for r in rows])
    bsm = np.array([r["bsmWall"] for r in rows])
    ndof = np.array([r["ndofWall"] for r in rows])
    mujoco = np.array([r["mujocoWall"] for r in rows])

    fig, ax = plt.subplots(figsize=(4.8, 3.3))
    ax.loglog(dof, bsm, "o-", color=BLUE, label="BSM nHingedRigidBody")
    ax.loglog(dof, ndof, "^-", color=YELLOW, label="BSM spinningBodyNDOF")
    ax.loglog(dof, mujoco, "s-", color=RED, label="MuJoCo")
    for x, yBsm, yMujoco in zip(dof, bsm, mujoco):
        ax.text(x, np.sqrt(yBsm*yMujoco), f"{yBsm/yMujoco:.1f}$\\times$",
                fontsize=7, color=GRAY, ha="center", va="center")
    ax.set_xlabel("system degrees of freedom")
    ax.set_ylabel("wall-clock for 600 steps [s]")
    ax.legend(loc="upper left")
    ax.grid(True, which="both", alpha=0.2)
    fig.savefig(os.path.join(outputDir, "scenarioCompareFlexPanels_runtime.pdf"))
    plt.close(fig)


def orbitDtFigure(outputDir):
    rows = loadJson("sweepOrbitDt")["rows"]
    dts = np.array([r["dt"] for r in rows])
    analytic = np.array([r["bsmVsAnalyticMax"] for r in rows])
    cross = np.array([r["crossParadigmPosMax"] for r in rows])

    fig, ax = plt.subplots(figsize=(4.2, 3.1))
    ax.loglog(dts, analytic, "o-", color=BLUE, label="engine vs analytic Kepler")
    ax.loglog(dts, cross, "s-", color=RED, label="BSM vs MuJoCo")
    dref = np.array([dts.max(), dts.min()])
    ax.loglog(dref, analytic[0]*(dref/dts.max())**4, ":", color=GRAY,
              label=r"$\propto \Delta t^4$")
    ax.set_xlabel(r"integrator step $\Delta t$ [s]")
    ax.set_ylabel("max position difference [m]")
    # Pad below the flat cross-engine series so the legend sits in empty space.
    ax.set_ylim(bottom=cross.min()/60.0)
    ax.legend(loc="lower right")
    ax.grid(True, which="both", alpha=0.2)
    fig.savefig(os.path.join(outputDir, "sweepOrbitDt.pdf"))
    plt.close(fig)


def torqueArtifactFigure(outputDir):
    data = loadJson("sweepTorqueArtifact")

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(7.0, 2.9))

    dts = [r["dt"] for r in data["dtSweep"]]
    mujocoArtifact = [r["mujocoMotionAttMax"] for r in data["dtSweep"]]
    bsmArtifact = [r["bsmMotionAttMax"] for r in data["dtSweep"]]
    restCross = [r["crossEngineRestAttMax"] for r in data["dtSweep"]]
    ax1.loglog(dts, mujocoArtifact, "o-", color=RED, label="MuJoCo orbiting-vs-rest")
    ax1.loglog(dts, restCross, "s-", color=YELLOW, label="cross-engine, at rest")
    ax1.loglog(dts, bsmArtifact, "^-", color=BLUE, label="BSM orbiting-vs-rest")
    dref = np.array([max(dts), min(dts)])
    ax1.loglog(dref, restCross[0]*(dref/max(dts))**4, ":", color=GRAY,
               label=r"$\propto \Delta t^4$")
    ax1.set_xlabel(r"integrator step $\Delta t$ [s]")
    ax1.set_ylabel("max principal angle [rad]")
    ax1.legend(loc="lower right")
    ax1.grid(True, which="both", alpha=0.2)

    speeds = np.array([r["speed"] for r in data["velocitySweep"]])/1000.0
    mujocoDrift = [r["mujocoDriftAttMax"] for r in data["velocitySweep"]]
    bsmDrift = [r["bsmDriftAttMax"] for r in data["velocitySweep"]]
    ax2.semilogy(speeds, mujocoDrift, "o-", color=RED, label="MuJoCo drifting-vs-rest")
    ax2.semilogy(speeds, bsmDrift, "^-", color=BLUE, label="BSM drifting-vs-rest")
    ax2.set_xlabel("inertial drift speed [km/s]")
    ax2.set_ylabel("max principal angle [rad]")
    ax2.legend(loc="center right")
    ax2.grid(True, which="both", alpha=0.2)

    fig.savefig(os.path.join(outputDir, "sweepTorqueArtifact.pdf"))
    plt.close(fig)


def run(outputDir=None):
    if outputDir is None:
        outputDir = os.path.join(resultsPath, "paper")
    os.makedirs(outputDir, exist_ok=True)
    matplotlib.rcParams.update(STYLE)

    paretoFigures(outputDir)
    scalingFigure(outputDir)
    orbitDtFigure(outputDir)
    torqueArtifactFigure(outputDir)
    print("Paper figures written to " + outputDir)


if __name__ == "__main__":
    run(sys.argv[1] if len(sys.argv) > 1 else None)
