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

r"""
Shared comparison-plot helpers for the dynamics-engine comparison series.

Every accuracy scenario overlays the back-substitution (BSM) solution against MuJoCo for the same
physical quantity, and what the reader wants to see is not just that the two curves lie on top of
one another but *how far apart* they are. These helpers enforce a single convention for that:

- :func:`overlayWithDifference` -- for a scalar time series, a top axis with both engines overlaid
  and a bottom axis with the BSM-minus-MuJoCo difference, sharing the time axis.
- :func:`componentComparison` -- for a three-component vector (e.g. the body rate), a ``2 x 3``
  grid: one column per axis (x, y, z), the top row overlaying both engines and the bottom row the
  per-component difference.

Both build the figure with ``layout="constrained"`` so the axis labels are never clipped or
overlapped (the scenarios save with ``savefig`` and no ``bbox_inches='tight'``), and both degrade
gracefully to the BSM curve alone when the MuJoCo history is absent (``mj`` is None).
"""

import numpy as np
import matplotlib.pyplot as plt

# Shared palette across the comparison series, from the standard Basilisk plotting colors.
from Basilisk.utilities import unitTestSupport

COLOR_BSM = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)
COLOR_DIFF = unitTestSupport.getLineColor(2, 3)

_AXES = ("x", "y", "z")


def _plotBoth(ax, tBSM, yBSM, tMj, yMj, bsmLabel, mjLabel):
    """Overlay one BSM curve (thick translucent underlay) and one MuJoCo curve (thin) on ``ax``."""
    ax.plot(tBSM, yBSM, lw=4, alpha=0.4, color=COLOR_BSM, label=bsmLabel)
    if yMj is not None:
        ax.plot(tMj, yMj, lw=1.3, color=COLOR_MUJOCO, label=mjLabel)


def _difference(tBSM, yBSM, tMj, yMj):
    """BSM-minus-MuJoCo difference on the common sample count, or ``(None, None)`` if no MuJoCo."""
    if yMj is None:
        return None, None
    n = min(len(yBSM), len(yMj))
    return tBSM[:n], yBSM[:n] - yMj[:n]


def overlayWithDifference(figName, time, yBSM, yMj, ylabel, xlabel="Time [s]",
                          diffScale=1.0, diffUnit=None, bsmLabel="BSM", mjLabel="MuJoCo",
                          logDiff=False):
    """Two stacked axes: BSM/MuJoCo overlay on top, their difference below.

    Args:
        figName (str): figure key returned in the ``{name: figure}`` mapping.
        time (numpy.ndarray): sample times [s] for the BSM curve.
        yBSM (numpy.ndarray): BSM scalar history, shape ``(N,)``.
        yMj (numpy.ndarray or None): MuJoCo scalar history, or None when MuJoCo is unavailable.
        ylabel (str): y-label for the overlay (top) axis.
        xlabel (str, optional): shared x-label. Defaults to ``"Time [s]"``.
        diffScale (float, optional): factor applied to the difference for display (e.g. 1e3 to show
            it in milli-units). Defaults to 1.0.
        diffUnit (str, optional): unit string for the difference axis; if None it is taken from the
            trailing bracketed unit of ``ylabel``.
        bsmLabel, mjLabel (str, optional): legend labels.
        logDiff (bool, optional): if True, plot ``|difference|`` on a log axis. Defaults to False.

    Returns:
        tuple: ``(figName, figure)``.
    """
    fig, (axTop, axBot) = plt.subplots(2, 1, sharex=True, figsize=(7.0, 4.4),
                                       layout="constrained")
    _plotBoth(axTop, time, yBSM, time, yMj, bsmLabel, mjLabel)
    axTop.set_ylabel(ylabel)
    if yMj is not None:
        axTop.legend(loc="best")

    tD, diff = _difference(time, yBSM, time, yMj)
    if diff is not None:
        if logDiff:
            axBot.semilogy(tD, np.maximum(np.abs(diff)*diffScale, 1e-16), color=COLOR_DIFF)
        else:
            axBot.plot(tD, diff*diffScale, color=COLOR_DIFF)
    unit = diffUnit if diffUnit is not None else _unitOf(ylabel)
    axBot.set_ylabel("Difference" + (f" [{unit}]" if unit else ""))
    axBot.set_xlabel(xlabel)
    return figName, fig


def componentComparison(figName, time, vBSM, vMj, quantity, unit, xlabel="Time [s]",
                        scale=1.0, bsmLabel="BSM", mjLabel="MuJoCo"):
    """A 2x3 grid comparing a three-component vector: columns x/y/z, rows overlay/difference.

    Args:
        figName (str): figure key.
        time (numpy.ndarray): sample times [s] for the BSM curves.
        vBSM (numpy.ndarray): BSM vector history, shape ``(N, 3)``.
        vMj (numpy.ndarray or None): MuJoCo vector history ``(M, 3)``, or None when unavailable.
        quantity (str): quantity name for the row labels, e.g. ``r"$\\omega_{BN}$"``.
        unit (str): unit string, e.g. ``"mrad/s"``.
        xlabel (str, optional): shared x-label. Defaults to ``"Time [s]"``.
        scale (float, optional): factor applied to both curves and the difference for display.
        bsmLabel, mjLabel (str, optional): legend labels.

    Returns:
        tuple: ``(figName, figure)``.
    """
    fig, axes = plt.subplots(2, 3, sharex=True, figsize=(9.6, 4.8), layout="constrained")
    for col in range(3):
        axTop, axBot = axes[0, col], axes[1, col]
        _plotBoth(axTop, time, vBSM[:, col]*scale, time,
                  None if vMj is None else vMj[:, col]*scale, bsmLabel, mjLabel)
        axTop.set_title(f"{quantity}$_{_AXES[col]}$")
        tD, diff = _difference(time, vBSM[:, col], time, None if vMj is None else vMj[:, col])
        if diff is not None:
            axBot.plot(tD, diff*scale, color=COLOR_DIFF)
        axBot.set_xlabel(xlabel)
    axes[0, 0].set_ylabel(f"{quantity} [{unit}]")
    axes[1, 0].set_ylabel(f"Difference [{unit}]")
    if vMj is not None:
        axes[0, 0].legend(loc="best", fontsize=8)
    return figName, fig


def _unitOf(label):
    """Best-effort extraction of a trailing ``[unit]`` from an axis label, else empty string."""
    if "[" in label and label.rstrip().endswith("]"):
        return label[label.rindex("[") + 1:label.rindex("]")]
    return ""


def finalizeFigures(figureList):
    """Run each figure's constrained-layout solve before the caller closes the pyplot state.

    The scenarios return their figures and then call ``plt.close("all")`` so an interactive
    ``showPlots`` session does not leak windows. The unit test / doc build, however, saves those
    figures *after* that close, and a closed figure will not re-run its constrained-layout solve at
    ``savefig`` time -- which lets long y-labels overflow and clip. Drawing each figure here, while
    it is still live, bakes in the correct label positions so the later save is not clipped.

    Args:
        figureList (dict): mapping of figure name to matplotlib figure, as returned by a scenario.

    Returns:
        dict: the same ``figureList`` (returned for convenient chaining).
    """
    for fig in figureList.values():
        try:
            fig.canvas.draw()
        except Exception:
            pass
    return figureList
