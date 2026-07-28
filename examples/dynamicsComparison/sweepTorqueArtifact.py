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
Sweep driver for the velocity-regime round-off artifact of :ref:`scenarioCompareTorque`.

Two sweeps, both storing JSON next to the other comparison metrics:

#. **Velocity sweep**: propagate the torqued body with *no gravity* but a nonzero
   inertial drift velocity. A force-free translation cannot torque the body about its
   own center of mass, so the attitude change relative to the zero-velocity run is a
   direct measure of each engine's velocity-dependent error, with gravity removed as a
   confound.
#. **dt sweep**: repeat the scenario's orbiting-vs-rest discriminator over a range of
   integrator steps, to test whether the artifact converges away with the step size
   (truncation error would; round-off does not).

Results are written to ``results/sweepTorqueArtifact.json``.
"""

import json
import os

import numpy as np

import scenarioCompareTorque as sct

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import simIncludeGravBody
from Basilisk.simulation import spacecraft
from Basilisk.simulation import svIntegrators
from Basilisk.simulation import extForceTorque
from Basilisk.architecture import messaging
from Basilisk.simulation import mujoco

resultsPath = os.path.join(os.path.dirname(__file__), "results")

# Generic inertial drift direction with three unequal nonzero components. Along a
# coordinate axis the m(v x v) round-off products are exactly zero in floating point,
# which hides the artifact; a generic direction exercises the rounded cancellation.
DRIFT_DIR = np.array([2.0, -3.0, 6.0])/7.0


def runBSMDrift(speed, dt, tf, recordDt):
    """Torqued body, no gravity, constant inertial drift velocity ``speed`` [m/s]."""
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "scBSM"
    scObject.hub.mHub = sct.MASS
    scObject.hub.IHubPntBc_B = np.diag(sct.INERTIA_DIAG).tolist()
    scSim.AddModelToTask("dynTask", scObject)

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    extTorque = extForceTorque.ExtForceTorque()
    extTorque.ModelTag = "extTorque"
    torqueMsg = messaging.CmdTorqueBodyMsg()
    torqueMsg.write(messaging.CmdTorqueBodyMsgPayload(torqueRequestBody=list(sct.TORQUE_B)))
    extTorque.cmdTorqueInMsg.subscribeTo(torqueMsg)
    scObject.addDynamicEffector(extTorque)
    scSim.AddModelToTask("dynTask", extTorque)

    scObject.hub.r_CN_NInit = [0.0, 0.0, 0.0]
    scObject.hub.v_CN_NInit = (speed*DRIFT_DIR).tolist()
    scObject.hub.sigma_BNInit = [[0.0], [0.0], [0.0]]
    scObject.hub.omega_BN_BInit = [[w] for w in sct.OMEGA0_B]

    recorder = scObject.scStateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", recorder)

    scSim.InitializeSimulation()
    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return np.array(recorder.sigma_BN)


def runMujocoDrift(speed, dt, tf, recordDt):
    """Same drift case with the MJScene engine."""
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(sct.mujocoModel())
    scene.ModelTag = "scMujoco"
    scene.extraEoMCall = True
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 1)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")

    torqueActuator = scene.addTorqueActuator("bodyTorque", hub.getOrigin())
    torqueMsg = messaging.TorqueAtSiteMsg()
    torqueMsg.write(messaging.TorqueAtSiteMsgPayload(torque_S=list(sct.TORQUE_B)))
    torqueActuator.torqueInMsg.subscribeTo(torqueMsg)

    recorder = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(recordDt))
    scSim.AddModelToTask("dynTask", recorder, 0)

    scSim.InitializeSimulation()

    hub.setPosition([0.0, 0.0, 0.0])
    hub.setVelocity((speed*DRIFT_DIR).tolist())
    hub.setAttitude([0.0, 0.0, 0.0])
    hub.setAttitudeRate(list(sct.OMEGA0_B))

    scSim.ConfigureStopTime(macros.sec2nano(tf))
    scSim.ExecuteSimulation()
    return np.array(recorder.sigma_BN)


def velocitySweep(speeds, dt, tf, recordDt):
    """Max attitude deviation vs the zero-velocity run, per engine, per speed."""
    sigmaBSMRest = runBSMDrift(0.0, dt, tf, recordDt)
    sigmaMujocoRest = runMujocoDrift(0.0, dt, tf, recordDt)
    rows = []
    for speed in speeds:
        sigmaBSM = runBSMDrift(speed, dt, tf, recordDt)
        sigmaMujoco = runMujocoDrift(speed, dt, tf, recordDt)
        rows.append({
            "speed": speed,  # [m/s]
            "bsmDriftAttMax": float(np.max(
                sct.relativePrincipalAngle(sigmaBSM, sigmaBSMRest))),  # [rad]
            "mujocoDriftAttMax": float(np.max(
                sct.relativePrincipalAngle(sigmaMujoco, sigmaMujocoRest))),  # [rad]
        })
        print("velocity {:8.1f} m/s: BSM {:.3e} rad, MuJoCo {:.3e} rad".format(
            speed, rows[-1]["bsmDriftAttMax"], rows[-1]["mujocoDriftAttMax"]))
    return rows


def dtSweep(dts, tf, recordDt, mu):
    """Repeat the scenario's orbiting-vs-rest discriminator per integrator step."""
    rows = []
    for dt in dts:
        bsmOrbitRec, _ = sct.runBSM(mu, dt, tf, recordDt, withGravity=True)
        bsmRestRec, _ = sct.runBSM(mu, dt, tf, recordDt, withGravity=False)
        mjOrbitRec, _ = sct.runMujoco(mu, dt, tf, recordDt, withGravity=True)
        mjRestRec, _ = sct.runMujoco(mu, dt, tf, recordDt, withGravity=False)
        sigmaBSM = np.array(bsmOrbitRec.sigma_BN)
        sigmaBSMRest = np.array(bsmRestRec.sigma_BN)
        sigmaMujoco = np.array(mjOrbitRec.sigma_BN)
        sigmaMujocoRest = np.array(mjRestRec.sigma_BN)
        rows.append({
            "dt": dt,  # [s]
            "bsmMotionAttMax": float(np.max(
                sct.relativePrincipalAngle(sigmaBSM, sigmaBSMRest))),  # [rad]
            "mujocoMotionAttMax": float(np.max(
                sct.relativePrincipalAngle(sigmaMujoco, sigmaMujocoRest))),  # [rad]
            "crossEngineRestAttMax": float(np.max(
                sct.relativePrincipalAngle(sigmaBSMRest, sigmaMujocoRest))),  # [rad]
        })
        print("dt {:7.4f} s: MuJoCo artifact {:.3e} rad, BSM artifact {:.3e} rad, "
              "at-rest cross-engine {:.3e} rad".format(
                  dt, rows[-1]["mujocoMotionAttMax"], rows[-1]["bsmMotionAttMax"],
                  rows[-1]["crossEngineRestAttMax"]))
    return rows


def run():
    tf = 600.0  # [s] same duration as the base scenario
    recordDt = 1.0  # [s]
    dt = 0.1  # [s] base step for the velocity sweep
    mu = simIncludeGravBody.BODY_DATA["earth"].mu

    speeds = [0.0, 750.0, 2500.0, 7500.0]  # [m/s]
    dts = [0.4, 0.2, 0.1, 0.05, 0.025, 0.0125]  # [s]

    print("=== velocity sweep (no gravity, inertial drift) ===")
    velRows = velocitySweep(speeds, dt, tf, recordDt)
    print("=== dt sweep (orbiting vs rest discriminator) ===")
    dtRows = dtSweep(dts, tf, recordDt, mu)

    os.makedirs(resultsPath, exist_ok=True)
    outFile = os.path.join(resultsPath, "sweepTorqueArtifact.json")
    with open(outFile, "w") as f:
        json.dump({
            "scenario": "sweepTorqueArtifact",
            "configuration": {
                "tf": tf, "recordDt": recordDt, "velocitySweepDt": dt,
                "body": "same rigid body, torque, and initial rate as scenarioCompareTorque",
                "velocityCase": "no gravity, constant inertial drift along +x",
                "dtCase": "orbiting (a=7000 km Kepler) vs at-rest discriminator",
            },
            "velocitySweep": velRows,
            "dtSweep": dtRows,
        }, f, indent=2)
    print("Wrote " + outFile)


if __name__ == "__main__":
    run()
