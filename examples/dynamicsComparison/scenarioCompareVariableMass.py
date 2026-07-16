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
Variable-mass scenario in the dynamics-engine comparison series (see
:ref:`scenarioCompareOrbit` for the introduction).

Where the earlier scenarios hold the spacecraft mass properties fixed, this one lets them
change. A spacecraft in a circular orbit performs a prograde orbit-raising burn: a main engine
draws propellant from a spherical tank, so the total mass, the tank inertia, and the system
center of mass all vary continuously through the burn, while the sloshing propellant reacts to
the thrust acceleration.

Modeling choices are deliberately those a GNC analyst would make rather than the ones that
maximize the effect:

- **Engine.** A monopropellant thruster, the MOOG Monarc-445, taken from the
  :ref:`simIncludeThruster` catalog. It is tied to the tank with ``addThrusterSet`` so the burn
  both applies thrust and consumes propellant, at roughly 0.19 kg/s.
- **Burn.** Fifteen minutes of continuous orbit-raising thrust, which expends roughly fifteen
  percent of the propellant.
- **Tank.** A spherical tank sized for hydrazine: 1500 kg in a 0.75 m sphere, sitting at about
  85% fill. It uses the centered ``FuelTankModelConstantVolume`` model, so its center of mass
  stays put within the tank and the inertia simply scales with the remaining mass. The tank is
  mounted aft of the dry-structure center of mass, so as it empties the system center of mass
  migrates forward along the thrust axis.
- **Attitude.** The spacecraft is velocity-aligned and pitches at the orbital rate, so the
  body-fixed engine stays pointed along the velocity vector as the orbit carries it around. It
  is not spinning or tumbling: a delta-v maneuver is flown attitude-stabilized.
- **Balance.** Every slosh element's equilibrium sits at the tank center, so the thrust line
  passes through the center of mass and the burn is torque-free.

Slosh model
-----------

The propellant slosh is the classical equivalent-mechanical model, parameterized from Dodge, *The
New "Dynamic Behavior of Liquids in Moving Containers"* (SwRI, 2000). For a spherical tank at this
fill the first lateral mode carries about a third of the propellant; the rest rides with the tank.
That first mode is a two-degree-of-freedom :ref:`sphericalPendulum` hinged at the tank center and
hanging aft, and three orthogonal :ref:`LinearSpringMassDamper` particles span the body axes. Every
slosh mass depletes with the tank, so the vehicle is a fully coupled variable-mass rigid body with
internal slosh degrees of freedom on top of the six rigid-body ones.

.. note::

    Both the spring-mass-dampers and the pendulum carry their physical bare-wall damping (ratio
    0.0026). Basilisk's ``sphericalPendulum`` applies its damping as a torque
    :math:`-d\,\pmb\omega_{\rm rel}\times{\bf l}` that a native MuJoCo ball-joint ``damping`` cannot
    reproduce, so the MuJoCo side supplies the identical torque through
    :class:`PendulumDampingCompensator` (both engines then model the same damped pendulum). A real
    hydrazine tank would carry baffles or a diaphragm and damp one to two orders of magnitude harder.

Running it
----------

::

    python3 scenarioCompareVariableMass.py

The scenario builds the same vehicle two ways and overlays them: the back-substitution
:ref:`spacecraft` (BSM, the reference) and the MuJoCo :ref:`MJScene<MJScene>`. Two switches on
:func:`run` control what is compared:

- ``useThruster`` (default True) fires the engine; set False to deplete via an equivalent
  prescribed leak rate with no thrust force, isolating pure mass loss.
- ``inOrbit`` (default True) flies the burn under Earth gravity; set False for a deep-space burn
  from rest, which removes gravity as a variable and isolates the variable-mass dynamics.

On the MuJoCo side, depletion is imposed by feeding each body's ``derivativeMassPropertiesInMsg`` a
constant mass-rate (their sum is the engine mass flow), matched to the BSM tank's proportional
depletion. Gravity (:ref:`NBodyGravity`) is applied to *every* massive body so the tree is in
free-fall and the slosh is restored purely by thrust, as on the BSM side.

What the comparison shows
-------------------------

The two engines agree on the depleting masses (to grams), the body rate, and the slosh
displacements (to microns). They differ in exactly two places, and each is a *modeling* difference,
not a solver error:

#. **In orbit -- gravity gradient.** BSM applies gravity once at the system center of mass; MuJoCo
   applies it per body. The tank mounted aft of the hub feels a slightly different field, so the
   attitudes separate to about a degree over the burn, consistent with the gravity-gradient scale
   :math:`3(\mu/r^3)\,\Delta I/\bar I` (the same effect as :ref:`scenarioCompareOrbitMultibody`).

#. **In deep space -- variable-mass reaction torque.** With gravity removed, a residual remains: the
   rotational reaction of the depleting, off-center propellant. The back-substitution hub equations
   carry it; MuJoCo's per-step inertia rescaling does not. To leading order it is a spin-axis torque
   :math:`2\dot m\,{\bf c}\times(\pmb\omega\times{\bf c})` (with :math:`{\bf c}` the center-of-mass
   offset and :math:`\dot m` the mass-flow rate). It needs depletion, spin, and the tank offset all
   present -- remove any one and the engines agree to machine precision.

:class:`VariableInertiaCompensator` supplies that torque back to the MuJoCo side, and
:func:`variableInertiaProof` uses it to prove the point: on a rigid vehicle (slosh disabled) the
torque collapses the cross-engine attitude difference from :math:`\sim 10^{-2}` to
:math:`\sim 3\times 10^{-6}` rad -- a three-thousand-fold reduction, 99.97% of the effect. The
remainder is a small higher-order coupling this single torque omits (it is dt-independent, so not
truncation; a fixed-mass body would agree to :math:`\sim 10^{-13}` rad). In the full slosh-bearing
scenario the compensator (``compensateVariableInertia``, on by default in deep space) helps less,
because the pendulum's own BSM-versus-MuJoCo difference sits on top of the rigid-body reaction.

Illustration of Simulation Results
----------------------------------

Each comparison figure shows the two engines overlaid on top and their difference beneath. The burn
raises the orbit and draws the tank down; both engines track the same semi-major-axis rise and the
same total system mass (dry hub plus depleting propellant), the difference staying at the gram
level.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_orbit.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_fuelMass.svg
   :align: center

The hub body rate and the three translational slosh displacements are each shown one column per
axis (x, y, z), the two engines over their difference. The rate holds the orbital pitch with the
slosh riding on top; both engines overlay and the differences stay small.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_rate.svg
   :align: center

.. image:: /_images/Scenarios/scenarioCompareVariableMass_slosh.svg
   :align: center

The attitude and center-of-mass difference over the burn: in orbit the gravity-gradient libration,
in deep space the variable-mass reaction torque shown with and without the compensator.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_attError.svg
   :align: center

The proof, on a single depleting rigid body: the reaction torque drops the cross-engine attitude
difference by three orders of magnitude, leaving only a dt-independent higher-order remainder.

.. image:: /_images/Scenarios/scenarioCompareVariableMass_proof.svg
   :align: center

"""

import os

import numpy as np
import matplotlib.pyplot as plt

from Basilisk.utilities import SimulationBaseClass
from Basilisk.utilities import macros
from Basilisk.utilities import unitTestSupport
from Basilisk.utilities import simIncludeGravBody
from Basilisk.utilities import simIncludeThruster
from Basilisk.utilities import pythonVariableLogger
from Basilisk.utilities import orbitalMotion
from Basilisk.utilities import RigidBodyKinematics as rbk
from Basilisk.architecture import messaging
from Basilisk.architecture import sysModel
from Basilisk.simulation import spacecraft
from Basilisk.simulation import fuelTank
from Basilisk.simulation import linearSpringMassDamper
from Basilisk.simulation import sphericalPendulum
from Basilisk.simulation import thrusterDynamicEffector
from Basilisk.simulation import svIntegrators

import _comparePlots

try:
    from Basilisk.simulation import mujoco
    from Basilisk.simulation import MJSystemCoM
    from Basilisk.simulation import NBodyGravity
    from Basilisk.simulation import pointMassGravityModel
    couldImportMujoco = True
except Exception:
    couldImportMujoco = False

# Consistent palette drawn from the standard Basilisk plotting colors.
COLOR_BSM = unitTestSupport.getLineColor(0, 3)
COLOR_MUJOCO = unitTestSupport.getLineColor(1, 3)
COLOR_AUX = unitTestSupport.getLineColor(2, 3)

fileName = os.path.basename(os.path.splitext(__file__)[0])

# Folder this scenario writes its JSON summary and reference trajectory into.
resultsPath = os.path.join(os.path.dirname(__file__), "results")

G0 = 9.80665  # [m/s^2] standard gravity used for the Isp-to-mass-flow conversion

# --- Orbit ---------------------------------------------------------------------------------
# A circular parking orbit that the burn raises. The spacecraft flies velocity-aligned and
# pitches at the orbital rate so the body-fixed engine tracks the velocity vector.
ORBIT_A = 7000.0e3  # [m] semi-major axis (circular)
ORBIT_I = 33.3*macros.D2R  # [rad] inclination
ORBIT_RAAN = 48.2*macros.D2R  # [rad] right ascension of the ascending node
ORBIT_ARGLAT = 85.3*macros.D2R  # [rad] argument of latitude at epoch

# --- Hub (dry structure) --------------------------------------------------------------------
HUB_MASS = 1500.0  # [kg] dry hub mass
# Solar arrays along the body x-axis make Ixx the smallest principal inertia and leave the pitch
# axis (body y, the orbit normal) the MAJOR axis of the loaded vehicle. That ordering matters:
# slosh dissipates energy, and an energy-dissipating body migrates toward major-axis rotation, so
# pitching about the intermediate axis instead lets the slosh actively drive the vehicle off its
# burn attitude (measured here: 49% of the pitch rate survives, against 85% about the major axis).
HUB_INERTIA = (1200.0, 2000.0, 1900.0)  # [kg*m^2] principal dry-hub inertia about Bc
HUB_R_BcB_B = (0.0, 0.0, 0.0)  # [m] dry-structure center of mass, at the body origin

# --- Tank and propellant ----------------------------------------------------------------------
# Spherical tank (the dominant shape for pressurized systems) holding monopropellant hydrazine,
# which is what the Monarc-445 burns. The radius is set by the propellant load: 1500 kg of
# hydrazine needs at least 1.494 m^3, so a 0.75 m sphere (1.767 m^3) sits at ~85% fill. It does
# NOT fit in a 0.70 m sphere.
#
# The FuelTankModelConstantVolume model keeps the load centered in the tank and scales the
# inertia with the remaining mass, which is the right idealization on orbit: a propellant
# management device holds the propellant in place, so it does not drain to one side (that is a
# launch/settled regime). Mounting the tank AFT of the dry-structure center of mass is what makes
# the SYSTEM center of mass migrate forward along the thrust axis (and so stay torque-free) as the
# propellant is spent, the physically honest source of a moving center of mass.
TANK_RADIUS = 0.75  # [m] spherical tank radius
TANK_R_TB_B = (0.0, 0.0, -0.8)  # [m] tank center, mounted aft of the dry CoM along -z
PROPELLANT_MASS = 1500.0  # [kg] total propellant (~50% of wet mass)
PROPELLANT_DENSITY = 1004.0  # [kg/m^3] hydrazine

# --- Slosh: equivalent mechanical model --------------------------------------------------------
# First-mode lateral slosh parameters for a SPHERICAL tank, from Dodge, "The New Dynamic Behavior
# of Liquids in Moving Containers", SwRI 2000 (the successor to Abramson, NASA SP-106), Figs. 1.11
# and 3.4. Two facts make the pendulum the right element here:
#
#   * The pendulum arm length is PURELY GEOMETRIC: L1/R is a function of fill fraction alone, so
#     the slosh frequency omega = sqrt(a/L1) tracks the thrust acceleration automatically as the
#     vehicle mass drops. A spring-mass-damper's sqrt(k/m) does not, so k must be retuned for every
#     acceleration level.
#   * For a sphere the pendulum hinge AND the non-sloshing mass both sit at the tank center at
#     every fill level (rotating a sphere about its center does not move an inviscid liquid), and
#     I0 = 0. This is exactly what Basilisk's sphericalPendulum + FuelTank express.
#
# Basilisk's FuelTank depletes every slosh mass PROPORTIONALLY, so the slosh-mass fraction is held
# constant and cannot track the true fill dependence (which runs 0.281 -> 0.339 over this burn).
# The parameters are therefore evaluated at the MID-BURN fill (~80%) to split the difference.
SLOSH_MASS_FRACTION = 0.335  # [-] m1/m_liq at ~80% fill (Dodge Fig. 3.4)
PEND_LENGTH_RATIO = 0.452  # [-] L1/R at ~80% fill (Dodge Fig. 3.4)
SLOSH_DAMPING_RATIO = 0.0026  # [-] zeta, bare smooth wall (Dodge Eq. 2.9b), see docstring

# Benchmarking fixture, NOT physics: the three orthogonal spring-mass-dampers are retained purely
# so the comparison exercises both Basilisk slosh effectors (a slide joint and a ball joint on the
# MuJoCo side). The classical lateral-slosh EMM has no axial degree of freedom, and one 2-DOF
# pendulum already spans both lateral axes. Their mass comes out of the non-sloshing mass so the
# propellant budget still closes. See the docstring.
SMD_MASS = 25.0  # [kg] each of the three particles
SMD_LATERAL_RHO0 = 0.02  # [m] residual lateral slosh carried into the burn

# P0 frame: rest axis pHat_01 along -z, so the bob hangs aft, the direction the propellant is
# pushed by a +z burn. pHat_03 = pHat_01 x pHat_02 keeps the triad right-handed. A left-handed
# triad silently produces an inverted, exponentially diverging pendulum.
PEND_PHAT_01 = (0.0, 0.0, -1.0)
PEND_PHAT_02 = (1.0, 0.0, 0.0)
PEND_PHAT_03 = (0.0, -1.0, 0.0)
PEND_RATE0 = 0.02  # [rad/s] initial phi and theta rates (residual slosh)

# --- Main-engine burn -------------------------------------------------------------------------
# MOOG Monarc-445: a real monopropellant thruster (445 N, Isp 234 s) from the simIncludeThruster
# catalog. Its catalog values are used as-is, giving mDot ~ 0.19 kg/s.
THRUSTER_TYPE = "MOOG_Monarc_445"
THRUST_DIR_B = (0.0, 0.0, 1.0)  # body +z, which starts aligned with the velocity vector
THRUST_POS_B = (0.0, 0.0, -1.5)  # [m] on the z-axis, aft: the burn is torque-free

# --- Simulation --------------------------------------------------------------------------------
SIM_DURATION = 900.0  # [s] a 15-minute orbit-raising burn
STEPS_PER_PERIOD = 150.0  # target RK4 steps per stiffest slosh oscillation
MAX_TIME_STEP = 0.05  # [s] cap on the integration step


def nominalMassFlow(maxThrust, steadyIsp):
    """Nominal propellant mass-flow rate of the engine [kg/s].

    Args:
        maxThrust (float): engine thrust [N]
        steadyIsp (float): specific impulse [s]

    Returns:
        float: mass flow rate [kg/s].
    """
    return maxThrust/(G0*steadyIsp)


def thrusterSpec():
    """Return the catalog ``(MaxThrust [N], steadyIsp [s])`` of the modeled engine."""
    factory = simIncludeThruster.thrusterFactory()
    device = factory.create(THRUSTER_TYPE, list(THRUST_POS_B), list(THRUST_DIR_B))
    return device.MaxThrust, device.steadyIsp


def earthMu():
    """Earth gravitational parameter [m^3/s^2], from the same body the sim uses."""
    return simIncludeGravBody.gravBodyFactory().createEarth().mu


def sloshParameters():
    """Derive the slosh model inputs from the literature ratios and the vehicle sizing.

    The pendulum is the primary element: its arm length is purely geometric, so its frequency
    ``omega1 = sqrt(a/L1)`` emerges from the thrust acceleration rather than being an input. The
    spring-mass-dampers are then tuned to that same physical slosh frequency, so the benchmarking
    fixture at least oscillates at the right rate.

    Returns:
        dict: slosh masses [kg], pendulum arm [m], slosh frequency [rad/s], spring constant
        [N/m], damping coefficients, the axial particle's settled offset [m], and the tank fill
        fraction [-].
    """
    maxThrust, _ = thrusterSpec()
    wetMass = HUB_MASS + PROPELLANT_MASS  # [kg]
    accel = maxThrust/wetMass  # [m/s^2] axial acceleration that restores the slosh

    pendMass = SLOSH_MASS_FRACTION*PROPELLANT_MASS  # [kg] first-mode slosh mass m1
    pendLength = PEND_LENGTH_RATIO*TANK_RADIUS  # [m] equivalent pendulum length L1
    omega1 = np.sqrt(accel/pendLength)  # [rad/s] first-mode slosh frequency (emergent)

    # The benchmarking-fixture particles come out of the non-sloshing mass so the budget closes.
    bulkMass = PROPELLANT_MASS - pendMass - 3.0*SMD_MASS  # [kg] non-sloshing mass m0

    tankVolume = 4.0/3.0*np.pi*TANK_RADIUS**3  # [m^3]
    fillFraction = PROPELLANT_MASS/(PROPELLANT_DENSITY*tankVolume)  # [-]

    return {
        "accel": accel,
        "pendMass": pendMass,
        "pendLength": pendLength,
        "omega1": omega1,
        "bulkMass": bulkMass,
        "fillFraction": fillFraction,
        "smdK": SMD_MASS*omega1**2,  # [N/m] matched to the physical slosh frequency
        "smdC": 2.0*SLOSH_DAMPING_RATIO*SMD_MASS*omega1,  # [N*s/m]
        # The first-mode pendulum carries its physical bare-wall damping. Basilisk's
        # sphericalPendulum applies this as a torque -D*l' about the tank center (a cross-product
        # coupling), which no MuJoCo ball-joint damping value reproduces; the MuJoCo side instead
        # supplies the identical torque through PendulumDampingCompensator (see that class), so both
        # engines model the same damped pendulum.
        "pendD": 2.0*SLOSH_DAMPING_RATIO*pendMass*pendLength*omega1,  # pendulum damping [N*m*s]
        # Under thrust the axial particle settles at rho = -a/omega1^2 (which equals -L1). It is
        # started there, as a settling burn would leave it, rather than ringing down from zero.
        "axialRho0": -accel/omega1**2,
    }


def timeStep():
    """Integrator step that resolves the slosh oscillation [s]."""
    naturalPeriod = 2.0*np.pi/sloshParameters()["omega1"]  # [s]
    return min(MAX_TIME_STEP, naturalPeriod/STEPS_PER_PERIOD)


def initialOrbitState(mu):
    """Initial inertial position and velocity on the circular parking orbit.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(rN [m], vN [m/s])`` as numpy arrays.
    """
    oe = orbitalMotion.ClassicElements()
    oe.a = ORBIT_A  # [m]
    oe.e = 0.0
    oe.i = ORBIT_I  # [rad]
    oe.Omega = ORBIT_RAAN  # [rad]
    oe.omega = 0.0  # [rad]
    oe.f = ORBIT_ARGLAT  # [rad]
    rN, vN = orbitalMotion.elem2rv(mu, oe)
    return np.array(rN), np.array(vN)


def velocityAlignedAttitude(rN, vN, mu):
    """Body attitude and rate that keep the engine pointed along the velocity vector.

    The body frame is built with z along the velocity (the thrust axis), y along the orbit
    normal, and x completing the triad. Holding that alignment as the orbit carries the
    spacecraft around requires pitching about the orbit normal at the orbital rate, which in body
    components is a rate purely about the body y-axis.

    Args:
        rN (numpy.ndarray): inertial position [m]
        vN (numpy.ndarray): inertial velocity [m/s]
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        tuple: ``(sigma_BN, omega_BN_B [rad/s], meanMotion [rad/s])``.
    """
    vHat = vN/np.linalg.norm(vN)
    hHat = np.cross(rN, vN)
    hHat = hHat/np.linalg.norm(hHat)
    xHat = np.cross(hHat, vHat)  # completes the right-handed triad (nadir for a circular orbit)
    dcm_BN = np.array([xHat, hHat, vHat])  # rows are the body axes in inertial components
    meanMotion = np.sqrt(mu/ORBIT_A**3)  # [rad/s]
    return rbk.C2MRP(dcm_BN), np.array([0.0, meanMotion, 0.0]), meanMotion


def buildBSM(dt, record, useThruster=True, inOrbit=True):
    """Build (and initialize) the back-substitution variable-mass reference simulation.

    Args:
        dt (float): integrator time step [s]
        record (bool): if True, attach the hub-state, fuel-tank and slosh recorders
        useThruster (bool, optional): if True, deplete the tank with the firing main engine that
            also applies thrust. If False, deplete it with an equivalent prescribed leak rate
            and no thrust force. Defaults to True.
        inOrbit (bool, optional): if True (default) fly the burn on the circular parking orbit
            under point-mass Earth gravity. If False, run the identical vehicle as a deep-space
            burn with no gravity field and starting from rest, which removes the gravity-gradient
            modeling difference so the two engines agree to round-off (see :func:`run`).

    Returns:
        tuple: ``(scSim, recorders, handles)`` where ``recorders`` is a dict of the attached
        recorders (empty when ``record`` is False) and ``handles`` keeps the created modules and
        stand-alone messages alive.
    """
    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scObject = spacecraft.Spacecraft()
    scObject.ModelTag = "hub"
    scObject.hub.mHub = HUB_MASS  # [kg]
    scObject.hub.r_BcB_B = [[c] for c in HUB_R_BcB_B]  # [m]
    scObject.hub.IHubPntBc_B = np.diag(HUB_INERTIA).tolist()  # [kg*m^2]
    scSim.AddModelToTask("dynTask", scObject)

    integrator = svIntegrators.svIntegratorRK4(scObject)
    scObject.setIntegrator(integrator)

    gravFactory = simIncludeGravBody.gravBodyFactory()
    earth = gravFactory.createEarth()
    mu = earth.mu  # [m^3/s^2]
    if inOrbit:
        earth.isCentralBody = True
        gravFactory.addBodiesTo(scObject)

    # The orbit geometry always defines the initial attitude and body rate. In deep-space mode the
    # translational state is zeroed: the MuJoCo gyroscopic-bias round-off scales with the hub
    # speed, so starting from rest (rather than at ~7.5 km/s orbital speed) keeps the cross-engine
    # difference at the round-off floor once gravity is off.
    rN, vN = initialOrbitState(mu)
    sigma_BN, omega_BN_B, _ = velocityAlignedAttitude(rN, vN, mu)
    if not inOrbit:
        rN = np.zeros(3)
        vN = np.zeros(3)
    scObject.hub.r_CN_NInit = rN.tolist()  # [m]
    scObject.hub.v_CN_NInit = vN.tolist()  # [m/s]
    scObject.hub.sigma_BNInit = [[c] for c in sigma_BN]
    scObject.hub.omega_BN_BInit = [[c] for c in omega_BN_B]  # [rad/s] orbital pitch rate

    slosh = sloshParameters()

    # Three orthogonal spring-mass-damper particles (a benchmarking fixture, not the physical
    # slosh model, see the docstring). Every equilibrium sits at the tank center, because a
    # slosh mass's equilibrium is the propellant center of mass. That keeps the thrust line
    # through the center of mass and the burn torque-free. Off-centering them instead produces a
    # standing thrust moment that steadily spins the vehicle up. The lateral pair carries residual
    # slosh from earlier maneuvers. The axial one starts at the offset the thrust settles it to,
    # as a settling burn would leave it.
    smdInit = (
        ((1.0, 0.0, 0.0), SMD_LATERAL_RHO0),
        ((0.0, 1.0, 0.0), -SMD_LATERAL_RHO0),
        ((0.0, 0.0, 1.0), slosh["axialRho0"]),
    )
    particles = []
    for (pHat, rho0) in smdInit:
        particle = linearSpringMassDamper.LinearSpringMassDamper()
        particle.k = slosh["smdK"]  # [N/m] tuned to the physical first-mode slosh frequency
        particle.c = slosh["smdC"]  # [N*s/m]
        particle.r_PB_B = [[c] for c in TANK_R_TB_B]  # [m] equilibrium at the tank center
        particle.pHat_B = [[c] for c in pHat]
        particle.rhoInit = rho0  # [m]
        particle.rhoDotInit = 0.0  # [m/s]
        particle.massInit = SMD_MASS  # [kg]
        particles.append(particle)

    # First-mode lateral slosh: a 2-DOF spherical pendulum pivoted at the tank center and hanging
    # aft along the settling axis. This is the physically standard element (Dodge, SwRI 2000).
    pendulum = sphericalPendulum.SphericalPendulum()
    pendulum.pendulumRadius = slosh["pendLength"]  # [m] L1, purely geometric
    pendulum.d = [[c] for c in TANK_R_TB_B]  # [m] pivot at the tank center (correct for a sphere)
    pendulum.D = (slosh["pendD"]*np.eye(3)).tolist()
    # phiInit/thetaInit are not exposed and default to zero, so the bob starts hanging aft.
    pendulum.phiDotInit = PEND_RATE0  # [rad/s] residual slosh
    pendulum.thetaDotInit = PEND_RATE0  # [rad/s]
    pendulum.massInit = slosh["pendMass"]  # [kg] first-mode slosh mass m1
    pendulum.pHat_01 = [[c] for c in PEND_PHAT_01]
    pendulum.pHat_02 = [[c] for c in PEND_PHAT_02]
    pendulum.pHat_03 = [[c] for c in PEND_PHAT_03]

    # Spherical tank carrying the non-sloshing mass m0: centered load, I0 = 0, inertia scaling
    # with the remaining mass.
    tank = fuelTank.FuelTank()
    tank.ModelTag = "propTank"
    tankModel = fuelTank.FuelTankModelConstantVolume()
    tankModel.propMassInit = slosh["bulkMass"]  # [kg] non-sloshing mass m0
    tankModel.maxFuelMass = slosh["bulkMass"]  # [kg]
    tankModel.radiusTankInit = TANK_RADIUS  # [m]
    tankModel.r_TcT_TInit = [[0.0], [0.0], [0.0]]  # [m] load centered in the tank
    tank.setTankModel(tankModel)
    tank.setR_TB_B([[c] for c in TANK_R_TB_B])  # [m]

    sloshEffectors = particles + [pendulum]
    for effector in sloshEffectors:
        tank.pushFuelSloshParticle(effector)
    tank.setUpdateOnly(True)

    scObject.addStateEffector(tank)
    for effector in sloshEffectors:
        scObject.addStateEffector(effector)
    scSim.AddModelToTask("dynTask", tank)

    handles = [scObject, integrator, gravFactory, tank, tankModel] + sloshEffectors
    maxThrust, steadyIsp = thrusterSpec()
    if useThruster:
        thFactory = simIncludeThruster.thrusterFactory()
        thFactory.create(THRUSTER_TYPE, list(THRUST_POS_B), list(THRUST_DIR_B))
        thruster = thrusterDynamicEffector.ThrusterDynamicEffector()
        thFactory.addToSpacecraft("mainEngine", thruster, scObject)
        tank.addThrusterSet(thruster)

        onTime = messaging.THRArrayOnTimeCmdMsgPayload()
        onTime.OnTimeRequest = [2.0*SIM_DURATION]  # [s] keep the engine lit for the whole burn
        thrCmdMsg = messaging.THRArrayOnTimeCmdMsg().write(onTime)
        thruster.cmdsInMsg.subscribeTo(thrCmdMsg)
        scSim.AddModelToTask("dynTask", thruster)
        handles += [thFactory, thruster, thrCmdMsg]
    else:
        # Same nominal mass flow as the engine, but with no thrust force.
        tank.setFuelLeakRate(nominalMassFlow(maxThrust, steadyIsp))  # [kg/s]

    recorders = {}
    if record:
        recorders["state"] = scObject.scStateOutMsg.recorder(macros.sec2nano(dt))
        recorders["tank"] = tank.fuelTankOutMsg.recorder(macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorders["state"])
        scSim.AddModelToTask("dynTask", recorders["tank"])

        # Slosh internal states are not messages. Read them from the dynamics state manager. The
        # state objects only exist after InitializeSimulation, so each lookup is deferred into a
        # callable evaluated at logging time.
        def rhoGetter(particle):
            return lambda _: scObject.dynManager.getStateObject(
                particle.nameOfRhoState).getState()[0][0]

        def pendGetter(name):
            return lambda _: scObject.dynManager.getStateObject(name).getState()[0][0]

        loggerSpec = {f"rho{i+1}": rhoGetter(p) for i, p in enumerate(particles)}
        loggerSpec["phi"] = pendGetter(pendulum.nameOfPhiState)
        loggerSpec["theta"] = pendGetter(pendulum.nameOfThetaState)
        recorders["slosh"] = pythonVariableLogger.PythonVariableLogger(
            loggerSpec, macros.sec2nano(dt))
        scSim.AddModelToTask("dynTask", recorders["slosh"])

    scSim.InitializeSimulation()
    return scSim, recorders, handles


def mujocoModel():
    """Return the MJCF model of the variable-mass vehicle for the :ref:`MJScene<MJScene>`.

    The tree mirrors the BSM effector stack: a free-floating ``hub`` carries a rigidly welded
    ``tank`` body (the non-sloshing propellant mass m0), three ``slide``-jointed spring-mass-damper
    particles, and a ``ball``-jointed pendulum bob hanging aft along the settling axis. A ``site``
    on the hub marks the main-engine application point, and a ``motor`` acting on it applies the
    thrust along the body +z axis.

    The masses written here are the INITIAL propellant masses. MuJoCo fixes body mass and inertia
    at model-compile time, so the depletion is layered on in :func:`buildMujoco` by feeding each
    propellant body's mass-rate into its ``derivativeMassPropertiesInMsg``: :ref:`MJScene<MJScene>`
    integrates the per-body mass as a state and rescales the body inertia (linearly, with the
    center of mass fixed) every step. That matches the ``FuelTankModelConstantVolume`` law on the
    BSM side, where the tank inertia is ``(2/5) m r^2`` about a fixed centroid.

    Returns:
        str: MJCF XML string.
    """
    ix, iy, iz = HUB_INERTIA
    tx, ty, tz = TANK_R_TB_B
    px, py, pz = THRUST_POS_B
    slosh = sloshParameters()
    m0 = slosh["bulkMass"]  # [kg] non-sloshing mass carried rigidly by the tank
    tankInertia = 0.4*m0*TANK_RADIUS**2  # [kg*m^2] (2/5) m0 r^2 solid-sphere inertia
    k = slosh["smdK"]  # [N/m] tuned to the physical first-mode slosh frequency
    c = slosh["smdC"]  # [N*s/m]
    length = slosh["pendLength"]  # [m] pendulum arm L1
    pendMass = slosh["pendMass"]  # [kg] first-mode slosh mass m1

    # Point-mass slosh particles carry a negligible spin inertia; the ball-jointed pendulum needs
    # a small but nonzero inertia about the rod axis so its spectator third (spin) degree of
    # freedom does not leave the mass matrix singular. The transverse inertia is dominated by the
    # m*L^2 parallel-axis term MuJoCo forms from the offset bob, matching the BSM point-mass model.
    return f"""
<mujoco>
  <option gravity="0 0 0"/>
  <worldbody>
    <body name="hub">
      <freejoint/>
      <inertial pos="0 0 0" mass="{HUB_MASS}" fullinertia="{ix} {iy} {iz} 0 0 0"/>
      <site name="thrustPoint" pos="{px} {py} {pz}"/>
      <!-- Non-sloshing propellant mass m0, welded aft of the hub center of mass. -->
      <body name="tank" pos="{tx} {ty} {tz}">
        <inertial pos="0 0 0" mass="{m0}"
                  diaginertia="{tankInertia} {tankInertia} {tankInertia}"/>
      </body>
      <!-- Three orthogonal spring-mass-damper particles as slide joints (springref=0 puts the
           equilibrium at the tank center); genuine dashpots, matching linearSpringMassDamper. -->
      <body name="sloshX" pos="{tx} {ty} {tz}">
        <joint name="sloshX" type="slide" axis="1 0 0" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{SMD_MASS}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <body name="sloshY" pos="{tx} {ty} {tz}">
        <joint name="sloshY" type="slide" axis="0 1 0" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{SMD_MASS}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <body name="sloshZ" pos="{tx} {ty} {tz}">
        <joint name="sloshZ" type="slide" axis="0 0 1" stiffness="{k}" damping="{c}" springref="0"/>
        <inertial pos="0 0 0" mass="{SMD_MASS}" diaginertia="1e-6 1e-6 1e-6"/>
      </body>
      <!-- First-mode lateral slosh: a ball joint at the tank center with the bob hanging aft
           along -z. Its restoring force comes from the THRUST-induced acceleration, not gravity.
           No native joint damping: the BSM pendulum's damping is a torque -d (omega_rel x l) that
           no ball-joint damping value matches, so it is supplied instead by
           PendulumDampingCompensator (see buildMujoco). -->
      <body name="pendulum" pos="{tx} {ty} {tz}">
        <joint name="pendulum" type="ball"/>
        <inertial pos="0 0 {-length}" mass="{pendMass}" diaginertia="1e-4 1e-4 1e-4"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <motor name="mainEngine" site="thrustPoint" gear="0 0 1 0 0 0"/>
  </actuator>
</mujoco>
"""


def bodyMassFlowRates():
    """Constant per-body propellant mass-flow rates for the MuJoCo depletion wiring [kg/s].

    On the BSM side the fuel tank depletes its own bulk mass and every attached slosh particle
    *proportionally* to their current mass, so the mass fractions are exact invariants and each
    body drains at a constant rate equal to its initial mass fraction times the total mass flow
    (see :func:`pullBSM`). MuJoCo has no equivalent coupling, so the same schedule is reproduced
    directly: each body is given a constant, negative mass-rate feeding its
    ``derivativeMassPropertiesInMsg``, and the rates sum to the engine's total mass flow.

    Returns:
        dict: body name -> mass-rate [kg/s] (negative), for ``tank`` and the four slosh bodies.
    """
    maxThrust, steadyIsp = thrusterSpec()
    totalFlow = nominalMassFlow(maxThrust, steadyIsp)  # [kg/s] engine propellant mass flow
    slosh = sloshParameters()
    # Initial mass fraction of each body within the PROPELLANT (the tank bulk mass plus every
    # slosh mass equals PROPELLANT_MASS; the dry hub does not deplete).
    rates = {"tank": -totalFlow*slosh["bulkMass"]/PROPELLANT_MASS}
    for name in ("sloshX", "sloshY", "sloshZ"):
        rates[name] = -totalFlow*SMD_MASS/PROPELLANT_MASS
    rates["pendulum"] = -totalFlow*slosh["pendMass"]/PROPELLANT_MASS
    return rates


# Body names of the depleting propellant bodies, in the order their mass-rates are tracked.
PROPELLANT_BODIES = ("tank", "sloshX", "sloshY", "sloshZ", "pendulum")

# Coefficient of the variable-mass reaction torque tau = COEF * mDot * c x (omega x c). The value
# 2 is not a guess: fitting the reaction torque the back-substitution spacecraft actually applies
# (recovered from its angular acceleration) against mDot*|c|^2*omega gives a ratio of 1.9993, flat
# across the entire burn and matching all three torque components. It matches the -2 mDot(...)
# coefficient the hub translational equation carries (spacecraft.cpp), which the back-substitution
# feeds into the rotational solve. This is the LEADING term: supplying it removes 99.97% of the
# rigid-body cross-engine difference, leaving a dt-independent ~3e-6 rad higher-order remainder (NOT
# truncation -- it does not shrink with the step; the machine-precision floor is ~1e-13 rad).
VARIABLE_INERTIA_TORQUE_COEFF = 2.0


class VariableInertiaCompensator(sysModel.SysModel):
    r"""Supplies the variable-mass reaction torque MuJoCo's depletion model leaves out.

    MuJoCo rescales each body's mass and inertia every step, but its solver never adds the
    rotational reaction that a shrinking, off-center propellant exerts on a spinning vehicle, which
    the back-substitution :ref:`spacecraft` carries. This module supplies that reaction as a
    body-frame torque on the hub through a ``MJTorqueActuator``, read live each sub-step:

    .. math::

        \pmb\tau \;=\; 2\,\dot m\;{\bf c}\times(\pmb\omega\times{\bf c}),

    with :math:`\dot m` the propellant mass-flow rate, :math:`{\bf c}` the system center-of-mass
    offset from the hub origin, and :math:`\pmb\omega` the hub rate. The coefficient 2 is measured
    from the back-substitution solver, not tuned (see :data:`VARIABLE_INERTIA_TORQUE_COEFF`); it is
    the *leading* term and captures 99.97% of the effect (see :func:`variableInertiaProof`).

    To use it: build the MuJoCo side with ``compensateVariableInertia=True`` (or call
    :func:`run` in deep space, where it is on by default), which attaches this module and its torque
    actuator to the hub.
    """

    def __init__(self, hub, scene, massFlowRates):
        """Create the compensator.

        Args:
            hub (MJBody): the free-floating hub body carrying the propellant bodies
            scene (MJScene): the scene the hub belongs to
            massFlowRates (dict): per-body propellant mass-rate [kg/s] from :func:`bodyMassFlowRates`
        """
        super().__init__()
        self.totalMassFlow = sum(massFlowRates.values())  # [kg/s]
        self.r_TB_B = np.array(TANK_R_TB_B)  # [m] propellant-body offset from the hub origin
        self.hubStateInMsg = messaging.SCStatesMsgReader()
        self.hubStateInMsg.subscribeTo(hub.getOrigin().stateOutMsg)
        self.hubMassInMsg = messaging.SCMassPropsMsgReader()
        self.hubMassInMsg.subscribeTo(hub.massPropertiesOutMsg)
        self.bodyMassInMsgs = {}
        for name in PROPELLANT_BODIES:
            reader = messaging.SCMassPropsMsgReader()
            reader.subscribeTo(scene.getBody(name).massPropertiesOutMsg)
            self.bodyMassInMsgs[name] = reader
        self.torqueOutMsg = messaging.TorqueAtSiteMsg()

    def UpdateState(self, CurrentSimNanos):
        """Compute and publish the variable-mass reaction torque for the current sub-step."""
        omega_BN_B = np.array(self.hubStateInMsg().omega_BN_B)  # [rad/s]
        propellantMass = sum(self.bodyMassInMsgs[name]().massSC for name in PROPELLANT_BODIES)
        totalMass = self.hubMassInMsg().massSC + propellantMass  # [kg]
        # System center of mass in the hub-origin body frame (the hub sits at the body origin and
        # every propellant body is mounted at r_TB_B, so the CoM migrates toward the origin as the
        # propellant is spent).
        c_B = propellantMass*self.r_TB_B/totalMass  # [m]
        torque_B = (VARIABLE_INERTIA_TORQUE_COEFF*self.totalMassFlow
                    *np.cross(c_B, np.cross(omega_BN_B, c_B)))  # [N*m]
        payload = messaging.TorqueAtSiteMsgPayload()
        payload.torque_S = list(torque_B)  # hub-origin site frame == body frame
        self.torqueOutMsg.write(payload, CurrentSimNanos, self.moduleID)


class PendulumDampingCompensator(sysModel.SysModel):
    r"""Reproduces the BSM spherical-pendulum damping torque on the MuJoCo ball joint.

    Basilisk's ``sphericalPendulum`` damps the first slosh mode with a torque about the tank center

    .. math::

        \pmb\tau_{\rm damp} \;=\; -\,d\;\pmb\omega_{\rm rel}\times{\bf l},

    where :math:`d` is the pendulum damping coefficient, :math:`\pmb\omega_{\rm rel}` the bob's
    angular rate relative to the hub, and :math:`{\bf l}` the rod vector (length ``pendulumRadius``
    along the joint's hang axis). A MuJoCo ball joint's native ``damping`` applies
    :math:`-b\,\pmb\omega_{\rm rel}`, a *different* form -- diagonal in the joint rate rather than
    the cross-product coupling above -- so no ``damping`` value reproduces it. This module supplies
    the exact torque instead, reading the ball joint's relative rate live each sub-step and applying
    it to the pendulum body through a ``MJTorqueActuator``.

    With it, the physical pendulum damping can be modeled on *both* engines (rather than switched off
    to sidestep the mismatch) while keeping the same cross-engine agreement.
    """

    def __init__(self, scene, dampingCoeff, rodLength):
        """Create the pendulum-damping compensator.

        Args:
            scene (MJScene): the scene containing the ``pendulum`` ball-jointed body.
            dampingCoeff (float): pendulum damping coefficient ``pendD`` [N*m*s].
            rodLength (float): pendulum rod length ``pendulumRadius`` [m]; the bob hangs at
                ``(0, 0, -rodLength)`` in the pendulum body frame.
        """
        super().__init__()
        self.dampingCoeff = dampingCoeff
        self.rodVector_P = np.array([0.0, 0.0, -rodLength])  # [m] rod in the pendulum body frame
        self.scene = scene
        self.qvelState = None  # bulk MuJoCo velocity state, resolved lazily after init
        self.ballQvelAdr = None
        self.torqueOutMsg = messaging.TorqueAtSiteMsg()

    def UpdateState(self, CurrentSimNanos):
        """Compute and publish the pendulum damping torque for the current sub-step."""
        if self.qvelState is None:
            # The ball joint's qvel address only exists once the model is compiled (after
            # InitializeSimulation), so resolve it on the first call rather than in __init__.
            self.ballQvelAdr = self.scene.getBody("pendulum").getBallJoint().getQvelAdr()
            self.qvelState = self.scene.dynManager.getStateObject("mujocoQvel")
        qvel = np.array(self.qvelState.getState()).flatten()
        omegaRel = qvel[self.ballQvelAdr:self.ballQvelAdr + 3]  # [rad/s] bob rate rel hub (body frame)
        torque_P = -self.dampingCoeff*np.cross(omegaRel, self.rodVector_P)  # [N*m]
        payload = messaging.TorqueAtSiteMsgPayload()
        payload.torque_S = list(torque_P)  # pendulum-origin site frame == pendulum body frame
        self.torqueOutMsg.write(payload, CurrentSimNanos, self.moduleID)


def buildMujoco(dt, record, initialState, useThruster=True, inOrbit=True,
                compensateVariableInertia=False):
    """Build (and initialize) the MuJoCo variable-mass simulation.

    The MJCF tree from :func:`mujocoModel` is fixed-mass; this function layers on the depletion
    (a constant mass-rate per body, from :func:`bodyMassFlowRates`), the point-mass gravity on every
    body, and the thrust (a constant ``SingleActuatorMsg`` on the ``mainEngine`` motor, omitted when
    ``useThruster`` is False) -- see the module docstring for why each is done this way. Two build
    details worth flagging: the pendulum's initial rate is written straight into the bulk MuJoCo
    velocity state (``MJBallJoint`` has no rate setter), and initial conditions are all set *after*
    ``InitializeSimulation`` and taken from ``initialState`` so both engines start identically.

    Args:
        dt (float): integrator time step [s]
        record (bool): if True, attach the hub-state, system-center-of-mass and slosh recorders
        initialState (dict): initial ``r_BN_N`` [m], ``v_BN_N`` [m/s], ``sigma_BN`` and
            ``omega_BN_B`` [rad/s] of the hub-origin frame, taken from the BSM reference so both
            engines start from the identical state.
        useThruster (bool, optional): if True (default) fire the main engine; if False, deplete
            with no thrust force, matching the ``useThruster=False`` BSM leak-rate reference.
        inOrbit (bool, optional): if True (default) add the point-mass gravity field; if False,
            omit it for the deep-space burn, matching :func:`buildBSM`.
        compensateVariableInertia (bool, optional): if True, attach the
            :class:`VariableInertiaCompensator` reaction torque that emulates the leading
            variable-mass rotational coupling MuJoCo's depletion model omits (see that class and the
            docstring). A first-order correction that shrinks the deep-space cross-engine attitude
            difference by about an order of magnitude. Defaults to False.

    Returns:
        tuple: ``(scSim, recorders, handles)`` matching :func:`buildBSM`.
    """
    if not couldImportMujoco:
        raise ImportError("Build Basilisk with --mujoco to run the MuJoCo comparison side.")

    scSim = SimulationBaseClass.SimBaseClass()
    process = scSim.CreateNewProcess("dyn")
    process.addTask(scSim.CreateNewTask("dynTask", macros.sec2nano(dt)))

    scene = mujoco.MJScene(mujocoModel())
    scene.ModelTag = "hubMj"
    scene.extraEoMCall = True
    # Advance the hub free-joint quaternion at the integrator's full order: the slosh coupling
    # continually turns the hub rate vector, so MuJoCo's default second-order attitude step would
    # dominate the cross-engine attitude difference.
    scene.highOrderAttitudeIntegration = True
    scSim.AddModelToTask("dynTask", scene, 2)

    integrator = svIntegrators.svIntegratorRK4(scene)
    scene.setIntegrator(integrator)

    hub = scene.getBody("hub")

    handles = [scene, integrator]
    if inOrbit:
        # MuJoCo's built-in gravity is disabled inside MJScene, so supply the same point-mass field
        # the BSM gravity effector uses, applied at every massive body's own position.
        gravFactory = simIncludeGravBody.gravBodyFactory()
        earth = gravFactory.createEarth()
        gravity = NBodyGravity.NBodyGravity()
        gravity.ModelTag = "gravity"
        scene.AddModelToDynamicsTask(gravity)
        gravityModel = pointMassGravityModel.PointMassGravityModel()
        gravityModel.muBody = earth.mu  # [m^3/s^2]
        gravity.addGravitySource("earth", gravityModel, True)
        massiveBodies = ["hub", "tank", "sloshX", "sloshY", "sloshZ", "pendulum"]
        gravityTargets = [gravity.addGravityTarget(name, scene.getBody(name))
                          for name in massiveBodies]
        handles += [gravFactory, gravity, gravityModel] + gravityTargets

    # Deplete each propellant body by driving a constant negative mass-rate into its mass-state
    # derivative. The dry hub is not registered, so it does not deplete.
    massRateMsgs = []
    for name, rate in bodyMassFlowRates().items():
        payload = messaging.SCMassPropsMsgPayload()
        payload.massSC = rate  # [kg/s]
        rateMsg = messaging.SCMassPropsMsg().write(payload)
        scene.getBody(name).derivativeMassPropertiesInMsg.subscribeTo(rateMsg)
        massRateMsgs.append(rateMsg)

    handles += massRateMsgs
    if useThruster:
        maxThrust, _ = thrusterSpec()
        thrustMsg = messaging.SingleActuatorMsg().write(
            messaging.SingleActuatorMsgPayload(input=maxThrust))  # [N] constant main-engine thrust
        scene.getSingleActuator("mainEngine").actuatorInMsg.subscribeTo(thrustMsg)
        handles.append(thrustMsg)

    if compensateVariableInertia:
        # Emulate the variable-mass rotational reaction the depletion model omits (see the class).
        compensator = VariableInertiaCompensator(hub, scene, bodyMassFlowRates())
        scene.AddModelToDynamicsTask(compensator)
        compTorqueActuator = scene.addTorqueActuator("varMassComp", hub.getOrigin())
        compTorqueActuator.torqueInMsg.subscribeTo(compensator.torqueOutMsg)
        handles += [compensator, compTorqueActuator]

    # Reproduce the BSM spherical-pendulum damping torque on the ball joint (see the class): the
    # pendulum carries its physical damping on both engines, and no native ball-joint damping value
    # matches the -d (omega_rel x l) form, so it is supplied as an explicit torque.
    slosh = sloshParameters()
    pendDamp = PendulumDampingCompensator(scene, slosh["pendD"], slosh["pendLength"])
    scene.AddModelToDynamicsTask(pendDamp)
    pendDampActuator = scene.addTorqueActuator("pendDamp", scene.getBody("pendulum").getOrigin())
    pendDampActuator.torqueInMsg.subscribeTo(pendDamp.torqueOutMsg)
    handles += [pendDamp, pendDampActuator]

    # The system center of mass, extracted directly from the MuJoCo tree, is the analogue of the
    # BSM hub state's r_CN_N/v_CN_N so the orbit can be compared across engines.
    systemCoM = MJSystemCoM.MJSystemCoM()
    systemCoM.ModelTag = "systemCoM"
    systemCoM.scene = scene
    scSim.AddModelToTask("dynTask", systemCoM, 1)
    handles.append(systemCoM)

    recorders = {}
    if record:
        recorders["state"] = hub.getOrigin().stateOutMsg.recorder(macros.sec2nano(dt))
        recorders["com"] = systemCoM.comStatesOutMsg.recorder(macros.sec2nano(dt))
        recorders["mass"] = {name: scene.getBody(name).massPropertiesOutMsg.recorder(
            macros.sec2nano(dt)) for name in ("tank", "sloshX", "sloshY", "sloshZ", "pendulum")}
        recorders["slosh"] = {name: scene.getBody(name).getScalarJoint(name).stateOutMsg.recorder(
            macros.sec2nano(dt)) for name in ("sloshX", "sloshY", "sloshZ")}
        scSim.AddModelToTask("dynTask", recorders["state"], 0)
        scSim.AddModelToTask("dynTask", recorders["com"], 0)
        for rec in recorders["mass"].values():
            scSim.AddModelToTask("dynTask", rec, 0)
        for rec in recorders["slosh"].values():
            scSim.AddModelToTask("dynTask", rec, 0)

    scSim.InitializeSimulation()

    # Free-body initial conditions are set after initialization, matching the BSM start exactly.
    hub.setPosition(list(initialState["r_BN_N"]))  # [m] hub-origin inertial position
    hub.setVelocity(list(initialState["v_BN_N"]))  # [m/s]
    hub.setAttitude(list(initialState["sigma_BN"]))
    hub.setAttitudeRate(list(initialState["omega_BN_B"]))

    # Seed the residual slosh. The spring-mass-damper particles take their initial displacement
    # through the slide-joint position setters; the lateral pair carries residual slosh and the
    # axial one starts at the settled offset -a/omega1^2, exactly as on the BSM side.
    slosh = sloshParameters()
    smdInit = {"sloshX": SMD_LATERAL_RHO0, "sloshY": -SMD_LATERAL_RHO0,
               "sloshZ": slosh["axialRho0"]}
    for name, rho0 in smdInit.items():
        scene.getBody(name).getScalarJoint(name).setPosition(rho0)  # [m]

    # The pendulum's residual slosh is an initial angular rate (phiDot, thetaDot). MJBallJoint has
    # no rate setter, so write it into the bulk MuJoCo velocity state directly. With the P0 rest
    # frame pHat_01=-z, pHat_02=+x, pHat_03=-y, a body-3-2 (phi about pHat_03, then theta about the
    # rotated pHat_02) rate of phiDot=thetaDot=PEND_RATE0 at zero angles maps to a joint-frame
    # angular velocity of (thetaDot, -phiDot, 0) = (PEND_RATE0, -PEND_RATE0, 0); the third
    # (rod-spin) component is zero to match the 2-DOF pendulum.
    ball = scene.getBody("pendulum").getBallJoint()
    qvelState = scene.dynManager.getStateObject("mujocoQvel")
    qvel = np.array(qvelState.getState()).flatten()
    qvAdr = ball.getQvelAdr()
    qvel[qvAdr:qvAdr+3] = [PEND_RATE0, -PEND_RATE0, 0.0]  # [rad/s]
    qvelState.setState(qvel.reshape(-1, 1))

    return scSim, recorders, handles


def relativePrincipalAngle(sigmaA, sigmaB):
    """Per-sample principal rotation angle between two MRP attitude histories.

    Args:
        sigmaA (numpy.ndarray): first MRP history, shape ``(N, 3)``
        sigmaB (numpy.ndarray): second MRP history, shape ``(N, 3)``

    Returns:
        numpy.ndarray: principal angle per sample [rad].
    """
    nSamples = min(len(sigmaA), len(sigmaB))
    angle = np.empty(nSamples)
    for i in range(nSamples):
        dcmRel = rbk.MRP2C(sigmaA[i]).dot(rbk.MRP2C(sigmaB[i]).T)
        # 4*atan(|sigma_rel|) is well conditioned down to machine precision, unlike
        # arccos((trace-1)/2) which collapses to zero below ~1e-8 rad.
        angle[i] = 4.0*np.arctan(np.linalg.norm(rbk.C2MRP(dcmRel)))
    return angle


def gravityGradientRateEstimate(mu):
    """Standard gravity-gradient angular-acceleration scale ``3 (mu/r^3) dI/I`` [rad/s^2].

    This is the order-of-magnitude rate at which the MuJoCo hub attitude (per-body gravity, which
    carries a gravity-gradient torque) departs from the BSM hub attitude (single-point gravity at
    the system center of mass, which does not). The inertia spread is dominated by the tank and
    propellant mounted an arm ``TANK_R_TB_B`` off the hub center of mass.

    Args:
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        float: angular-acceleration scale [rad/s^2].
    """
    arm = np.linalg.norm(TANK_R_TB_B)  # [m] tank offset from the hub center of mass
    deltaInertia = PROPELLANT_MASS*arm**2  # [kg*m^2] parallel-axis spread from the offset propellant
    meanInertia = np.mean(HUB_INERTIA)  # [kg*m^2]
    return 3.0*(mu/ORBIT_A**3)*deltaInertia/meanInertia  # [rad/s^2]


def pullBSM(recorders, mu):
    """Collect the BSM reference histories into a plain dictionary of numpy arrays.

    Args:
        recorders (dict): the recorder dict returned by :func:`buildBSM`
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        dict: time, hub state, orbit, tank mass and slosh-state histories.
    """
    stateRec = recorders["state"]
    tankRec = recorders["tank"]
    sloshRec = recorders["slosh"]

    rBN = np.array(stateRec.r_BN_N)  # [m]
    vBN = np.array(stateRec.v_BN_N)  # [m/s]
    rMag = np.linalg.norm(rBN, axis=1)  # [m]
    vMag = np.linalg.norm(vBN, axis=1)  # [m/s]
    semiMajorAxis = 1.0/(2.0/rMag - vMag**2/mu)  # [m] vis-viva

    fuelMass = np.array(tankRec.fuelMass)  # [kg] non-sloshing mass m0 history
    # The tank depletes m0 and every slosh mass proportionally to their current mass, so all mass
    # ratios stay fixed and the total propellant tracks m0 exactly. Total system mass = dry hub +
    # non-sloshing mass + the (proportionally depleting) slosh masses.
    bulkMass = sloshParameters()["bulkMass"]  # [kg] initial m0
    totalMass = HUB_MASS + fuelMass*(PROPELLANT_MASS/bulkMass)  # [kg]

    return {
        "t": np.array(stateRec.times())*macros.NANO2SEC,
        "sigma_BN": np.array(stateRec.sigma_BN),
        "omega_BN_B": np.array(stateRec.omega_BN_B),
        "r_BN_N": rBN,
        "v_BN_N": vBN,
        "r_CN_N": np.array(stateRec.r_CN_N),
        "v_CN_N": np.array(stateRec.v_CN_N),
        "semiMajorAxis": semiMajorAxis,
        "fuelMass": fuelMass,
        "totalMass": totalMass,
        "rho": np.column_stack([sloshRec.rho1, sloshRec.rho2, sloshRec.rho3]),
        "phi": np.array(sloshRec.phi),
        "theta": np.array(sloshRec.theta),
    }


def pullMujoco(recorders, mu):
    """Collect the MuJoCo histories into a dictionary matching :func:`pullBSM`.

    The hub-origin frame is read from the free-joint site, the system center of mass from
    :ref:`MJSystemCoM`, and the propellant masses and slosh displacements from the per-body mass
    and slide-joint recorders. The total system mass is summed from the dry hub and the recorded
    depleting propellant-body masses, the direct analogue of the BSM ``totalMass``.

    Args:
        recorders (dict): the recorder dict returned by :func:`buildMujoco`
        mu (float): gravitational parameter [m^3/s^2]

    Returns:
        dict: time, hub state, system-center-of-mass orbit, mass and slosh histories.
    """
    stateRec = recorders["state"]
    comRec = recorders["com"]

    rCN = np.array(comRec.r_CN_N)  # [m] system center of mass, MuJoCo per-body gravity
    vCN = np.array(comRec.v_CN_N)  # [m/s]
    rMag = np.linalg.norm(rCN, axis=1)  # [m]
    vMag = np.linalg.norm(vCN, axis=1)  # [m/s]
    semiMajorAxis = 1.0/(2.0/rMag - vMag**2/mu)  # [m] vis-viva

    tankMass = np.array(recorders["mass"]["tank"].massSC)  # [kg] non-sloshing mass m0
    totalMass = HUB_MASS + tankMass.copy()
    for name in ("sloshX", "sloshY", "sloshZ", "pendulum"):
        totalMass = totalMass + np.array(recorders["mass"][name].massSC)

    return {
        "t": np.array(stateRec.times())*macros.NANO2SEC,
        "sigma_BN": np.array(stateRec.sigma_BN),
        "omega_BN_B": np.array(stateRec.omega_BN_B),
        "r_BN_N": np.array(stateRec.r_BN_N),
        "v_BN_N": np.array(stateRec.v_BN_N),
        "r_CN_N": rCN,
        "v_CN_N": vCN,
        "semiMajorAxis": semiMajorAxis,
        "fuelMass": tankMass,
        "totalMass": totalMass,
        "rho": np.column_stack([np.array(recorders["slosh"][name].state)
                                for name in ("sloshX", "sloshY", "sloshZ")]),
    }


def run(showPlots=False, saveJson=False, simDuration=SIM_DURATION, useThruster=True,
        inOrbit=True, compensateVariableInertia=None, saveReference=False):
    """Main function, see scenario description.

    Args:
        showPlots (bool, optional): if True, plot and show the simulation results.
            Defaults to False.
        saveJson (bool, optional): if True, write scalar comparison metrics to
            ``results/scenarioCompareVariableMass.json``. Defaults to False.
        simDuration (float, optional): burn/comparison window [s]. Defaults to ``SIM_DURATION``.
        useThruster (bool, optional): if True (default) deplete the tank with the firing main
            engine. If False, use an equivalent prescribed leak rate with no thrust force.
        inOrbit (bool, optional): if True (default) fly under Earth gravity (engines separate by the
            gravity gradient); if False, a deep-space burn from rest that isolates the variable-mass
            reaction torque.
        compensateVariableInertia (bool, optional): if True, additionally run the MuJoCo side with
            the :class:`VariableInertiaCompensator` and report/plot the reduced difference. Defaults
            to None, which enables it in deep space (where the effect is isolated) and disables it in
            orbit (where the gravity gradient dominates).
        saveReference (bool, optional): if True, write the BSM ground-truth trajectory to
            ``results/scenarioCompareVariableMass_reference.npz``. Defaults to False.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    dt = timeStep()  # [s]
    maxThrust, steadyIsp = thrusterSpec()
    if compensateVariableInertia is None:
        compensateVariableInertia = not inOrbit

    bsmSim, bsmRec, _ = buildBSM(dt, True, useThruster, inOrbit)
    mu = earthMu()  # [m^3/s^2]
    bsmSim.ConfigureStopTime(macros.sec2nano(simDuration))
    bsmSim.ExecuteSimulation()
    bsm = pullBSM(bsmRec, mu)

    metrics = {
        "scenario": fileName,
        "timeStep": dt,
        "simDuration": simDuration,
        "useThruster": useThruster,
        "inOrbit": inOrbit,
        "thruster": THRUSTER_TYPE,
        "maxThrust": maxThrust,
        "steadyIsp": steadyIsp,
        "nominalMassFlow": nominalMassFlow(maxThrust, steadyIsp),
        "propellantDepletedFraction": float(1.0 - bsm["fuelMass"][-1]/bsm["fuelMass"][0]),
        "wetMassChangeFraction": float(1.0 - bsm["totalMass"][-1]/bsm["totalMass"][0]),
        "deltaV": float(np.linalg.norm(bsm["v_BN_N"][-1]) - np.linalg.norm(bsm["v_BN_N"][0])),
    }
    if inOrbit:
        metrics["semiMajorAxisRise"] = float(
            bsm["semiMajorAxis"][-1] - bsm["semiMajorAxis"][0])

    mj = None
    mjComp = None
    if couldImportMujoco:
        # Start the MuJoCo run from the BSM hub-origin state at t=0 so both engines begin from the
        # identical state (feeding r_CN_NInit to the hub origin would offset them by the tank arm).
        initialState = {"r_BN_N": bsm["r_BN_N"][0], "v_BN_N": bsm["v_BN_N"][0],
                        "sigma_BN": bsm["sigma_BN"][0], "omega_BN_B": bsm["omega_BN_B"][0]}
        mjSim, mjRec, _ = buildMujoco(dt, True, initialState, useThruster, inOrbit)
        mjSim.ConfigureStopTime(macros.sec2nano(simDuration))
        mjSim.ExecuteSimulation()
        mj = pullMujoco(mjRec, mu)

        nSamples = min(len(bsm["t"]), len(mj["t"]))
        attError = relativePrincipalAngle(bsm["sigma_BN"], mj["sigma_BN"])  # [rad]
        comError = np.linalg.norm(bsm["r_CN_N"][:nSamples] - mj["r_CN_N"][:nSamples], axis=1)  # [m]
        rateError = np.linalg.norm(
            bsm["omega_BN_B"][:nSamples] - mj["omega_BN_B"][:nSamples], axis=1)  # [rad/s]
        metrics["attitudeErrorMax"] = float(np.max(attError))
        metrics["comPositionErrorMax"] = float(np.max(comError))
        metrics["rateErrorMax"] = float(np.max(rateError))
        metrics["totalMassErrorMax"] = float(
            np.max(np.abs(bsm["totalMass"][:nSamples] - mj["totalMass"][:nSamples])))
        if inOrbit:
            # In orbit the cross-engine attitude/orbit difference is the gravity-gradient torque
            # MuJoCo's per-body gravity carries and the BSM single-point gravity omits; this is the
            # order-of-magnitude scale to compare it against.
            metrics["gravityGradientRateEstimate"] = float(gravityGradientRateEstimate(mu))

        if compensateVariableInertia:
            # Re-run the MuJoCo side with the variable-inertia reaction-torque compensator and
            # report how much of the cross-engine attitude difference it removes.
            mjCompSim, mjCompRec, _ = buildMujoco(
                dt, True, initialState, useThruster, inOrbit, compensateVariableInertia=True)
            mjCompSim.ConfigureStopTime(macros.sec2nano(simDuration))
            mjCompSim.ExecuteSimulation()
            mjComp = pullMujoco(mjCompRec, mu)
            metrics["attitudeErrorCompensatedMax"] = float(
                np.max(relativePrincipalAngle(bsm["sigma_BN"], mjComp["sigma_BN"])))

    if saveReference:
        os.makedirs(resultsPath, exist_ok=True)
        np.savez(os.path.join(resultsPath, fileName+"_reference.npz"), **bsm)

    if saveJson:
        import json
        os.makedirs(resultsPath, exist_ok=True)
        with open(os.path.join(resultsPath, fileName+".json"), "w") as f:
            json.dump(metrics, f, indent=2)

    figureList = plotResults(bsm, mj, inOrbit, mjComp)

    # Always append the rigid-body proof figure (its own self-contained deep-space comparison) so
    # the isolated variable-mass-reaction demonstration is regenerated for the documentation.
    if couldImportMujoco:
        figureList.update(variableInertiaProof(showPlots=False, saveJson=saveJson,
                                                simDuration=simDuration))

    _comparePlots.finalizeFigures(figureList)
    if showPlots:
        plt.show()
    plt.close("all")

    return figureList


def plotResults(bsm, mj=None, inOrbit=True, mjComp=None):
    """Build the scenario figures.

    The BSM curves are drawn as thick translucent underlays and the MuJoCo curves as thin lines on
    top, so the two engines stay distinguishable where they overlap. A dedicated figure shows the
    cross-engine attitude difference: in orbit it is dominated by the gravity-gradient torque the
    two gravity models treat differently, and in deep space by the variable-inertia reaction torque
    (see the docstring), with the :class:`VariableInertiaCompensator` overlay when supplied.

    Args:
        bsm (dict): BSM reference histories from :func:`pullBSM`
        mj (dict, optional): MuJoCo histories from :func:`pullMujoco`, or None when MuJoCo is
            unavailable
        inOrbit (bool, optional): whether the run was on orbit. Selects the first figure between
            the orbit-raise plot (in orbit) and the burn-speed plot (deep space).
        mjComp (dict, optional): MuJoCo histories with the variable-inertia compensator enabled,
            overlaid on the attitude-difference figure to show the reduction. Defaults to None.

    Returns:
        dict: mapping from figure name to matplotlib figure.
    """
    t = bsm["t"]
    figureList = {}
    mjT = mj["t"] if mj is not None else None
    nSamples = min(len(t), len(mjT)) if mj is not None else len(t)

    # Orbit-raise (in orbit) or accumulated burn speed (deep space): both engines with the
    # difference below.
    if inOrbit:
        yBSM = (bsm["semiMajorAxis"] - bsm["semiMajorAxis"][0])*1e-3  # [km]
        yMj = None if mj is None else (mj["semiMajorAxis"] - mj["semiMajorAxis"][0])*1e-3
        name, fig = _comparePlots.overlayWithDifference(
            fileName+"_orbit", t, yBSM, yMj, "Semi-major axis rise [km]")
    else:
        yBSM = np.linalg.norm(bsm["v_CN_N"], axis=1) - np.linalg.norm(bsm["v_CN_N"][0])  # [m/s]
        yMj = (None if mj is None
               else np.linalg.norm(mj["v_CN_N"], axis=1) - np.linalg.norm(mj["v_CN_N"][0]))
        name, fig = _comparePlots.overlayWithDifference(
            fileName+"_orbit", t, yBSM, yMj, "Accumulated burn speed [m/s]")
    figureList[name] = fig

    # Total system mass (dry hub plus depleting propellant) with the cross-engine difference below.
    name, fig = _comparePlots.overlayWithDifference(
        fileName+"_fuelMass", t, bsm["totalMass"], None if mj is None else mj["totalMass"],
        "Total system mass [kg]", diffUnit="g", diffScale=1e3)
    figureList[name] = fig

    # Hub body rate: one column per body axis, both engines over the difference.
    name, fig = _comparePlots.componentComparison(
        fileName+"_rate", t, bsm["omega_BN_B"], None if mj is None else mj["omega_BN_B"],
        r"$\omega_{BN}$", "mrad/s", scale=1e3)
    figureList[name] = fig

    # Translational slosh: one column per particle (x/y/z spring-mass-damper), both engines over
    # the difference.
    name, fig = _comparePlots.componentComparison(
        fileName+"_slosh", t, bsm["rho"], None if mj is None else mj["rho"],
        r"$\rho$", "mm", scale=1e3)
    figureList[name] = fig

    if mj is not None:
        # Cross-engine attitude difference (log) with the compensator overlay, and the system
        # center-of-mass difference below.
        attError = relativePrincipalAngle(bsm["sigma_BN"], mj["sigma_BN"])  # [rad]
        comError = np.linalg.norm(
            bsm["r_CN_N"][:nSamples] - mj["r_CN_N"][:nSamples], axis=1)  # [m]

        figFig, (axAtt, axCoM) = plt.subplots(2, 1, sharex=True, figsize=(7.0, 4.4),
                                              layout="constrained")
        axAtt.semilogy(t[:nSamples], np.maximum(attError[:nSamples], 1e-12), color=COLOR_MUJOCO,
                       label="MuJoCo (uncompensated)")
        if mjComp is not None:
            nComp = min(nSamples, len(mjComp["t"]))
            attComp = relativePrincipalAngle(bsm["sigma_BN"], mjComp["sigma_BN"])
            axAtt.semilogy(t[:nComp], np.maximum(attComp[:nComp], 1e-12), color=COLOR_BSM,
                           label="MuJoCo + variable-inertia compensator")
            axAtt.legend(loc="best", fontsize=8)
        axAtt.set_ylabel("Hub attitude\ndifference [rad]")
        axAtt.set_title("BSM vs MuJoCo: " + ("gravity-gradient modeling difference" if inOrbit
                        else "variable-mass reaction torque (deep space)"), fontsize=9)
        axCoM.plot(t[:nSamples], comError, color=COLOR_MUJOCO)
        axCoM.set_xlabel("Time [s]")
        axCoM.set_ylabel("System CoM\ndifference [m]")
        figureList[fileName+"_attError"] = figFig

    return figureList


def variableInertiaProof(showPlots=False, saveJson=False, simDuration=SIM_DURATION):
    r"""Isolate the variable-mass reaction torque by comparing the engines on a rigid vehicle.

    Strips the scenario to its rigid core -- slosh masses made negligible, so the tank carries all
    the propellant -- and runs the deep-space burn twice: MuJoCo without compensation, and MuJoCo
    with the :class:`VariableInertiaCompensator` torque. The compensator collapses the cross-engine
    attitude difference from :math:`\sim 10^{-2}` to :math:`\sim 3\times 10^{-6}` rad (a
    three-thousand-fold drop, 99.97% of the difference), proving the reaction torque is the dominant
    rigid-body modeling difference. The remainder is dt-independent -- a genuine higher-order term
    the single leading torque omits, not integrator truncation.

    Args:
        showPlots (bool, optional): if True, plot and show the proof figure. Defaults to False.
        saveJson (bool, optional): if True, write the proof metrics to
            ``results/scenarioCompareVariableMass_proof.json``. Defaults to False.
        simDuration (float, optional): burn window [s]. Defaults to ``SIM_DURATION``.

    Returns:
        dict: mapping from figure name to matplotlib figure (empty if MuJoCo is unavailable).
    """
    global SLOSH_MASS_FRACTION, SMD_MASS, SMD_LATERAL_RHO0, PEND_RATE0
    saved = (SLOSH_MASS_FRACTION, SMD_MASS, SMD_LATERAL_RHO0, PEND_RATE0)
    # Rigid limit: negligible (not exactly zero, which would make the pendulum effector singular)
    # slosh masses and no residual slosh excitation, so the tank carries all the propellant.
    SLOSH_MASS_FRACTION = 1.0e-6/PROPELLANT_MASS  # pendulum mass ~ 1e-6 kg
    SMD_MASS = 1.0e-9  # [kg]
    SMD_LATERAL_RHO0 = 0.0  # [m]
    PEND_RATE0 = 0.0  # [rad/s]
    try:
        dt = timeStep()  # [s]
        mu = earthMu()  # [m^3/s^2]

        def crossEngineAttitude(compensate):
            bsmSim, bsmRec, _ = buildBSM(dt, True, useThruster=True, inOrbit=False)
            bsmSim.ConfigureStopTime(macros.sec2nano(simDuration))
            bsmSim.ExecuteSimulation()
            bsm = pullBSM(bsmRec, mu)
            initialState = {"r_BN_N": bsm["r_BN_N"][0], "v_BN_N": bsm["v_BN_N"][0],
                            "sigma_BN": bsm["sigma_BN"][0], "omega_BN_B": bsm["omega_BN_B"][0]}
            mjSim, mjRec, _ = buildMujoco(dt, True, initialState, useThruster=True, inOrbit=False,
                                          compensateVariableInertia=compensate)
            mjSim.ConfigureStopTime(macros.sec2nano(simDuration))
            mjSim.ExecuteSimulation()
            mj = pullMujoco(mjRec, mu)
            return bsm["t"], relativePrincipalAngle(bsm["sigma_BN"], mj["sigma_BN"])

        figureList = {}
        if not couldImportMujoco:
            return figureList

        t, attRaw = crossEngineAttitude(False)
        _, attComp = crossEngineAttitude(True)
        metrics = {
            "scenario": fileName + "_proof",
            "timeStep": dt,
            "rigidAttitudeDiffMax": float(np.max(attRaw)),
            "rigidAttitudeDiffCompensatedMax": float(np.max(attComp)),
            "reductionFactor": float(np.max(attRaw)/np.max(attComp)),
        }

        if saveJson:
            import json
            os.makedirs(resultsPath, exist_ok=True)
            with open(os.path.join(resultsPath, fileName+"_proof.json"), "w") as f:
                json.dump(metrics, f, indent=2)

        figureList[fileName+"_proof"], ax = plt.subplots(layout="constrained")
        nRaw = min(len(t), len(attRaw))
        nComp = min(len(t), len(attComp))
        ax.semilogy(t[:nRaw], np.maximum(attRaw[:nRaw], 1e-14), color=COLOR_MUJOCO,
                    label="MuJoCo (uncompensated)")
        ax.semilogy(t[:nComp], np.maximum(attComp[:nComp], 1e-14), color=COLOR_BSM,
                    label=r"MuJoCo + $2\dot m\,c\times(\omega\times c)$ compensator")
        ax.set_xlabel("Time [s]")
        ax.set_ylabel("Rigid-body cross-engine\nattitude difference [rad]")
        ax.set_title(f"Variable-mass reaction torque isolated: "
                     f"{metrics['reductionFactor']:.0f}x reduction", fontsize=9)
        ax.legend(loc="best")

        if showPlots:
            plt.show()
        return figureList
    finally:
        SLOSH_MASS_FRACTION, SMD_MASS, SMD_LATERAL_RHO0, PEND_RATE0 = saved


if __name__ == "__main__":
    run(showPlots=True, saveReference=True)
    variableInertiaProof(showPlots=True)
