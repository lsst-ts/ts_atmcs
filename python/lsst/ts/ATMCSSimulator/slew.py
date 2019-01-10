# This file comes from

__all__ = ["slew"]

import math
import sys

from . import path


FUDGE = 1.05
"""Fudge factor to avoid borderline cases; should be a bit larger than 1"""


def slew(pA, pB, vA, vB, t0, vmax, amax):
    """Compute a trapezoidal slew from A to B.

    A and B are paths of constant velocity.

    Parameters
    ----------
    pA : `float`
        Position of A at time t0 (deg)
    pB : `float`
        Position of B at time t0 (deg)
    vA : `float`
        Velocity of A at time t0 (deg/sec)
    vB : `float`
        Velocity of B at time t0 (deg/sec)
    t0 : `float`
        Start time of slew.
    vmax : `float`
        Maximum allowed velocity (deg/sec)
    amax : `float`
        Maximum allowed acceleration (deg/sec^2)

    Returns
    -------
    path : `Path`
        A path with 1-4 segments, ending with a segment of zero acceleration
        that matches B.

    Raises
    ------
    ValueError if any of the following are true:

    * vmax <= 0
    * amax <= 0
    * |vA| > vmax*FUDGE
    * |vB| > vmax/FUDGE

    RuntimeError if internal consistency checks fail.

    Notes
    -----
    Dies if vmax or amax are so small as to cause under- or over-flow.

    * Details *

    The magic number `FUDGE` is used to avoid borderline cases.
    It should be a number a bit bigger than one.

    How it works:
    The slew begins by tracking object A, and ends by tracking object B.
    Both objects are assumed to be moving at constant velocity.

    A trapezoidal slew has three segments, two of constant acceleration
    separated by a constant velocity segment. It is called "trapezoidal"
    because that is the shape of the v vs. t curve.

    Here are the initial velocity and constant acceleration for each segment,
    and the duration of that segment, in the notation of this subroutine
    (each of the first 3 segments is present only if its duration is nonzero,
    and the last is absent if the final segment already has zero acceleration):

        segment   v   a   duration
           1      vA  a1  dt1
           2      vP  0   dt2
           3      vP  a2  dt3
           4      vB  0   unlimited
    """
    if vmax <= 0.0:
        raise ValueError("vmax=%s < 0" % (vmax,))
    if amax <= 0.0:
        raise ValueError("amax=%s < 0" % (amax,))

    # check velocities; errors are: |vB| FUDGE > vmax,
    # |vA| FUDGE > vmax (can lead to dt1 < 0 for type 2 slews)
    if abs(vA) > vmax*FUDGE:
        raise ValueError(f"Telescope is moving too fast (|{vA:0.4f}| > {FUDGE} * {vmax}).")
    if abs(vB)*FUDGE > vmax:
        raise ValueError(f"Target is moving too fast (|{vB:0.4f}| * {FUDGE} > {vmax}; "
                         "telescope cannot acquire it.")

    # compute vBA, half_vBAsq, sign_rBAi and sign_vBA
    # and handle null slews (rBAi and vBA both zero)
    rBAi = pB - pA
    vBA = vB - vA
    half_vBAsq = 0.5*vBA*vBA
    if rBAi != 0.0 and vBA != 0.0:
        sign_rBAi = math.copysign(1.0, rBAi)
        sign_vBA = math.copysign(1.0, vBA)
    elif rBAi != 0.0:
        sign_rBAi = math.copysign(1.0, rBAi)
        sign_vBA = sign_rBAi
    elif vBA != 0.0:
        sign_vBA = math.copysign(1.0, vBA)
        sign_rBAi = sign_vBA
    else:
        return path.Path(path.PVAT(p0=pA, v0=vA, a=0, t0=t0))

    # compute a1 and a3
    # if sign(rBAi) = sign(vBA), slew is type 1
    # a solution is sure because dt3 has no upper limit over range of soln
    if sign_rBAi == sign_vBA:
        a1 = sign_rBAi * amax

    # else sign(rBAi) = -sign(vBA) so we use type 2, 3 or 4 slew...

    # a type 2 slew has a maximum dt2 dependent on initial conditions;
    # the biggest dt2 occurs at largest |a|, |a| = amax,
    # and smallest |vPB|, |vPB| = |vBA|
    # so test at that point to see if solutions exist with dt2 > 0
    elif abs(vA) * FUDGE < vmax and 0 <= (amax*abs(rBAi) - half_vBAsq):
        a1 = sign_rBAi * amax

    # a type 3 slew only exists if amax is small enough
    elif amax*abs(rBAi)*FUDGE <= half_vBAsq:
        a1 = -sign_rBAi * amax

    # a type 4 slew requires reducing accel. to obtain a solution
    else:
        # since the equation for a1 is sure to give |a1| < amax
        # (otherwise slew would have been type 3)
        # the equation is guranteed to not overflow
        a1 = -half_vBAsq / (FUDGE*rBAi)
    a3 = -a1

    # make sure velocity / acceleration divisions will not overflow;
    # this is especially important for slew type 4 because acceleration
    # gets reduced, but could also catch stupid amax or vmax inputs
    max_vdiff = vmax + abs(vA) + abs(vB)
    if max_vdiff >= min(abs(a1), 1.0)*sys.float_info.max:
        raise RuntimeError('Computed slew time is ridiculous')

    # compute dt2 and vP
    # first assume that dt2 = 0 and compute vP;
    # if resulting vP is too big, reduce it to maximum allowed
    # and compute corresponding increased dt2
    dt2 = 0
    vPB_temp = (0.5*a1*dt2)**2 + half_vBAsq + a1*rBAi
    if vPB_temp < 0.0:
        raise RuntimeError('Bug! Tried to compute square root of negative value')
    vPB = math.copysign(math.sqrt(vPB_temp), a1) - 0.5*a1*dt2
    vP = vPB + vB
    if abs(vP) > vmax:
        # |vP| is too big, and so must be reduced.
        # Note that |vB| < vmax / FUDGE (as tested far above),
        # so |vP| is guaranteed to be reducible to vmax without causing
        # vPB to approach zero (much less change sign).
        # The division velocity / acceleration was proved safe above.
        # Thus dt2 may be computed without overflow.
        vP = math.copysign(vmax, vP)
        vPB = vP - vB
        dt2 = (rBAi + ((half_vBAsq - vPB*vPB)/a1)) / vPB

    # compute dt1 and dt3
    # the following divisions were proved safe from overflow above,
    # just after computing a1 and a3
    vPA = vPB + vBA
    dt1 = vPA/a1
    dt3 = -vPB/a3

    # sanity checks
    if dt1 < 0.0 or dt2 < 0.0 or dt3 < 0.0:
        raise RuntimeError('Bug! Computed negative duration for one or more segments')
    if abs(vP) > vmax:
        raise RuntimeError('Bug! Computed velocity greater than max velocity')
    if abs(a1) > amax:
        raise RuntimeError('Bug! Computed acceleration greater than max acceleration')

    # t0 is typically large enough that small time changes
    # have poor accuracy, so to improve accuracy
    # compute the path segments using t0 = 0
    # then offset all the times before returning the path
    pvats = []
    if dt1 > 0:
        pvats.append(path.PVAT(p0=pA, v0=vA, a=a1, t0=0))
        t20 = dt1
        p20, v20 = pvats[-1].pva(t20)[0:2]
    else:
        t20 = 0
        p20 = pA
        v20 = vA
    if dt2 > 0:
        pvats.append(path.PVAT(p0=p20, v0=v20, a=0, t0=t20))
        t30 = t20 + dt2
        p30, v30 = pvats[-1].pva(t30)[0:2]
    else:
        t30 = t20
        p30 = p20
        v30 = v20
    if dt3 > 0:
        pvats.append(path.PVAT(p0=p30, v0=v30, a=a3, t0=t30))
        t40 = t30 + dt3
        p40, v40 = pvats[-1].pva(t40)[0:2]
    else:
        t40 = t30
        p40 = p30
        v40 = v30
    if pvats[-1].a != 0:
        # if the last PVAT has non-zero acceleraton then append
        # a segment with zero acceleration
        pvats.append(path.PVAT(p0=p40, v0=v40, a=0, t0=t40))
    for pvat in pvats:
        pvat.t0 += t0

    return path.Path(*pvats)
