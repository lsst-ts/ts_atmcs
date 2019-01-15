# This file is part of ts_ATMCSSimulator.
#
# Developed for the LSST Data Management System.
# This product includes software developed by the LSST Project
# (https://www.lsst.org).
# See the COPYRIGHT file at the top-level directory of this distribution
# for details of code ownership.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

__all__ = ["slew", "SLEW_FUDGE"]

import math
import sys

from . import path


SLEW_FUDGE = 1.05
"""Fudge factor to avoid borderline cases; should be a bit larger than 1"""


def slew(t0, pA, vA, pB, vB, vmax, amax):
    """Compute a trapezoidal slew from A to B.

    A and B are paths of constant velocity.

    Parameters
    ----------
    t0 : `float`
        Start time of slew.
    pA : `float`
        Position of A at time t0 (deg)
    vA : `float`
        Velocity of A at time t0 (deg/sec)
    pB : `float`
        Position of B at time t0 (deg)
    vB : `float`
        Velocity of B at time t0 (deg/sec)
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
    * |vA| > vmax*SLEW_FUDGE
    * |vB| > vmax/SLEW_FUDGE

    RuntimeError if internal consistency checks fail.

    Notes
    -----
    Dies if vmax or amax are so small as to cause under- or over-flow.

    * Details *

    The magic number `SLEW_FUDGE` is used to avoid borderline cases.
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
        raise ValueError(f"vmax={vmax} < 0")
    if amax <= 0.0:
        raise ValueError(f"amax={amax} < 0")

    # check velocities; errors are: |vB| SLEW_FUDGE > vmax,
    # |vA| SLEW_FUDGE > vmax (can lead to dt1 < 0 for type 2 slews)
    if abs(vA) > vmax*SLEW_FUDGE:
        raise ValueError(f"Telescope is moving too fast (|{vA:0.4f}| > {SLEW_FUDGE} * {vmax}).")
    if abs(vB)*SLEW_FUDGE > vmax:
        raise ValueError(f"Target is moving too fast (|{vB:0.4f}| * {SLEW_FUDGE} > {vmax}; "
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
        return path.Path(path.TPVAJ(t0=t0, p0=pA, v0=vA), kind=path.Kind.Slewing)

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
    elif abs(vA) * SLEW_FUDGE < vmax and 0 <= (amax*abs(rBAi) - half_vBAsq):
        a1 = sign_rBAi * amax

    # a type 3 slew only exists if amax is small enough
    elif amax*abs(rBAi)*SLEW_FUDGE <= half_vBAsq:
        a1 = -sign_rBAi * amax

    # a type 4 slew requires reducing accel. to obtain a solution
    else:
        # since the equation for a1 is sure to give |a1| < amax
        # (otherwise slew would have been type 3)
        # the equation is guranteed to not overflow
        a1 = -half_vBAsq / (SLEW_FUDGE*rBAi)
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
        # Note that |vB| < vmax / SLEW_FUDGE (as tested far above),
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
    tpvajs = []
    if dt1 > 0:
        tpvajs.append(path.TPVAJ(t0=0, p0=pA, v0=vA, a0=a1))
        t20 = dt1
        p20, v20 = tpvajs[-1].pva(t20)[0:2]
    else:
        t20 = 0
        p20 = pA
        v20 = vA
    if dt2 > 0:
        tpvajs.append(path.TPVAJ(t0=t20, p0=p20, v0=v20))
        t30 = t20 + dt2
        p30, v30 = tpvajs[-1].pva(t30)[0:2]
    else:
        t30 = t20
        p30 = p20
        v30 = v20
    if dt3 > 0:
        tpvajs.append(path.TPVAJ(t0=t30, p0=p30, v0=v30, a0=a3))
        t40 = t30 + dt3
        p40, v40 = tpvajs[-1].pva(t40)[0:2]
    else:
        t40 = t30
        p40 = p30
        v40 = v30
    if tpvajs[-1].a0 != 0:
        # if the last TPVAJ has non-zero acceleraton then append
        # a segment with zero acceleration
        tpvajs.append(path.TPVAJ(t0=t40, p0=p40, v0=v40))
    for tpvaj in tpvajs:
        tpvaj.t0 += t0

    return path.Path(*tpvajs, kind=path.Kind.Slewing)
