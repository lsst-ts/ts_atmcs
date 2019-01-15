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

import math
import sys

__all__ = ["Segment"]


class Segment(object):
    """A path segment of constant jerk; typically used for tracking.

    Also computes maximum ``|velocity|`` and ``|acceleration|`` and,
    optionally, minimum and maximum position during the segment.

    Parameters
    ----------
    dt : `float`
       Duration of motion (sec)
    pA : `float`
       Position of starting point at time t = 0 (deg)
    vA : `float`
       Velocity of starting point at time t = 0 (deg/sec)
    pB : `float`
       Position of ending point at time t = dt (deg)
    vB : `float`
       Velocity of ending point at time t = dt (deg/sec)
    do_pos_lim : `bool`
       Compute minimum and maximum position?

    Raises
    ------
    ValueError
        If ``dt < sys.float_info.min``
    """
    def __init__(self, dt, pA, vA, pB, vB, do_pos_lim):
        self.dt = dt
        self.pA = pA
        self.vA = vA
        self.pB = pB
        self.vB = vB

        dt_sq = dt * dt

        # crude test for overflow of vx, aA, and j;
        # assumes |numerator| < sqrt(bignum),
        # tests |denominator| > sqrt(smallnum)
        # use <= to simplify unit tests
        if dt <= math.sqrt(sys.float_info.min):
            raise ValueError(f"dt={dt} <= math.sqrt(sys.float_info.min))={math.sqrt(sys.float_info.min)}")

        # compute aA, j and aB
        vx = (pB - pA) / dt
        aA = (3.0 * vx - (2.0 * vA + vB)) * 2.0 / dt
        j = (((vA + vB) / 2.0) - vx) * (12.0 / dt_sq)
        aB = aA + (j * dt)

        # compute maximum |velocity| (vpeak);
        # this may occur at the endpoints or at time t_vex = - aA/j
        # compute t_vex and vex = v(t_vex);
        # if t_vex is out of the range [0, dt), set t_vex = 0, so that vex = vA
        if abs(aA) < abs(j * dt):
            t_vex = max(-aA/j, 0.0)
        else:
            t_vex = 0.0
        vex = vA + t_vex * (aA + (t_vex / 2.0) * j)

        self.aA = aA
        self.j = j
        self.pmin = None
        self.pmax = None
        self.vpeak = max(abs(vA), abs(vB), abs(vex))
        self.apeak = max(abs(aA), abs(aB))

        # If desired, compute minimum and maximum position (pmin and pmax)
        # pmin and pmax may occur at the endpoints or at times t_pex1 or t_pex2
        # (the two solutions to the quadratic equation v(t) = 0).
        # Note that lim (j->0) t_pexArr = - vA / aA, yet the equation used below
        # is ill-behaved at small j. Also, it's hard to distinguish the cases
        # t_pexArr out of the range [0,dt], and t_pexArr unevaluatable.
        # Rather than try, I simply use - vA / aA whenever the standard
        # equation would not give me a reasonable answer
        # (hence either t_pexArr = an endpoint, or =[0] - vA / aA;
        # in the former case, - vA / aA simply won't give a maximal position).
        if do_pos_lim:
            t_pexArr = [0]*2
            numArr = [0]*2
            pexArr = [0]*2

            # compute the two times t_pexArr, and positions pexArr = p(t_pexArr);
            # if a t_pexArr is out of range [0, dt), set it to 0 (so its pexArr = pA)
            if abs(vA) < abs(aA * dt):
                t_pex_zeroj = max(-vA / aA, 0.0)
            else:
                t_pex_zeroj = 0.0
            sqrt_arg = (aA * aA) - (2.0 * vA * j)
            if sqrt_arg < 0.0:
                t_pexArr[0] = 0.0
                t_pexArr[1] = 0.0
            else:
                sqrt_val = math.sqrt(sqrt_arg)
                numArr[0] = -aA - sqrt_val
                numArr[1] = -aA + sqrt_val
                for branch in range(2):
                    if abs(numArr[branch]) < abs(j * dt):
                        t_pexArr[branch] = max(0.0, numArr[branch] / j)
                    else:
                        t_pexArr[branch] = t_pex_zeroj
            for branch in range(2):
                pexArr[branch] = pA + t_pexArr[branch] \
                    * (vA + (t_pexArr[branch] / 2.0) * (aA + (t_pexArr[branch] / 3.0) * j))

            self.pmin = min(pA, pB, pexArr[0], pexArr[1])
            self.pmax = max(pA, pB, pexArr[0], pexArr[1])

    def __repr__(self):
        return f"Segment(dt={self.dt}, pA={self.pA}, vA={self.vA}, " \
               f"pB={self.pB}, vB={self.vB}, do_pos_lim={self.do_pos_lim})"
