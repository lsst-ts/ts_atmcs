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

__all__ = ["stop"]

import math

from . import path


def stop(p0, v0, t0, amax):
    """Compute a path to stop as quickly as possible.

    Parameters
    ----------
    p0 : `float`
        Starting position (deg)
    v0 : `float`
        Starting velocity (deg/sec)
    t0 : `float`
        Start time of path.
    amax : `float`
        Maximum allowed acceleration (deg/sec^2)

    Returns
    -------
    path : `Path`
        A path with 1-2 segments, ending with a segment of zero acceleration
        and zero velocity.

    Raises
    ------
    ValueError if |amax| <= 0
    """
    if amax <= 0.0:
        raise ValueError(f"amax={amax} < 0")

    if v0 == 0:
        return path.Path(path.TPVAJ(t0=t0, p0=p0), kind=path.Kind.Stopped)

    tpvajs = []
    # t0 is typically large enough that small time changes
    # have poor accuracy, so to improve accuracy
    # compute the path segments using t0 = 0
    # then offset all the times before returning the path
    dt = abs(v0)/amax
    accel = -math.copysign(amax, v0)
    p1 = p0 + dt*(v0 + dt*0.5*accel)
    tpvajs.append(path.TPVAJ(t0=0, p0=p0, v0=v0, a0=accel))
    tpvajs.append(path.TPVAJ(t0=dt, p0=p1, v0=0))

    for tpvaj in tpvajs:
        tpvaj.t0 += t0

    return path.Path(*tpvajs, kind=path.Kind.Stopping)
