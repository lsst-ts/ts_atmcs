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

__all__ = ["Actuator"]

import time

from . import path
from . import slew


class Actuator:
    """Current and commanded position for an actuator.

    Parameters
    ----------
    vmax : `float`
        Maximum allowed velocity (deg/sec)
    amax : `float`
        Maximum allowed acceleration (deg/sec^2)
    """
    def __init__(self, vmax, amax):
        if vmax <= 0:
            raise ValueError(f"vmax={vmax} must be > 0")
        if amax <= 0:
            raise ValueError(f"vmax={vmax} must be > 0")

        t = time.time()
        self.vmax = vmax
        self.amax = amax
        self.cmd = path.PVAT(0, 0, 0, t)
        self.curr = path.Path(path.PVAT(0, 0, 0, t))

    def set_cmd(self, pos, vel):
        """Set commanded position and velocity.

        Parameters
        ----------
        pos : `float`
            Position (deg)
        vel : `float`
            Velocity (deg/sec)
        """
        t = time.time()
        pcurr, vcurr, acurr = self.curr.pvat(t)
        self.cmd = path.PVAT(p0=pos, v0=vel, a0=0, t0=t)
        self.curr = slew.slew(t0=t, pA=pcurr, pB=pos, vA=vcurr, vB=vel, vmax=self.vmax, amax=self.amax)

    def pva(self, t):
        """Return current position, velocity and acceleration.

        Parameters
        ----------
        t : `float`
            Time (unix seconds, e.g. from time.time())
        """
        return self.curr.pva(t)

    def stop(self):
        """Stop the axis using maximum acceleration.

        Update the commanded position to match the end point of the stop.
        """
        t0 = time.time()
        p0, v0 = self.pva(t0)[0:2]
        self.curr = slew.stop(p0=p0, v0=v0, t0=t0, amax=self.amax)
        self.cmd = self.curr[-1]

    def abort(self):
        """Stop motion immediately.

        Do not change the commanded position.
        """
        t0 = time.time()
        p0 = self.pva(t0)[0]
        self.curr = path.Path(path.PVAT(t0=t0, p0=p0, a=0, v0=0))
