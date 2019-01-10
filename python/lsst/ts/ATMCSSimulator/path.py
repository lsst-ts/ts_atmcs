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

__all__ = ["PVAT", "Path"]

import bisect


class PVAT:
    """A path of constant acceleration.

    Parameters
    ----------
    p0 : `float`
        Initial position (deg)
    v0 : `float`
        Initial velocity (deg/sec)
    a : `float`
        Acceleration (deg/sec^2)
    t0 : `float`
        Initial time (unix seconds, e.g. from time.time())
    """
    def __init__(self, p0, v0, a, t0):
        self.p0 = float(p0)
        self.v0 = float(v0)
        self.a = float(a)
        self.t0 = float(t0)

    def pva(self, t):
        """Compute position, velocity and acceleration at a given time.

        Parameters
        ----------
        t : `float`
            Time (unix seconds, e.g. from time.time())
        """
        dt = t - self.t0
        return (
            self.p0 + (self.v0 + 0.5*self.a*dt)*dt,
            self.v0 + self.a*dt,
            self.a,
        )


class Path:
    """A path defined by a sequence of one or more PVATs.

    Parameters
    ----------
    pvats : ``iterable`` of `PVAT`
        PVATs in the path. For the `pva` method to work correctly,
        times must be in increasing order, but this is not checked.
    """
    def __init__(self, *pvats):
        if len(pvats) < 1:
            raise RuntimeError(f"pvats={pvats} needs at least one element")
        self.pvats = pvats
        self.ts = [pvat.t0 for pvat in pvats]

    def pva(self, t):
        """Compute position at a given time.

        Parameters
        ----------
        t : `float`
            Time (unix seconds, e.g. from time.time())

        Returns
        -------
        position : `float`
            Position at time ``t`` (deg), extrapolated if necessary.
        """
        ind = bisect.bisect(self.ts, t)
        if ind > 0:
            ind -= 1
        return self.pvats[ind].pva(t)

    def __len__(self):
        return len(self.pvats)

    def __getitem__(self, ind):
        """Indexed access to the PVATs that make up the path."""
        return self.pvats[ind]
