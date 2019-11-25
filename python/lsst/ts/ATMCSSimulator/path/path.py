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

__all__ = ["Kind", "TPVAJ", "Path"]

import bisect
import enum


class Kind(enum.Enum):
    Stopped = enum.auto()
    Tracking = enum.auto()
    Slewing = enum.auto()
    Stopping = enum.auto()


class TPVAJ:
    """A path of constant jerk.

    Parameters
    ----------
    t0 : `float`
        Initial time (unix seconds, e.g. from time.time())
    p0 : `float` (optional)
        Initial position (deg)
    v0 : `float` (optional)
        Initial velocity (deg/sec)
    a0 : `float` (optional)
        Acceleration (deg/sec^2)
    j : `float` (optional)
        Jerk (deg/sec^3)
    """
    def __init__(self, t0, p0=0, v0=0, a0=0, j=0):
        self.t0 = float(t0)
        self.p0 = float(p0)
        self.v0 = float(v0)
        self.a0 = float(a0)
        self.j = float(j)

    def pva(self, t):
        """Compute position, velocity and acceleration at a given time.

        Parameters
        ----------
        t : `float`
            Time (unix seconds, e.g. from time.time())
        """
        dt = t - self.t0
        return (
            self.p0 + dt*(self.v0 + dt*(0.5*self.a0 + dt*self.j/6)),
            self.v0 + dt*(self.a0 + dt*(0.5*self.j)),
            self.a0 + dt*self.j,
        )

    def __repr__(self):
        fields = [f"t0={self.t0}"]
        for name in ("p0", "v0", "a0", "j"):
            val = getattr(self, name)
            if val != 0:
                fields.append(f"{name}={val}")
        return f"TPVAJ({', '.join(fields)})"


class Path:
    """A path defined by a sequence of one or more PVATs.

    Parameters
    ----------
    tpvajs : ``iterable`` of `TPVAJ`
        PVATs in the path. For the `pva` method to work correctly,
        times must be in increasing order, but this is not checked.
    kind : `Kind`
        Kind of path
    """
    def __init__(self, *tpvajs, kind):
        if len(tpvajs) < 1:
            raise RuntimeError(f"tpvajs={tpvajs} needs at least one element")
        self.tpvajs = tpvajs
        self.kind = Kind(kind)
        self.ts = [tpvaj.t0 for tpvaj in tpvajs]

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
        return self.tpvajs[ind].pva(t)

    def __len__(self):
        return len(self.tpvajs)

    def __getitem__(self, ind):
        """Indexed access to the PVATs that make up the path."""
        return self.tpvajs[ind]

    def __repr__(self):
        tpvajs_str = ", ".join(repr(tpvaj) for tpvaj in self.tpvajs[:-1])
        return f"Path({tpvajs_str}, kind={self.kind})"
