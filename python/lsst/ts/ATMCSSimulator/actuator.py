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


class Actuator:
    """Current and commanded position for an actuator.

    Parameters
    ----------
    pmin : `float`
        Minimum allowed position (deg)
    pmax : `float`
        Maximum allowed position (deg)
    vmax : `float`
        Maximum allowed velocity (deg/sec)
    amax : `float`
        Maximum allowed acceleration (deg/sec^2)
    dtmax_track : `float`
        Maximum allowed tB-tA for `set_cmd` to compute a tracking path (sec);
        if this limit is not met then `set_cmd` computes a slewing path.
        This should be larger than the maximum expected time between calls to
        `set_cmd`, but not much more than that.
    nsettle : `int` (optional)
        Number of calls to `set_cmd` after a slew finishes
        (meaning ``self.curr.kind`` is tracking)
        before ``self.kind(t)`` reports tracking instead of slewing.
    t : `float` (optional)
        Time for initial `cmd` `TPVAJ` and `curr` `Path`;
        if None then use ``time.time()``.
        This is primarily for unit tests; None is usually what you want.

    Raises
    ------
    ValueError
        If ``pmin >= pmax``, ``vmax <= 0``, or ``amax <= 0``.
    """
    def __init__(self, pmin, pmax, vmax, amax, dtmax_track, nsettle=2, t=None):
        if pmin >= pmax:
            raise ValueError(f"pmin={pmin} must be < pmax={pmax}")
        if vmax <= 0:
            raise ValueError(f"vmax={vmax} must be > 0")
        if amax <= 0:
            raise ValueError(f"vmax={vmax} must be > 0")
        self.pmin = pmin
        self.pmax = pmax
        self.vmax = vmax
        self.amax = amax
        self.dtmax_track = dtmax_track
        self.nsettle = nsettle

        if t is None:
            t = time.time()
        if pmin <= 0 and 0 < pmax:
            p0 = 0
        else:
            p0 = pmin
        self.cmd = path.TPVAJ(t0=t, p0=p0)
        self.curr = path.Path(path.TPVAJ(t0=t, p0=p0), kind=path.Kind.Stopped)
        self._ntrack = 0

    def set_cmd(self, pos, vel, t):
        """Set commanded position and velocity.

        Parameters
        ----------
        pos : `float`
            Position (deg)
        vel : `float`
            Velocity (deg/sec)
        t : `float`
            Time as a unix time, e.g. ``time.time()`` (sec)
        """
        tA = self.cmd.t0  # last commanded time
        dt = t - tA
        newcurr = None
        if 0 < dt < self.dtmax_track:
            # try tracking
            pcurr_tA, vcurr_tA = self.curr.pva(tA)[0:2]
            segment = path.Segment(dt=dt, pA=pcurr_tA, pB=pos, vA=vcurr_tA, vB=vel, do_pos_lim=True)
            if segment.vpeak <= self.vmax and segment.apeak <= self.amax \
                    and segment.pmin >= self.pmin and segment.pmax <= self.pmax:
                # tracking works
                newcurr = path.Path(
                    path.TPVAJ(t0=tA, p0=pcurr_tA, v0=vcurr_tA, a0=segment.aA, j=segment.j),
                    path.TPVAJ(t0=t, p0=pos, v0=vel),
                    kind=path.Kind.Tracking)

        if newcurr is None:
            # tracking didn't work, so slew
            pcurr_t, vcurr_t = self.curr.pva(t)[0:2]
            newcurr = path.slew(t0=t, pA=pcurr_t, vA=vcurr_t, pB=pos, vB=vel, vmax=self.vmax, amax=self.amax)
        self.cmd = path.TPVAJ(t0=t, p0=pos, v0=vel)
        self.curr = newcurr

    @property
    def curr(self):
        """Current path."""
        return self._curr

    @curr.setter
    def curr(self, curr):
        self._curr = curr
        if curr.kind == path.Kind.Tracking:
            self._ntrack += 1
        else:
            self._ntrack = 0

    def stop(self, t=None):
        """Stop the axis using maximum acceleration.

        Update the commanded position to match the end point of the stop.

        Parameters
        ----------
        t : `float` (optional)
            Time for initial `cmd` `TPVAJ` and `curr` `Path`;
            if None then use ``time.time()``.
            This is primarily for unit tests; None is usually what you want.
        """
        if t is None:
            t = time.time()
        p0, v0 = self.curr.pva(t)[0:2]
        self.curr = path.stop(p0=p0, v0=v0, t0=t, amax=self.amax)
        self.cmd = self.curr[-1]

    def abort(self, t=None, pos=None):
        """Stop motion immediately.

        Do not change the commanded position.

        Parameters
        ----------
        t : `float` (optional)
            Time for initial `cmd` `TPVAJ` and `curr` `Path`;
            if None then use ``time.time()``.
            This is primarily for unit tests; None is usually what you want.
        pos : `float` (optional)
            Position at which to stop (deg); if `None` then stop at position
            at time ``t``.
        """
        if t is None:
            t = time.time()
        if pos is None:
            pos = self.curr.pva(t)[0]
        self.curr = path.Path(path.TPVAJ(t0=t, p0=pos), kind=path.Kind.Stopped)

    def kind(self, t=None):
        """Kind of path we are currently following.

        The answer will match ``self.curr.kind`` except as follows:

        - After a slew we report ``path.kind.Slewing`` until ``nsettle``
          consecutive calls to `set_cmd` result in a path that is tracking.
        - if self.curr.kind is stopping and t > start time of last segment,
          the kind is reported as stopped.
        """
        if t is None:
            t = time.time()
        if self.curr.kind == path.Kind.Tracking:
            if self._ntrack > self.nsettle:
                return path.Kind.Tracking
            else:
                return path.Kind.Slewing
        elif self.curr.kind == path.Kind.Stopping and t > self.curr[-1].t0:
            return path.Kind.Stopped
        return self.curr.kind
