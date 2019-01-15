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

import itertools
import unittest

from lsst.ts import ATMCSSimulator


class TestStop(unittest.TestCase):
    def check_path(self, path, t0, p0, v0, amax):
        """Check various aspects of a path

        Checks the following:

        - The initial time is correct.
        - Times increase monotonically.
        - The position and velocity at the end of each segment
          matches the start of the next.
        - The final position and velocity are correct.

        Parameters
        ----------
        path : `Path`
            Path to check
        t0 : `float`
            Initial time (unix seconds, e.g. from time.time())
        p0 : `float` (optional)
            Initial position (deg)
        v0 : `float` (optional)
            Initial velocity (deg/sec)
        amax : `float` (optional)
            Maximum allowed acceleration (deg/sec^2)
        """
        self.assertAlmostEqual(path[0].t0, t0)
        self.assertAlmostEqual(path[0].p0, p0)
        self.assertAlmostEqual(path[0].v0, v0)

        self.assertIn(len(path), (1, 2))

        self.assertEqual(path[-1].v0, 0)
        self.assertEqual(path[-1].a0, 0)

        if len(path) > 1:
            pvat0 = path[0]
            pvat1 = path[1]
            dt = pvat1.t0 - pvat0.t0
            self.assertGreater(dt, 0)
            pred_p1 = pvat0.p0 + dt*(pvat0.v0 + dt*0.5*pvat0.a0)
            pred_v1 = pvat0.v0 + dt*pvat0.a0
            self.assertAlmostEqual(pvat1.p0, pred_p1, places=4)
            self.assertAlmostEqual(pvat1.v0, pred_v1, places=4)

    def test_slew_to_stop(self):
        t0 = 1550000000
        amax = 10

        for p0, v0 in itertools.product(
            (-5, 0, 30), (-3, -1, 2, 4),
        ):
            path = ATMCSSimulator.path.stop(p0=p0, v0=v0, t0=t0, amax=amax)
            self.assertEqual(path.kind, ATMCSSimulator.path.Kind.Stopping)
            self.assertEqual(len(path), 2)
            self.check_path(path, p0=p0, v0=v0, t0=t0, amax=amax)

    def test_already_stopped(self):
        """Test stop when already stopped."""
        # Arbitrary but reasonable values
        t0 = 1550000000
        amax = 2

        for p0 in (-5, 0, 30):
            path = ATMCSSimulator.path.stop(p0=p0, v0=0, t0=t0, amax=amax)
            self.assertEqual(len(path), 1)
            self.assertEqual(path.kind, ATMCSSimulator.path.Kind.Stopped)
            self.check_path(path, p0=p0, v0=0, t0=t0, amax=amax)

    def test_invalid_inputs(self):
        # Arbitrary but reasonable values
        t0 = 1530000000
        p0 = 1
        v0 = -2

        # amax must be >= 0
        with self.assertRaises(ValueError):
            ATMCSSimulator.path.stop(p0=p0, v0=v0, t0=t0, amax=0)
        with self.assertRaises(ValueError):
            ATMCSSimulator.path.stop(p0=p0, v0=v0, t0=t0, amax=-1)


if __name__ == '__main__':
    unittest.main()
