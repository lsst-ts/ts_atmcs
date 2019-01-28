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
import math
import unittest

from lsst.ts import ATMCSSimulator


class TestSlew(unittest.TestCase):
    def check_path(self, path, t0, pA, vA, pB, vB, vmax, amax):
        """Check various aspects of a path created by `slew`.

        Parameters
        ----------
        path : `Path`
            Path to check
        t0 : `float`
            Start time of slew
        pA : `float`
            Initial position of path (deg)
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

        Notes
        -----

        Checks the following:

        - The initial time is correct.
        - Times increase monotonically.
        - The position and velocity at the end of each segment
          matches the start of the next.
        - The final position and velocity are correct.
        """
        self.assertAlmostEqual(path[0].t0, t0)
        self.assertAlmostEqual(path[0].p0, pA)
        self.assertAlmostEqual(path[0].v0, vA)

        for i in range(len(path) - 1):
            pvat0 = path[i]
            pvat1 = path[i+1]
            dt = pvat1.t0 - pvat0.t0
            self.assertGreater(dt, 0)
            pred_p1 = pvat0.p0 + dt*(pvat0.v0 + dt*0.5*pvat0.a0)
            pred_v1 = pvat0.v0 + dt*pvat0.a0
            self.assertAlmostEqual(pvat1.p0, pred_p1, places=4)
            self.assertAlmostEqual(pvat1.v0, pred_v1, places=4)

        for tpvaj in path:
            self.assertLessEqual(abs(tpvaj.v0), vmax)
            self.assertLessEqual(abs(tpvaj.a0), amax)

    def test_no_slew(self):
        """Test moving from a point to itself (no slew needed)."""
        # Arbitrary but reasonable values
        t0 = 1550000000
        vmax = 3
        amax = 2

        for pA in (-5, 0, 30):
            for vA in (0, 1):
                path = ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pA, vB=vA, vmax=vmax, amax=amax)
                self.assertEqual(path.kind, ATMCSSimulator.path.Kind.Slewing)
                self.check_path(path, t0=t0, pA=pA, vA=vA, pB=pA, vB=vA, vmax=vmax, amax=amax)
                self.assertEqual(len(path), 1)
                self.assertAlmostEqual(path[0].a0, 0)

    def test_long_fixed_points(self):
        """Test a fixed point to fixed point slew that is long enough
        to have a segment with constant velocity=+/-vmax

        This case is trivial to guess the required answer.
        """
        # Arbitrary but reasonable values
        t0 = 1540000000
        vmax = 3.5
        amax = 2.1

        # compute expected delta time and distance covered
        # going from 0 velocity to full velocity at full acceleration
        dt_vmax = vmax/amax
        dp_vmax = 0.5*amax*dt_vmax**2

        for pA in (-5, 0):
            for dp in (-2.001*dp_vmax, 10*dp_vmax):
                # time enough to ramp up to full speed
                # and stay there for at least a short time
                pB = pA + dp
                path = ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=0, pB=pB, vB=0, vmax=vmax, amax=amax)
                self.check_path(path, t0=t0, pA=pA, vA=0, pB=pB, vB=0, vmax=vmax, amax=amax)
                self.assertEqual(path.kind, ATMCSSimulator.path.Kind.Slewing)
                self.assertEqual(len(path), 4)

                self.assertEqual(len(path), 4)
                self.assertAlmostEqual(path[0].t0, t0)
                self.assertAlmostEqual(path[0].p0, pA)
                self.assertAlmostEqual(path[0].v0, 0)
                self.assertAlmostEqual(path[0].a0, math.copysign(amax, dp))

                predicted_dt1 = dt_vmax
                predicted_t1 = t0 + predicted_dt1
                predicted_dp1 = math.copysign(dp_vmax, dp)
                predicted_p1 = pA + predicted_dp1
                self.assertAlmostEqual(path[1].t0, predicted_t1, places=4)
                self.assertAlmostEqual(path[1].p0, predicted_p1)
                self.assertAlmostEqual(abs(path[1].v0), vmax)
                self.assertAlmostEqual(path[1].a0, 0)

                predicted_abs_dp2 = abs(dp) - 2*dp_vmax
                predicted_dp2 = math.copysign(predicted_abs_dp2, dp)
                predicted_p2 = path[1].p0 + predicted_dp2
                predicted_dt2 = abs(predicted_dp2)/vmax
                predicted_t2 = path[1].t0 + predicted_dt2
                self.assertAlmostEqual(path[2].t0, predicted_t2, places=4)
                self.assertAlmostEqual(path[2].p0, predicted_p2)
                self.assertAlmostEqual(abs(path[2].v0), vmax)
                self.assertAlmostEqual(path[2].a0, -path[0].a0)

                predicted_duration = 2*dt_vmax + predicted_dt2
                predicted_t3 = t0 + predicted_duration
                self.assertAlmostEqual(path[3].t0, predicted_t3, places=4)
                self.assertAlmostEqual(path[3].p0, pB)
                self.assertAlmostEqual(path[3].v0, 0)
                self.assertAlmostEqual(path[3].a0, 0)

    def test_short_fixed_points(self):
        """Test a fixed point to fixed point slew that is long enough
        to have a segment with constant velocity=+/-vmax

        This case is trivial to guess the required answer.
        """
        # Arbitrary but reasonable values
        t0 = 1560000000
        vmax = 3.5
        amax = 2.1

        # compute expected delta time and distance covered
        # going from 0 velocity to full velocity at full acceleration
        dt_vmax = vmax/amax
        dp_vmax = 0.5*amax*dt_vmax**2

        for pA in (-5, 0):
            for dp in (0.1*dp_vmax, -0.9*dp_vmax):
                # not enough time to ramp up to full speed
                pB = pA + dp
                path = ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=0, pB=pB, vB=0, vmax=vmax, amax=amax)
                self.assertEqual(path.kind, ATMCSSimulator.path.Kind.Slewing)
                self.check_path(path, t0=t0, pA=pA, vA=0, pB=pB, vB=0, vmax=vmax, amax=amax)

                self.assertEqual(len(path), 3)
                self.assertAlmostEqual(path[0].t0, t0)
                self.assertAlmostEqual(path[0].p0, pA)
                self.assertAlmostEqual(path[0].v0, 0)
                self.assertAlmostEqual(path[0].a0, math.copysign(amax, dp))

                predicted_dt1 = math.sqrt(abs(dp)/amax)
                predicted_t1 = t0 + predicted_dt1
                predicted_p1 = pA + dp/2
                predicted_v1 = predicted_dt1*amax
                self.assertAlmostEqual(path[1].t0, predicted_t1, places=4)
                self.assertAlmostEqual(path[1].p0, predicted_p1)
                self.assertAlmostEqual(abs(path[1].v0), predicted_v1)
                self.assertAlmostEqual(path[1].a0, -path[0].a0)

                predicted_t2 = t0 + 2*predicted_dt1
                self.assertAlmostEqual(path[2].t0, predicted_t2, places=4)
                self.assertAlmostEqual(path[2].p0, pB)
                self.assertAlmostEqual(path[2].v0, 0)
                self.assertAlmostEqual(path[2].a0, 0)

    def test_other_slews(self):
        # Arbitrary but reasonable values
        t0 = 1560000000
        vmax = 3.1
        amax = 1.76
        dt_vmax = vmax/amax

        for pA, dp, vA, dv in itertools.product(
            (-5, 0), (dt_vmax*0.1, dt_vmax*10), (-vmax, -2, 0, 1, vmax), (-1, 0, 2),
        ):
            pB = pA + dp
            vB = vA + dv
            if abs(vB) > vmax/ATMCSSimulator.path.SLEW_FUDGE:
                continue
            path = ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pB, vB=vB, vmax=vmax, amax=amax)
            self.assertEqual(path.kind, ATMCSSimulator.path.Kind.Slewing)
            self.check_path(path, t0=t0, pA=pA, vA=vA, pB=pB, vB=vB, vmax=vmax, amax=amax)

    def test_invalid_inputs(self):
        # Arbitrary but reasonable values
        t0 = 1530000000
        vmax = 3.1
        amax = 1.76
        pA = 1
        pB = 2
        vA = -2
        vB = 3
        # Local version of SLEW_FUDGE with a bit of margin to avoid
        # test failure due to roundoff error
        fudge = ATMCSSimulator.path.SLEW_FUDGE*1.000001

        # vmax must be >= 0
        with self.assertRaises(ValueError):
            ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pB, vB=vB, vmax=0, amax=amax)
        with self.assertRaises(ValueError):
            ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pB, vB=vB, vmax=-1, amax=amax)

        # amax must be >= 0
        with self.assertRaises(ValueError):
            ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pB, vB=vB, vmax=vmax, amax=0)
        with self.assertRaises(ValueError):
            ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pB, vB=vB, vmax=vmax, amax=-1)

        for sign in (-1, 1):
            # |vA| must be < vmax*FUDDGE
            with self.assertRaises(ValueError):
                ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=sign*vmax*fudge, pB=pB, vB=vB, vmax=vmax, amax=amax)
            with self.assertRaises(ValueError):
                ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=sign*vmax*2, pB=pB, vB=vB, vmax=vmax, amax=amax)
            # |vB| must be < vmax/FUDDGE
            with self.assertRaises(ValueError):
                ATMCSSimulator.path.slew(t0=t0, pA=pA, vA=vA, pB=pB, vB=sign*vmax*fudge, vmax=vmax, amax=amax)


if __name__ == '__main__':
    unittest.main()
