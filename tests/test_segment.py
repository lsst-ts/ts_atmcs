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
import sys
import unittest

import numpy as np

from lsst.ts import ATMCSSimulator


class TestSegment(unittest.TestCase):
    def check_segment(self, pA, vA, pB, vB, dt):
        """Check that a segment meets is constructor conditions
        """
        segment = ATMCSSimulator.path.Segment(dt=dt, pA=pA, vA=vA, pB=pB, vB=vB, do_pos_lim=True)
        self.assertAlmostEqual(segment.dt, dt)
        self.assertAlmostEqual(segment.pA, pA)
        self.assertAlmostEqual(segment.pB, pB)
        self.assertAlmostEqual(segment.vA, vA)
        self.assertAlmostEqual(segment.vB, vB)
        aA = segment.aA
        j = segment.j
        desired_pB = pA + dt*(vA + dt*(0.5*aA + dt*(1/6)*j))
        desired_vB = vA + dt*(aA + dt*0.5*j)
        self.assertAlmostEqual(segment.pB, desired_pB)
        self.assertAlmostEqual(segment.vB, desired_vB)

        # estimate limits by computing at many points
        desired_pmin = min(pA, pB)
        desired_pmax = max(pA, pB)
        desired_vpeak = max(abs(vA), abs(vB))
        for t in np.linspace(start=0, stop=dt, num=100):
            pt = pA + t*(vA + t*(0.5*aA + t*j/6))
            vt = vA + t*(aA + t*0.5*j)
            desired_pmin = min(pt, desired_pmin)
            desired_pmax = max(pt, desired_pmax)
            desired_vpeak = max(abs(vt), desired_vpeak)
        self.assertAlmostEqual(segment.pmin, desired_pmin, places=3)
        self.assertAlmostEqual(segment.pmax, desired_pmax, places=3)
        self.assertAlmostEqual(segment.vpeak, desired_vpeak, places=3)

        aB = aA + dt*j
        desired_apeak = max(abs(aA), abs(aB))
        self.assertAlmostEqual(segment.apeak, desired_apeak)

    def test_basics(self):
        # pA, vA, pB, vB, dt, do_pos_lim
        for pA, vA, pB, vB, dt in itertools.product(
            (0, -0.5, 0.2), (0, -0.2, 0.1), (0, 0.3, -0.6), (0, 0.3, -0.2), (1, 5),
        ):
            self.check_segment(dt=dt, pA=pA, vA=vA, pB=pB, vB=vB)

    def test_invalid_inputs(self):
        for dt in (0, math.sqrt(sys.float_info.min)):
            with self.assertRaises(ValueError):
                ATMCSSimulator.path.Segment(dt=dt, pA=1, vA=2, pB=1, vB=2, do_pos_lim=True)


if __name__ == '__main__':
    unittest.main()
