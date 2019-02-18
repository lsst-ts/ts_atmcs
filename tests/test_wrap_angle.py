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


class TestWrapAngle(unittest.TestCase):

    def test_invalid_range(self):
        for min_angle, dmax, angle, wrap_pos in itertools.product(
                (-100, 0, 100), (0, 359.9999), (-90, 0, 90), (False, True)):
            max_angle = min_angle + dmax
            with self.subTest(angle=angle, wrap_pos=wrap_pos, min_angle=min_angle, max_angle=max_angle):
                with self.assertRaises(ValueError):
                    ATMCSSimulator.path.wrap_angle(angle=angle, wrap_pos=wrap_pos,
                                                   min_angle=min_angle, max_angle=max_angle)

    def test_neg_wrap(self):
        for min_angle, dmax, dang in itertools.product(
                (-100, 0, 100), (360.0001, 450, 900),
                (-360.0001, -359.9999, -0.0001, 0.0001, 359.9999, 360.0001)):
            max_angle = min_angle + dmax
            angle = min_angle + dang
            with self.subTest(min_angle=min_angle, max_angle=max_angle, angle=angle):
                desired_wrapped_angle = angle
                for i in range(2):  # handle 2 wraps
                    if desired_wrapped_angle < min_angle:
                        desired_wrapped_angle += 360
                    elif desired_wrapped_angle > min_angle + 360:
                        desired_wrapped_angle -= 360

                wrapped_angle = ATMCSSimulator.path.wrap_angle(angle=angle, wrap_pos=False,
                                                               min_angle=min_angle, max_angle=max_angle)
                self.assertAlmostEqual(wrapped_angle, desired_wrapped_angle)

    def test_pos_wrap(self):
        for min_angle, dmax, dang in itertools.product(
                (-100, 0, 100), (360.0001, 450, 900),
                (-360.0001, -359.9999, -0.0001, 0.0001, 359.9999, 360.0001)):
            max_angle = min_angle + dmax
            angle = max_angle + dang
            with self.subTest(min_angle=min_angle, max_angle=max_angle, angle=angle):
                desired_wrapped_angle = angle
                for i in range(2):  # handle 2 wraps
                    if desired_wrapped_angle < max_angle - 360:
                        desired_wrapped_angle += 360
                    elif desired_wrapped_angle > max_angle:
                        desired_wrapped_angle -= 360

                wrapped_angle = ATMCSSimulator.path.wrap_angle(angle=angle, wrap_pos=True,
                                                               min_angle=min_angle, max_angle=max_angle)
                self.assertAlmostEqual(wrapped_angle, desired_wrapped_angle)


if __name__ == '__main__':
    unittest.main()
