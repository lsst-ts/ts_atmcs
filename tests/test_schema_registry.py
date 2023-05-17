# This file is part of ts_atmcssimulator.
#
# Developed for the Vera Rubin Observatory Telescope and Site Systems.
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

import pathlib
import unittest

from lsst.ts import atmcssimulator

# The directory holding the JSON schemas.
SCHEMAS_DIR = (
    pathlib.Path(__file__).parents[1]
    / "python"
    / "lsst"
    / "ts"
    / "atmcssimulator"
    / "schemas"
)


class SchemaRegistryTestCase(unittest.IsolatedAsyncioTestCase):
    def test_schema_registry(self) -> None:
        registry = atmcssimulator.registry
        num_schema_files = len(
            list(SCHEMAS_DIR.glob(f"*{atmcssimulator.JSON_SCHEMA_EXTENSION}"))
        )
        assert len(registry) == num_schema_files
