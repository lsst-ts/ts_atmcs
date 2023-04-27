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
