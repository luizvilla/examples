# SPDX-License-Identifier: LGPL-2.1
"""Tests of tools/gen_data_objects.py. Run: python3 -m unittest discover -s tests -p 'test_*.py'"""
import importlib.util
import pathlib
import unittest

import yaml

TOOLS = pathlib.Path(__file__).resolve().parent.parent / "tools"
spec = importlib.util.spec_from_file_location("gen", TOOLS / "gen_data_objects.py")
gen = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gen)


def table(*objects, subset="SER"):
    return {"subset": subset, "groups": [{"name": "Measurements", "id": 0x05, "objects": list(objects)}]}


VOLT = {"name": "rV1Low_V", "id": 0x50, "type": "float", "decimals": 2, "unit": "V", "variable": "v1"}


class MissionTable(unittest.TestCase):
    def test_mission_table_is_valid_and_generated_files_are_up_to_date(self):
        self.assertEqual(gen.main(["--check"]), 0)


class Rendering(unittest.TestCase):
    def test_read_only_prefix_gives_read_access(self):
        _, body = gen.render(table(VOLT))
        self.assertIn('THINGSET_ADD_ITEM_FLOAT(ID_MEASUREMENTS, ID_MEASUREMENTS_V1_LOW, "rV1Low_V", &v1, 2, THINGSET_ANY_R, SUBSET_SER);', body)

    def test_setting_without_prefix_is_writable(self):
        mode = {"name": "Mode", "id": 0x61, "type": "uint8", "variable": "mode"}
        header, body = gen.render(table(mode))
        self.assertIn("THINGSET_ANY_RW", body)
        self.assertIn("extern uint8_t mode;", header)


class Strings(unittest.TestCase):
    COMMIT = {"name": "rCommit", "id": 0x70, "type": "string", "size": 41, "variable": "commit"}

    def test_string_is_passed_as_a_buffer_with_its_size(self):
        header, body = gen.render(table(self.COMMIT))
        self.assertIn('THINGSET_ADD_ITEM_STRING(ID_MEASUREMENTS, ID_MEASUREMENTS_COMMIT, "rCommit", commit, 41, THINGSET_ANY_R, SUBSET_SER);', body)
        self.assertIn("extern char commit[41];", header)

    def test_string_needs_a_size(self):
        with self.assertRaises(gen.SpecError):
            gen.validate(table({k: v for k, v in self.COMMIT.items() if k != "size"}))

    def test_only_strings_take_a_size(self):
        with self.assertRaises(gen.SpecError):
            gen.validate(table(dict(VOLT, size=4)))


class FloatArrays(unittest.TestCase):
    DATA = {"name": "rData", "id": 0x87, "type": "float_array", "size": 64, "decimals": 3, "variable": "data"}

    def test_array_is_wrapped_and_declared_with_its_size(self):
        header, body = gen.render(table(self.DATA))
        self.assertIn("THINGSET_DEFINE_FLOAT_ARRAY(data_ts, 3, data, 64);", body)
        self.assertIn('THINGSET_ADD_ITEM_ARRAY(ID_MEASUREMENTS, ID_MEASUREMENTS_DATA, "rData", &data_ts, THINGSET_ANY_R, SUBSET_SER);', body)
        self.assertIn("extern float data[64];", header)

    def test_array_needs_a_size(self):
        with self.assertRaises(gen.SpecError):
            gen.validate(table({k: v for k, v in self.DATA.items() if k != "size"}))


class Callbacks(unittest.TestCase):
    def test_group_callback_is_declared_and_passed(self):
        data = table(VOLT)
        data["groups"][0]["callback"] = "on_measurements"
        _, body = gen.render(data)
        self.assertIn("void on_measurements(enum thingset_callback_reason reason);", body)
        self.assertIn('THINGSET_ADD_GROUP(ID_ROOT, ID_MEASUREMENTS, "Measurements", on_measurements);', body)

    def test_callback_must_be_a_function_name(self):
        data = table(VOLT)
        data["groups"][0]["callback"] = "on measurements()"
        with self.assertRaises(gen.SpecError):
            gen.validate(data)


class Validation(unittest.TestCase):
    def rejects(self, data, words):
        with self.assertRaises(gen.SpecError) as ctx:
            gen.validate(data)
        self.assertIn(words, str(ctx.exception))

    def test_group_name_used_by_core_is_refused(self):
        data = table(VOLT)
        data["groups"][0]["name"] = "Control"
        self.rejects(data, "already used by OwnTech Core")

    def test_id_reserved_by_core_is_refused(self):
        self.rejects(table(dict(VOLT, id=0x8001)), "reserved")

    def test_duplicate_id(self):
        self.rejects(table(VOLT, dict(VOLT, name="rV2Low_V", variable="v2")), "already used")

    def test_unit_must_match_name(self):
        self.rejects(table(dict(VOLT, unit="A")), "must end with _A")

    def test_float_needs_decimals(self):
        self.rejects(table({k: v for k, v in VOLT.items() if k != "decimals"}), "needs 'decimals'")

    def test_unknown_type(self):
        self.rejects(table(dict(VOLT, type="double")), "unknown type")

    def test_unquoted_comma_in_description_is_caught(self):
        data = yaml.safe_load(
            "subset: SER\ngroups:\n  - name: Measurements\n    id: 0x05\n    objects:\n"
            "      - {name: rDuty, id: 0x5A, type: float, decimals: 4, variable: d, description: Duty, 0 to 1}\n")
        self.rejects(data, "unknown key")

    def test_macro_clash(self):
        current = {"name": "rV1Low_A", "id": 0x53, "type": "float", "decimals": 2, "unit": "A", "variable": "i1"}
        self.rejects(table(VOLT, dict(current, name="rV1Low_A")), "clashes")

    def test_same_variable_twice(self):
        self.rejects(table(VOLT, dict(VOLT, name="rV2Low_V", id=0x51)), "already exposed")


if __name__ == "__main__":
    unittest.main()
