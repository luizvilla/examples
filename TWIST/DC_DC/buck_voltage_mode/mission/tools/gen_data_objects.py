#!/usr/bin/env python3
# SPDX-License-Identifier: LGPL-2.1
"""Generate the firmware's ThingSet declarations from data_objects.yaml.

Usage:
    tools/gen_data_objects.py            write firmware/data_objects.{h,cpp}
    tools/gen_data_objects.py --check    exit 1 if they are out of date

The output follows the style of OwnTech Core4's user_data_objects.{h,cpp}
(branch serial_thingset_example_modular).
"""
import argparse
import pathlib
import re
import sys

import yaml

MISSION = pathlib.Path(__file__).resolve().parent.parent
SOURCE = MISSION / "data_objects.yaml"
HEADER = MISSION / "firmware" / "data_objects.h"
BODY = MISSION / "firmware" / "data_objects.cpp"

# type -> (C type, ThingSet macro suffix, takes a decimals argument)
TYPES = {
    "float": ("float", "FLOAT", True),
    "bool": ("bool", "BOOL", False),
    "uint8": ("uint8_t", "UINT8", False),
    "uint16": ("uint16_t", "UINT16", False),
    "uint32": ("uint32_t", "UINT32", False),
    "int32": ("int32_t", "INT32", False),
}

BANNER = (
    "/*\n"
    " * GENERATED from data_objects.yaml by tools/gen_data_objects.py.\n"
    " * Do not edit: change data_objects.yaml and run the generator again.\n"
    " *\n"
    " * SPDX-License-Identifier: LGPL-2.1\n"
    " */\n"
)


class SpecError(Exception):
    pass


GROUP_KEYS = {"name", "id", "objects"}
OBJECT_KEYS = {"name", "id", "type", "decimals", "unit", "variable", "description"}


def macro_name(name):
    """rV1Low_V -> V1_LOW, wVoltageRef_V -> VOLTAGE_REF, Mode -> MODE (unit dropped)."""
    if len(name) > 1 and name[0] in "rw" and name[1].isupper():
        name = name[1:]
    name = name.split("_")[0] if "_" in name else name
    snake = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", "_", name)
    return re.sub(r"[^A-Za-z0-9]+", "_", snake).upper()


def access_of(name):
    """ThingSet convention: r prefix read-only, everything else writable."""
    if len(name) > 1 and name[0] == "r" and name[1].isupper():
        return "THINGSET_ANY_R"
    return "THINGSET_ANY_RW"


def validate(spec):
    if not isinstance(spec, dict) or "groups" not in spec:
        raise SpecError("data_objects.yaml needs a 'groups' list")
    if not re.fullmatch(r"[A-Z][A-Z0-9_]*", str(spec.get("subset", ""))):
        raise SpecError("'subset' must be an upper-case name such as SER")
    ids, names, variables, macros = {}, set(), set(), set()
    for group in spec["groups"]:
        unknown = set(group) - GROUP_KEYS
        if unknown:
            raise SpecError(f"group {group.get('name', '?')}: unknown key(s) {sorted(map(str, unknown))}")
        for key in ("name", "id", "objects"):
            if key not in group:
                raise SpecError(f"group {group.get('name', '?')}: missing '{key}'")
        entries = [(group["id"], f"group {group['name']}")]
        for obj in group["objects"]:
            where = f"{group['name']}/{obj.get('name', '?')}"
            unknown = set(obj) - OBJECT_KEYS
            if unknown:
                raise SpecError(f"{where}: unknown key(s) {sorted(map(str, unknown))}"
                                " (a comma inside an unquoted description splits it)")
            for key in ("name", "id", "type", "variable"):
                if key not in obj:
                    raise SpecError(f"{where}: missing '{key}'")
            if obj["type"] not in TYPES:
                raise SpecError(f"{where}: unknown type '{obj['type']}' (use {', '.join(TYPES)})")
            if TYPES[obj["type"]][2] and "decimals" not in obj:
                raise SpecError(f"{where}: a float needs 'decimals'")
            unit = obj.get("unit")
            if unit is not None and not obj["name"].endswith("_" + str(unit)):
                raise SpecError(f"{where}: the name must end with _{unit} to match its unit")
            if obj["name"] in names:
                raise SpecError(f"{where}: name used twice")
            if obj["variable"] in variables:
                raise SpecError(f"{where}: variable {obj['variable']} already exposed")
            macro = macro_name(group["name"]) + "_" + macro_name(obj["name"])
            if macro in macros:
                raise SpecError(f"{where}: its ID macro ID_{macro} clashes with another object")
            macros.add(macro)
            names.add(obj["name"])
            variables.add(obj["variable"])
            entries.append((obj["id"], where))
        for ident, where in entries:
            if not isinstance(ident, int) or ident <= 0:
                raise SpecError(f"{where}: id must be a positive integer")
            if ident in ids:
                raise SpecError(f"{where}: id 0x{ident:X} already used by {ids[ident]}")
            ids[ident] = where


def render(spec):
    subset = spec["subset"]
    h = [BANNER, "#pragma once\n", "#include <stdbool.h>", "#include <stdint.h>\n",
         "#define ID_ROOT 0x00", f"#define SUBSET_{subset} (1U << 0)\n"]
    cpp = [BANNER, '#include "data_objects.h"\n',
           "/* Compiled only when app.conf enables ThingSet (CONFIG_THINGSET=y). */",
           "#if defined(CONFIG_THINGSET)\n", "#include <thingset.h>\n"]
    for group in spec["groups"]:
        gmacro = "ID_" + macro_name(group["name"])
        h.append(f"/* {group['name']} */")
        h.append(f"#define {gmacro} 0x{group['id']:02X}")
        cpp.append(f'THINGSET_ADD_GROUP(ID_ROOT, {gmacro}, "{group["name"]}", THINGSET_NO_CALLBACK);')
        for obj in group["objects"]:
            omacro = f"{gmacro}_{macro_name(obj['name'])}"
            ctype, suffix, has_decimals = TYPES[obj["type"]]
            h.append(f"#define {omacro} 0x{obj['id']:02X}")
            args = [gmacro, omacro, f'"{obj["name"]}"', f"&{obj['variable']}"]
            if has_decimals:
                args.append(str(obj["decimals"]))
            args += [access_of(obj["name"]), f"SUBSET_{subset}"]
            if obj.get("description"):
                cpp.append(f"/* {obj['description']} */")
            cpp.append(f"THINGSET_ADD_ITEM_{suffix}({', '.join(args)});")
        h.append("")
        cpp.append("")
    h.append("/* Variables exposed above; defined in main.cpp. */")
    for group in spec["groups"]:
        for obj in group["objects"]:
            h.append(f"extern {TYPES[obj['type']][0]} {obj['variable']};")
    cpp.append("#endif /* CONFIG_THINGSET */")
    return "\n".join(h) + "\n", "\n".join(cpp).rstrip("\n") + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--check", action="store_true", help="fail if the generated files are out of date")
    args = parser.parse_args(argv)
    try:
        spec = yaml.safe_load(SOURCE.read_text())
        validate(spec)
    except (SpecError, yaml.YAMLError) as err:
        print(f"data_objects.yaml: {err}", file=sys.stderr)
        return 2
    header, body = render(spec)
    if args.check:
        stale = [p.name for p, text in ((HEADER, header), (BODY, body))
                 if not p.exists() or p.read_text() != text]
        if stale:
            print("out of date: " + ", ".join(stale) + " (run tools/gen_data_objects.py)", file=sys.stderr)
            return 1
        return 0
    HEADER.write_text(header)
    BODY.write_text(body)
    return 0


if __name__ == "__main__":
    sys.exit(main())
