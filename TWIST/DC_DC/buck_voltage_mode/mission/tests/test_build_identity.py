# SPDX-License-Identifier: LGPL-2.1
"""Tests of tools/build_identity.py."""
import importlib.util
import pathlib
import shutil
import subprocess
import tempfile
import unittest

MISSION = pathlib.Path(__file__).resolve().parent.parent
spec = importlib.util.spec_from_file_location("build_identity", MISSION / "tools" / "build_identity.py")
bi = importlib.util.module_from_spec(spec)
spec.loader.exec_module(bi)


def git(cwd, *args):
    subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True)


class SourceHash(unittest.TestCase):
    def setUp(self):
        self.dir = pathlib.Path(tempfile.mkdtemp())
        (self.dir / "firmware").mkdir()
        (self.dir / "firmware" / "main.cpp").write_text("int main() {}\n")
        (self.dir / "data_objects.yaml").write_text("groups: []\n")

    def tearDown(self):
        shutil.rmtree(self.dir)

    def test_same_sources_give_the_same_hash(self):
        self.assertEqual(bi.source_hash(self.dir), bi.source_hash(self.dir))
        self.assertRegex(bi.source_hash(self.dir), r"^[0-9a-f]{16}$")

    def test_the_generated_header_is_not_hashed(self):
        before = bi.source_hash(self.dir)
        (self.dir / "firmware" / "build_identity.h").write_text("#define X 1\n")
        self.assertEqual(bi.source_hash(self.dir), before)

    def test_a_changed_or_renamed_file_changes_the_hash(self):
        before = bi.source_hash(self.dir)
        (self.dir / "firmware" / "main.cpp").write_text("int main() { return 1; }\n")
        changed = bi.source_hash(self.dir)
        self.assertNotEqual(changed, before)
        (self.dir / "firmware" / "main.cpp").rename(self.dir / "firmware" / "app.cpp")
        self.assertNotEqual(bi.source_hash(self.dir), changed)

    def test_data_objects_are_hashed(self):
        before = bi.source_hash(self.dir)
        (self.dir / "data_objects.yaml").write_text("groups: [x]\n")
        self.assertNotEqual(bi.source_hash(self.dir), before)

    def test_dirty_follows_the_hashed_sources_only(self):
        git(self.dir, "init", "-q")
        git(self.dir, "-c", "user.name=t", "-c", "user.email=t@t", "add", ".")
        git(self.dir, "-c", "user.name=t", "-c", "user.email=t@t", "commit", "-qm", "x")
        self.assertFalse(bi.identity(self.dir)["dirty"])
        (self.dir / "notes.txt").write_text("not a source\n")
        self.assertFalse(bi.identity(self.dir)["dirty"])
        (self.dir / "firmware" / "main.cpp").write_text("changed\n")
        self.assertTrue(bi.identity(self.dir)["dirty"])

    def test_outside_git_the_commit_is_unknown_and_dirty(self):
        ident = bi.identity(self.dir)
        self.assertEqual(ident["commit"], "unknown")
        self.assertTrue(ident["dirty"])

    def test_header_defines_the_three_macros(self):
        text = bi.header({"commit": "a" * 40, "dirty": False, "source_hash": "0123456789abcdef"})
        self.assertIn('#define BUILD_COMMIT "' + "a" * 40 + '"', text)
        self.assertIn("#define BUILD_DIRTY false", text)
        self.assertIn('#define BUILD_SOURCE_HASH "0123456789abcdef"', text)


class FirmwareBuffers(unittest.TestCase):
    def test_buffers_fit_the_identity(self):
        """The C buffers must hold a full commit and hash plus the end NUL."""
        text = (MISSION / "firmware" / "build_identity.cpp").read_text()
        self.assertIn("char build_commit[41]", text)
        self.assertIn("char build_source_hash[17]", text)


if __name__ == "__main__":
    unittest.main()
