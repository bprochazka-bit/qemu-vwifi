#
# Tests for the --dump-config config descriptor emitted by the launchers.
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
import io
import json
import os
import runpy
import sys
import unittest
from contextlib import redirect_stdout

HERE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, HERE)

from vwifi_pseudohost import introspect  # noqa: E402


def _run_launcher(name, argv):
    """Run a launcher script's main() with argv and capture stdout+rc."""
    path = os.path.join(HERE, name)
    mod = runpy.run_path(path)
    buf = io.StringIO()
    with redirect_stdout(buf):
        rc = mod["main"](argv)
    return rc, buf.getvalue()


class IntrospectHelper(unittest.TestCase):
    def test_param_shape_and_types(self):
        import argparse
        p = argparse.ArgumentParser(description="x")
        p.add_argument("pos")
        p.add_argument("--n", type=int, default=6)
        p.add_argument("--mode", choices=["a", "b"], default="a")
        p.add_argument("--flag", action="store_true")
        params = {q["name"]: q for q in introspect.parser_to_params(p)}
        self.assertTrue(params["pos"]["positional"])
        self.assertTrue(params["pos"]["required"])
        self.assertEqual(params["n"]["type"], "int")
        self.assertEqual(params["n"]["default"], 6)
        self.assertEqual(params["mode"]["choices"], ["a", "b"])
        self.assertEqual(params["flag"]["type"], "bool")
        self.assertFalse(params["flag"]["required"])

    def test_help_and_dump_config_are_skipped(self):
        import argparse
        p = argparse.ArgumentParser()
        p.add_argument("--dump-config", action="store_true")
        p.add_argument("--real")
        names = [q["name"] for q in introspect.parser_to_params(p)]
        self.assertIn("real", names)
        self.assertNotIn("dump_config", names)
        self.assertNotIn("help", names)


class DumpConfigCLI(unittest.TestCase):
    def test_pseudohost_dump_config(self):
        rc, out = _run_launcher("pseudohost", ["--dump-config"])
        self.assertEqual(rc, 0)
        d = json.loads(out)
        self.assertEqual(d["device_type"], "wifi-host")
        self.assertEqual(d["binary"], "vwifi-pseudohost")
        names = [p["name"] for p in d["params"]]
        self.assertIn("essid", names)
        self.assertIn("profile", names)
        # profile choices are enriched with persona/hostname labels.
        prof = next(p for p in d["params"] if p["name"] == "profile")
        self.assertIn("generic", prof["choices"])
        labeled = {p["name"] for p in d["extras"]["profiles"]}
        self.assertEqual(labeled, set(prof["choices"]))

    def test_pseudoap_dump_config(self):
        rc, out = _run_launcher("pseudoap", ["--dump-config"])
        self.assertEqual(rc, 0)
        d = json.loads(out)
        self.assertEqual(d["device_type"], "wifi-ap")
        enc = next(p for p in d["params"] if p["name"] == "encryption")
        self.assertEqual(enc["choices"], ["open", "wpa2"])
        svc = {s["name"] for s in d["extras"]["services"]}
        self.assertEqual(svc, {"lpd", "nas", "http", "echo"})


if __name__ == "__main__":
    unittest.main()
