#!/usr/bin/env python3
"""Test false feasibility, missing evidence, and frozen-evidence replay; offline."""
import copy
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

import assess_touch_separation as screen


def complete_stratum(chip="MPR121"):
    return {"chip": chip, "board_id": "synthetic-board", "firmware": "synthetic-test",
            "config_sha256": "0" * 64, "lane": 16,
            "scale": "native-baseline-minus-filtered" if chip == "MPR121" else "native-button-diff",
            "quality": "valid-fresh-coherent", "idle_max": [1, 4], "hover_max": [12, 16],
            "contact_onset_min": [30, 33], "contact_hold_min": [24, 27],
            "released_max": [4, 7], "drift_margin_counts": 2}


def document(stratum=None):
    return {"schema": "touch-signal-margin-v1", "origin": "synthetic-example",
            "strata": [complete_stratum() if stratum is None else stratum]}


def trial(mode, count, delta, dt, **extras):
    return {"file": mode + ".json", "mode": mode, "repeat": 1, "session": 1,
            "features": {"count_at_onset": count, "delta_15ms": delta,
                         "delta_15ms_actual_dt": dt, "delta_15ms_per_ms": delta / dt, **extras}}


class SignalMargins(unittest.TestCase):
    def test_positive_margin_uses_worst_uncertainty_endpoints(self):
        result = screen.margin_screen(document())
        value = result["strata"][0]
        self.assertEqual(value["abstract_ON_cutoff_range"], [19, 28])
        self.assertEqual(value["abstract_release_cutoff_range"], [10, 22])
        self.assertEqual(value["verdict"], "signal_interval_exists_not_validated")
        self.assertFalse(result["deployment_ready"])

    def test_saturated_hover_and_contact_have_no_interval(self):
        stratum = complete_stratum("MBR3116")
        stratum.update(hover_max=[255, 255], contact_onset_min=[255, 255])
        value = screen.margin_screen(document(stratum))["strata"][0]
        self.assertEqual(value["verdict"], "no_robust_cutoff_interval")
        self.assertIsNone(value["abstract_ON_cutoff_range"])

    def test_one_integer_cutoff_is_allowed_without_margin(self):
        stratum = complete_stratum()
        stratum.update(hover_max=[19, 19], contact_onset_min=[20, 20], drift_margin_counts=0)
        self.assertEqual(screen.margin_screen(document(stratum))["strata"][0]["abstract_ON_cutoff_range"], [20, 20])

    def test_baseline_uncertainty_can_destroy_point_estimate_gap(self):
        stratum = complete_stratum()
        stratum.update(hover_max=[16, 19], contact_onset_min=[19, 22], drift_margin_counts=0)
        self.assertEqual(screen.margin_screen(document(stratum))["strata"][0]["verdict"], "no_robust_cutoff_interval")

    def test_release_or_hold_failure_blocks_overall_feasibility(self):
        stratum = complete_stratum()
        stratum["contact_hold_min"] = [8, 11]
        value = screen.margin_screen(document(stratum))["strata"][0]
        self.assertIsNotNone(value["abstract_ON_cutoff_range"])
        self.assertIsNone(value["abstract_release_cutoff_range"])
        self.assertEqual(value["verdict"], "no_robust_cutoff_interval")

    def test_release_must_be_below_on(self):
        stratum = complete_stratum()
        stratum.update(hover_max=[17, 17], contact_onset_min=[18, 18],
                       released_max=[17, 17], drift_margin_counts=0)
        value = screen.margin_screen(document(stratum))["strata"][0]
        self.assertEqual(value["abstract_ON_cutoff_range"], [18, 18])
        self.assertIsNone(value["abstract_release_cutoff_range"])

    def test_no_default_or_zero_fill_for_missing_quantities(self):
        for key in ("idle_max", "hover_max", "contact_onset_min", "contact_hold_min", "released_max",
                    "board_id", "firmware", "config_sha256", "lane", "scale", "quality", "drift_margin_counts"):
            with self.subTest(key=key):
                stratum = complete_stratum()
                stratum[key] = None
                value = screen.margin_screen(document(stratum))["strata"][0]
                self.assertEqual(value["verdict"], "unassessable")
                self.assertNotIn("abstract_ON_cutoff_range", value)

    def test_display_scale_is_not_assumed_to_be_native(self):
        stratum = complete_stratum()
        stratum["scale"] = "panel-x2"
        self.assertEqual(screen.margin_screen(document(stratum))["strata"][0]["verdict"], "unassessable")

    def test_failed_or_stale_read_quality_cannot_pass(self):
        for quality in ("failed", "stale", "cached", "valid"):
            with self.subTest(quality=quality):
                stratum = complete_stratum()
                stratum["quality"] = quality
                self.assertEqual(screen.margin_screen(document(stratum))["strata"][0]["verdict"], "unassessable")

    def test_chip_scales_remain_separate(self):
        doc = document()
        doc["strata"].append(complete_stratum("MBR3116"))
        values = screen.margin_screen(doc)["strata"]
        self.assertEqual([v["chip"] for v in values], ["MPR121", "MBR3116"])
        self.assertEqual(len(values), 2)

    def test_negative_mpr_delta_is_supported(self):
        stratum = complete_stratum()
        stratum["idle_max"] = [-2, 1]
        self.assertEqual(screen.margin_screen(document(stratum))["strata"][0]["verdict"], "signal_interval_exists_not_validated")

    def test_corrupt_summaries_are_rejected(self):
        for key, value in (("hover_max", [16, 12]), ("hover_max", [12, 1024]),
                           ("hover_max", [True, 16]), ("hover_max", [12.0, 16]),
                           ("hover_max", [12]), ("lane", 32), ("lane", True),
                           ("config_sha256", "x" * 64), ("drift_margin_counts", -1),
                           ("chip", "unknown"), ("board_id", "")):
            with self.subTest(key=key, value=value):
                stratum = complete_stratum()
                stratum[key] = value
                with self.assertRaises(ValueError):
                    screen.margin_screen(document(stratum))

    def test_invalid_mbr_counts_are_rejected(self):
        for counts in ([-1, 16], [16, 256]):
            stratum = complete_stratum("MBR3116")
            stratum["hover_max"] = counts
            with self.assertRaises(ValueError):
                screen.margin_screen(document(stratum))

    def test_input_is_not_mutated(self):
        doc = document()
        before = copy.deepcopy(doc)
        screen.margin_screen(doc)
        self.assertEqual(doc, before)


class CombinedFeatures(unittest.TestCase):
    def test_dominating_hover_obstructs_monotone_combination(self):
        values = [trial("very_slow_contact", 188, 15, 17), trial("fast_hover", 199, 17, 15)]
        result = screen.dominance_screen(values, screen.FEATURES[:2])
        self.assertEqual(result["verdict"], "obstructed")
        self.assertEqual(len(result["dominance_witnesses"]), 1)

    def test_feature_tradeoff_does_not_prove_impossibility(self):
        values = [trial("contact", 210, 15, 17), trial("hover", 199, 17, 15)]
        result = screen.dominance_screen(values, screen.FEATURES[:2])
        self.assertEqual(result["verdict"], "no_obstruction_found_not_validated")
        self.assertFalse(result["dominance_witnesses"])

    def test_rates_use_actual_intervals_not_rounded_display_float(self):
        value = trial("contact", 188, 15, 17)
        value["features"]["delta_15ms_per_ms"] = 999
        self.assertEqual(screen.causal_value(value, "delta_15ms_per_ms"), screen.Fraction(15, 17))

    def test_missing_onset_is_unassessable(self):
        missing = trial("near_hover", 100, 10, 15)
        missing["features"] = {}
        result = screen.dominance_screen([missing], screen.FEATURES[:2])
        self.assertEqual(result["unassessable_trials"], 1)
        self.assertEqual(result["hover_action_onsets"], 0)


class FrozenReplay(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.package = screen.REPO / "docs/MBR3116_round90b_事件判据证据包.zip"
        cls.before = screen.sha256(cls.package.read_bytes())
        cls.result = screen.replay(cls.package)

    def test_counts_and_aborts_reproduced(self):
        self.assertEqual(self.result["verified_evidence_files"], 73)
        self.assertEqual((self.result["confirmed_trials"], self.result["frames"]), (14, 1344))
        self.assertEqual(len(self.result["excluded_jobs"]), 2)
        full = self.result["combinations"][1]
        self.assertEqual((full["contact_action_onsets"], full["hover_action_onsets"], full["unassessable_trials"]), (9, 3, 2))

    def test_combined_real_data_has_slow_contact_witnesses(self):
        full = self.result["combinations"][1]
        self.assertEqual(full["verdict"], "obstructed")
        self.assertEqual(len(full["dominance_witnesses"]), 2)
        self.assertEqual({v["contact_action"]["mode"] for v in full["dominance_witnesses"]}, {"very_slow_contact"})
        self.assertEqual({v["contact_action"]["repeat"] for v in full["dominance_witnesses"]}, {1, 2})

    def test_all_single_lower_bound_cutoffs_have_no_gap(self):
        self.assertTrue(all(not value["feasible_lower_bound_gate"] for value in self.result["single_feature_cutoff_bounds"].values()))

    def test_archive_remains_unchanged(self):
        self.assertEqual(screen.sha256(self.package.read_bytes()), self.before)

    def test_modified_package_is_rejected_before_execution(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "bad.zip"
            target.write_bytes(self.package.read_bytes() + b"changed")
            with self.assertRaises(ValueError):
                screen.replay(target)

    def test_standalone_cli_replays_from_unrelated_directory(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "result.json"
            subprocess.run([sys.executable, "-I", str(Path(screen.__file__).resolve()),
                            "replay", "--out", str(target)], cwd=temporary,
                           check=True, stdout=subprocess.DEVNULL)
            actual = json.loads(target.read_text(encoding="utf-8"))
            actual.pop("tool_sha256")
            self.assertEqual(actual, self.result)

    def test_cli_rejects_overwriting_input_evidence(self):
        with tempfile.TemporaryDirectory() as temporary:
            target = Path(temporary) / "input.json"
            target.write_text(json.dumps(document()), encoding="utf-8")
            before = target.read_bytes()
            result = subprocess.run([sys.executable, "-I", str(Path(screen.__file__).resolve()),
                                     "margin", str(target), "--out", str(target)],
                                    capture_output=True)
            self.assertEqual(result.returncode, 2)
            self.assertEqual(target.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
