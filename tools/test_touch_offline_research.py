"""Official-register fixtures and corrupted-evidence rejection tests; offline only."""

import argparse
import copy
import csv
import hashlib
import io
import json
import shutil
import sys
import tempfile
import unittest
from pathlib import Path

import analyze_mbr_raw_evidence as mbr
import decode_mpr121_snapshot as mpr

REPO = Path(__file__).resolve().parents[1]
MANIFEST_PATH = REPO / "docs/MBR3116_round90f_输入清单.json"
DATA_ROOT = REPO.parent / "_dev_tools/round89q_no_autoreset_20261002"


def fixture(**registers):
    return {"schema": mpr.SCHEMA, "origin": "synthetic-test",
            "registers": {"0x5C": 0x10, "0x5D": 0x28, "0x5E": 0xCC, **registers}}


class MprOfficialFields(unittest.TestCase):
    def test_source_config_0x28_means_six_samples(self):
        output = mpr.decode(fixture())
        self.assertEqual(output["global"], {"ffi_samples": 6, "cdc_uA": 16, "cdt_us": .5,
                                          "sfi_samples": 6, "esi_ms": 1, "nominal_output_update_ms": 6})

    def test_sfi_nxp_table(self):
        for encoded, samples in ((0x20, 4), (0x28, 6), (0x30, 10), (0x38, 18)):
            with self.subTest(config2=encoded):
                self.assertEqual(mpr.decode(fixture(**{"0x5D": encoded}))["global"]["sfi_samples"], samples)

    def test_ffi_nxp_table(self):
        for encoded, samples in ((0x10, 6), (0x50, 10), (0x90, 18), (0xD0, 34)):
            with self.subTest(config1=encoded):
                self.assertEqual(mpr.decode(fixture(**{"0x5C": encoded}))["global"]["ffi_samples"], samples)

    def test_esi_nxp_table(self):
        for encoded, interval in enumerate((1, 2, 4, 8, 16, 32, 64, 128)):
            with self.subTest(esi=encoded):
                self.assertEqual(mpr.decode(fixture(**{"0x5D": 0x28 | encoded}))["global"]["esi_ms"], interval)

    def test_cl_seed_source_and_tracking(self):
        expected = {0x0C: (True, "current baseline register"), 0x4C: (False, "disabled"),
                    0x8C: (True, "high 5 bits of first electrode data"), 0xCC: (True, "all 10 bits of first electrode data")}
        for encoded, (tracking, meaning) in expected.items():
            with self.subTest(ecr=encoded):
                value = mpr.decode(fixture(**{"0x5E": encoded}))["ecr"]
                self.assertIs(value["tracking_enabled"], tracking)
                self.assertIn(meaning, value["cl_meaning"])

    def test_bva_is_not_retry_mask(self):
        before = mpr.decode(fixture(**{"0x7B": 0x3B}))["autoconfig"]
        after = mpr.decode(fixture(**{"0x7B": 0x0B}))["autoconfig"]
        self.assertEqual((before["bva_code"], before["retry_code"]), (2, 3))
        self.assertEqual((after["bva_code"], after["retry_code"]), (2, 0))
        self.assertTrue(after["ace"])
        self.assertTrue(after["are"])

    def test_individual_override_and_packed_cdt(self):
        decoded = mpr.decode(fixture(**{"0x5F": 0, "0x60": 63, "0x6C": 0x70}))["electrodes"]
        self.assertEqual((decoded[0]["effective_cdc_uA"], decoded[0]["effective_cdt_us"]), (16, .5))
        self.assertEqual((decoded[1]["effective_cdc_uA"], decoded[1]["effective_cdt_us"]), (63, 32))
        self.assertTrue(decoded[0]["charge_parameters_complete"])
        self.assertTrue(decoded[1]["charge_parameters_complete"])

    def test_missing_charge_bytes_are_unknown_not_global(self):
        electrode = mpr.decode(fixture())["electrodes"][0]
        self.assertIsNone(electrode["effective_cdc_uA"])
        self.assertIsNone(electrode["effective_cdt_us"])
        self.assertFalse(electrode["charge_parameters_complete"])

    def test_cdt_nxp_table(self):
        for code, duration in enumerate((0, .5, 1, 2, 4, 8, 16, 32)):
            with self.subTest(cdt=code):
                value = mpr.decode(fixture(**{"0x5D": code << 5, "0x6C": 0, "0x5F": 0}))["electrodes"][0]
                self.assertEqual(value["effective_cdt_us"], duration)

    def test_all_twelve_encoding_and_proximity(self):
        output = mpr.decode(fixture(**{"0x5E": 0x3F}))
        self.assertEqual(output["ecr"]["electrode_count"], 12)
        self.assertEqual(output["ecr"]["proximity_code"], 3)
        self.assertTrue(all(e["enabled"] for e in output["electrodes"]))

    def test_stop_mode(self):
        output = mpr.decode(fixture(**{"0x5E": 0}))
        self.assertFalse(output["ecr"]["run_mode"])
        self.assertFalse(any(e["enabled"] for e in output["electrodes"]))

    def test_baseline_precision_is_not_exact(self):
        electrode = mpr.decode(fixture(**{"0x04": 0xBC, "0x05": 2, "0x1E": 176}))["electrodes"][0]
        self.assertEqual(electrode["filtered_counts"], 700)
        self.assertEqual(electrode["visible_baseline_floor_counts"], 704)
        self.assertEqual(electrode["internal_delta_range_if_same_instant"], [4, 7])

    def test_reserved_filtered_bits_discarded(self):
        electrode = mpr.decode(fixture(**{"0x04": 0, "0x05": 4, "0x1E": 176}))["electrodes"][0]
        self.assertIsNone(electrode["filtered_counts"])
        self.assertIsNone(electrode["internal_delta_range_if_same_instant"])

    def test_origin_never_becomes_readback_by_decoding(self):
        source = fixture()
        source["origin"] = "source-intent"
        decoded = mpr.decode(source)
        self.assertEqual(decoded["origin"], "source-intent")
        self.assertIn("this input is not a physical register readback", decoded["warnings"])

    def test_invalid_input_rejected(self):
        for input_doc in (fixture(**{"0x5F": True}), fixture(**{"0x5F": -1}), fixture(**{"0x5F": 256}),
                          fixture(**{"0x5F": "16"}), fixture(**{"0x80": 0}), fixture(**{"0x5c": 16}),
                          {**fixture(), "origin": "unknown"}, {**fixture(), "schema": "other"},
                          {**fixture(), "registers": {"0x5D": 40, "0x5E": 204}}):
            with self.subTest(document=input_doc):
                with self.assertRaises(ValueError):
                    mpr.decode(input_doc)


class HistoricalEvidence(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.frozen = json.loads(MANIFEST_PATH.read_text(encoding="utf-8-sig"))

    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(prefix="round90f_")
        self.addCleanup(self.tmp.cleanup)
        self.data = Path(self.tmp.name)
        self.manifest = copy.deepcopy(self.frozen)
        for relative in self.manifest["files"]:
            target = self.data / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(DATA_ROOT / relative, target)

    def replace(self, relative, payload):
        (self.data / relative).write_bytes(payload)
        self.manifest["files"][relative] = hashlib.sha256(payload).hexdigest()

    def alter_csv(self, edit):
        relative = self.manifest["jobs"][0]["id"] + ".csv"
        rows = list(csv.DictReader(io.StringIO((self.data / relative).read_text(encoding="utf-8"))))
        fields = list(rows[0])
        edit(rows)
        stream = io.StringIO(newline="")
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
        self.replace(relative, stream.getvalue().encode("utf-8"))

    def test_historical_result_and_exact_collisions(self):
        result = mbr.audit(self.manifest, self.data)
        self.assertEqual((result["input_files_verified"], result["csv_rows"], result["error_rows_excluded"],
                          result["hold_observations"]), (44, 722, 21, 175))
        self.assertEqual(result["unsaturated_phase_valid_diff_over_delta"]["n"], 115)
        jobs = {job["mode"]: job for job in result["jobs"]}
        self.assertEqual(jobs["finger_contact"]["hold_diff_255_observations"], 40)
        self.assertEqual(jobs["palm_contact"]["clipped_hold_delta"]["min"], 226)
        self.assertEqual(jobs["palm_contact"]["clipped_hold_delta"]["max"], 541)
        collisions = result["exact_opposite_label_collisions"]["diff+delta+cp_pf"]
        self.assertEqual(collisions["distinct_opposite_label_keys"], 3)
        for pair in collisions["examples"]:
            self.assertIn("contact", pair["contact"]["mode"])
            self.assertIn("hover", pair["hover"]["mode"])
            self.assertEqual([pair["contact"][field] for field in ("diff", "delta", "cp_pf")], pair["values"])
            self.assertEqual([pair["hover"][field] for field in ("diff", "delta", "cp_pf")], pair["values"])
        full = result["exact_opposite_label_collisions"]["diff+raw+baseline+cp_pf"]
        self.assertEqual(full["distinct_opposite_label_keys"], 3)
        for pair in full["examples"]:
            self.assertEqual([pair["contact"][field] for field in ("diff", "raw", "baseline", "cp_pf")], pair["values"])
            self.assertEqual([pair["hover"][field] for field in ("diff", "raw", "baseline", "cp_pf")], pair["values"])

    def test_corrupted_bytes_rejected(self):
        relative = self.manifest["jobs"][0]["id"] + ".csv"
        with (self.data / relative).open("ab") as stream:
            stream.write(b"corruption")
        with self.assertRaisesRegex(ValueError, "SHA-256"):
            mbr.audit(self.manifest, self.data)

    def test_whole_action_confirmation_required(self):
        relative = self.manifest["jobs"][0]["id"] + ".json"
        meta = json.loads((self.data / relative).read_text(encoding="utf-8"))
        meta["user_confirmed"] = "true"
        self.replace(relative, json.dumps(meta).encode("utf-8"))
        with self.assertRaisesRegex(ValueError, "unconfirmed"):
            mbr.audit(self.manifest, self.data)

    def test_arithmetic_rejected_even_with_updated_hash(self):
        def edit(rows):
            row = next(row for row in rows if not row["error"] and row["phase_valid"] == "1")
            row["delta"] = str(int(row["delta"]) + 1)
        self.alter_csv(edit)
        with self.assertRaisesRegex(ValueError, "arithmetic"):
            mbr.audit(self.manifest, self.data)

    def test_window_retuning_rejected(self):
        self.manifest["hold_window_s"] = [6.5, 11.0]
        with self.assertRaisesRegex(ValueError, "retuned"):
            mbr.audit(self.manifest, self.data)

    def test_error_partial_fields_are_excluded(self):
        def edit(rows):
            row = next(row for row in rows if row["error"])
            for field in ("sync", "cp_pf", "diff", "baseline", "raw", "delta", "ref_raw", "ref_delta"):
                row[field] = "unavailable"
        self.alter_csv(edit)
        result = mbr.audit(self.manifest, self.data)
        self.assertEqual(result["hold_observations"], 175)
        self.assertEqual(result["error_rows_excluded"], 21)

    def test_restored_config_mismatch_rejected(self):
        relative = self.manifest["jobs"][0]["id"] + "_device/after_reboot/chip_40.bin"
        payload = bytearray((self.data / relative).read_bytes())
        payload[20] ^= 1
        self.replace(relative, bytes(payload))
        with self.assertRaisesRegex(ValueError, "configuration"):
            mbr.audit(self.manifest, self.data)

    def test_nonfinite_time_rejected(self):
        self.alter_csv(lambda rows: rows[0].__setitem__("elapsed_s", "nan"))
        with self.assertRaisesRegex(ValueError, "non-finite"):
            mbr.audit(self.manifest, self.data)

    def test_path_outside_evidence_root_rejected(self):
        self.manifest["files"]["../outside"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "leaves data root"):
            mbr.audit(self.manifest, self.data)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", type=Path, default=MANIFEST_PATH)
    parser.add_argument("--data-root", type=Path, default=DATA_ROOT)
    args = parser.parse_args()
    MANIFEST_PATH, DATA_ROOT = args.manifest, args.data_root
    unittest.main(argv=[sys.argv[0]], verbosity=2)
