"""Audit saved sequential C7 button/proximity records without accessing hardware.

No threshold fitting or contact classifier. Preserve errors and phase exclusions.
The manifest freezes inputs and the preexisting 6..11.5 s hold window.
"""

import argparse
import csv
import hashlib
import io
import json
import math
import statistics
from pathlib import Path

SCHEMA = "mbr-native-raw-evidence-manifest-v1"
REFERENCE_MODE = "0x42 CS0 Proximity 16-bit ALP OFF ARST OFF; center 0x40 CS7 button; sequential native C7"
MODES = ("finger_contact", "finger_hover3", "palm_hover5", "palm_contact")


def summary(values):
    values = list(values)
    return {"n": len(values), "min": min(values), "median": statistics.median(values), "max": max(values)} if values else {"n": 0}


def load_inputs(manifest, data_root):
    if manifest.get("schema") != SCHEMA or manifest.get("reference_mode") != REFERENCE_MODE:
        raise ValueError("unsupported manifest or reference mode")
    if manifest.get("hold_window_s") != [6.0, 11.5]:
        raise ValueError("preexisting hold window must not be retuned")
    entries = manifest.get("files")
    if not isinstance(entries, dict) or not entries:
        raise ValueError("input hashes required")
    data_root = data_root.resolve()
    blobs = {}
    for relative, digest in entries.items():
        path = (data_root / relative).resolve()
        if not path.is_relative_to(data_root):
            raise ValueError("input path leaves data root")
        payload = path.read_bytes()
        if hashlib.sha256(payload).hexdigest() != digest:
            raise ValueError(f"SHA-256 disagreement: {relative}")
        blobs[relative] = payload
    return blobs


def audit(manifest, data_root):
    blobs = load_inputs(manifest, data_root)
    jobs = manifest.get("jobs", [])
    if len(jobs) != 4 or {j.get("mode") for j in jobs} != set(MODES):
        raise ValueError("four distinct confirmed historical modes required")
    points, result_jobs, common_config = {}, [], None
    ratios = []
    error_rows = phase_excluded_rows = total_rows = 0
    for spec in jobs:
        job_id, mode = spec["id"], spec["mode"]
        meta = json.loads(blobs[job_id + ".json"])
        if meta.get("id") != job_id or meta.get("mode") != mode:
            raise ValueError("job identity disagreement")
        if any(meta.get(key) is not True for key in ("completed", "user_confirmed", "capture_started")):
            raise ValueError("unconfirmed, aborted, or unstarted trial")
        hardware = meta.get("hardware", {})
        if meta.get("reference_mode") != REFERENCE_MODE or any(hardware.get(key) is not True for key in
                ("programming_attempted", "candidate_applied", "restore_verified", "restore_after_reboot")):
            raise ValueError("candidate mode or restoration evidence missing")
        if hardware.get("usb_serial") != manifest.get("usb_serial") or meta.get("firmware") != manifest.get("firmware"):
            raise ValueError("device or firmware stratum disagreement")

        device = job_id + "_device/"
        config = []
        for filename in ("controller_config.bin", "chip_40.bin", "chip_41.bin", "chip_42.bin"):
            original = blobs[device + filename]
            restored = blobs[device + "after_reboot/" + filename]
            if len(original) != 128 or original != restored:
                raise ValueError("backup/restored configuration not equal or not 128 bytes")
            config.append(hashlib.sha256(original).hexdigest())
        if common_config is not None and config != common_config:
            raise ValueError("trials do not share the same backup configuration")
        common_config = config
        snapshot = json.loads(blobs[device + "after_reboot/snapshot.json"])
        if (snapshot.get("usb_serial") != manifest["usb_serial"] or snapshot.get("version") != manifest["firmware"] or
                snapshot.get("raw_status") != "RAW=0" or len(snapshot.get("initial_input", [])) != 33 or
                any(snapshot["initial_input"][:32])):
            raise ValueError("restored snapshot identity or state disagreement")
        if bytes.fromhex(snapshot["controller_config"]) != blobs[device + "controller_config.bin"]:
            raise ValueError("snapshot controller bytes disagree")
        for address in (0x40, 0x41, 0x42):
            if bytes.fromhex(snapshot["chips"][f"0x{address:02x}"]["hex"]) != blobs[device + f"chip_{address:02x}.bin"]:
                raise ValueError("snapshot chip bytes disagree")

        rows = list(csv.DictReader(io.StringIO(blobs[job_id + ".csv"].decode("utf-8-sig"))))
        total_rows += len(rows)
        hold, unsaturated, clipped = [], [], []
        previous_time = -math.inf
        for line, row in enumerate(rows, start=2):
            elapsed = float(row["elapsed_s"])
            if not math.isfinite(elapsed) or elapsed <= previous_time:
                raise ValueError("non-finite or non-increasing row time")
            previous_time = elapsed
            if row["phase_valid"] not in ("0", "1") or row["phase"] not in ("baseline", "hold", "release"):
                raise ValueError("invalid phase flags")
            # Errors may contain partial numbers. Never decode or zero-fill them.
            if row["error"]:
                error_rows += 1
                continue
            if row["phase_valid"] != "1":
                phase_excluded_rows += 1
            sample = {"csv_line": line, "elapsed_s": elapsed, "phase": row["phase"]}
            limits = {"sync": 15, "cp_pf": 255, "diff": 255, "raw": 65535, "baseline": 65535,
                      "ref_sync": 15, "ref_cp_pf": 255, "ref_diff": 65535, "ref_raw": 65535, "ref_baseline": 65535}
            for field, maximum in limits.items():
                value = int(row[field])
                if not 0 <= value <= maximum:
                    raise ValueError(f"out-of-range {field} in {job_id}:{line}")
                sample[field] = value
            for prefix in ("", "ref_"):
                sample[prefix + "delta"] = int(row[prefix + "delta"])
                if sample[prefix + "delta"] != sample[prefix + "raw"] - sample[prefix + "baseline"]:
                    raise ValueError(f"RAW-BASE arithmetic disagreement in {job_id}:{line}")
            if row["phase_valid"] != "1":
                continue
            if 0 < sample["diff"] < 255 and sample["delta"] > 0:
                unsaturated.append(sample)
                ratios.append(sample["diff"] / sample["delta"])
            if sample["phase"] == "hold" and 6.0 <= elapsed <= 11.5:
                hold.append(sample)
                if sample["diff"] == 255:
                    clipped.append(sample)
        if not hold:
            raise ValueError("no usable fixed hold records")
        points[mode] = hold
        result_jobs.append({
            "id": job_id, "mode": mode, "csv_rows": len(rows),
            "hold_valid_observations": len(hold), "hold_diff_255_observations": len(clipped),
            "hold": {field: summary(s[field] for s in hold) for field in
                     ("diff", "delta", "raw", "baseline", "cp_pf", "ref_delta", "ref_cp_pf")},
            "clipped_hold_delta": summary(s["delta"] for s in clipped),
            "clipped_hold_raw": summary(s["raw"] for s in clipped),
            "unsaturated_phase_valid_observations": len(unsaturated),
            "unsaturated_diff_over_delta": summary(s["diff"] / s["delta"] for s in unsaturated),
            "configuration_restore_equal": True,
            "mode_application_evidence": "historical metadata; not an in-capture NVM byte dump",
        })

    # Exact observed collisions; no fitted threshold and no hold-window training.
    collisions = {}
    for fields in (("delta",), ("diff", "delta"), ("diff", "delta", "cp_pf"), ("diff", "raw", "baseline", "cp_pf")):
        index = {}
        for mode, samples in points.items():
            for sample in samples:
                key = tuple(sample[field] for field in fields)
                index.setdefault(key, {}).setdefault(mode, sample)
        examples = []
        for key, by_mode in sorted(index.items()):
            contacts = sorted(mode for mode in by_mode if "contact" in mode)
            hovers = sorted(mode for mode in by_mode if "hover" in mode)
            if contacts and hovers:
                examples.append({"values": list(key), "contact": {"mode": contacts[0], **by_mode[contacts[0]]},
                                 "hover": {"mode": hovers[0], **by_mode[hovers[0]]}})
        collisions["+".join(fields)] = {"distinct_opposite_label_keys": len(examples), "examples": examples}

    return {
        "schema": "mbr-native-raw-observability-audit-v1", "hardware_access": False,
        "hold_window_s": manifest["hold_window_s"], "input_sha256": manifest["files"],
        "input_files_verified": len(blobs), "csv_rows": total_rows, "error_rows_excluded": error_rows,
        "non_error_phase_rows_excluded": phase_excluded_rows,
        "hold_observations": sum(len(samples) for samples in points.values()),
        "common_restored_config_sha256": dict(zip(("controller", "0x40", "0x41", "0x42"), common_config)),
        "jobs": result_jobs, "unsaturated_phase_valid_diff_over_delta": summary(ratios),
        "exact_opposite_label_collisions": collisions,
        "limits": ["one historical action per pose; observations are not independent touches",
                   "manual heights and whole-action confirmation do not label exact contact time",
                   "sequential C7 pairs are not simultaneous; no independent scan identity",
                   "temporary reference proximity mode is distinct from current production configuration",
                   "ratio is empirical at this center/configuration, not a universal normalization constant",
                   "overlap rejects a scalar classifier on these observations, not every causal algorithm"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--data-root", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8-sig"))
    output = audit(manifest, args.data_root or args.manifest.parent)
    payload = json.dumps(output, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")


if __name__ == "__main__":
    main()
