#!/usr/bin/env python3
"""Offline screening of touch mitigations; never connects to or configures hardware.

replay reuses the hash-pinned round90b parser and its frozen source mapping.
margin evaluates integer signal cutoffs, not register values or a firmware policy.
Neither command establishes physical contact times or deployment readiness.
"""
import argparse
from fractions import Fraction
import hashlib
import importlib.util
import io
import json
from pathlib import Path, PurePosixPath
import tempfile
import zipfile

REPO = Path(__file__).resolve().parents[1]
PACKAGE_SHA256 = "ef3c14945c4900537711ba2586c63a0a8599917cc532fe716a0b6773520c4e29"
PARSER_PATH = "_dev_tools/round90b_analyze.py"
PARSER_SHA256 = "49d45c10f02d9bc5e641d294816f80b4d7d70c23e88b9deaf68b321480b9e8b5"
SOURCE_PATH = "Nyanithm_firmware_hw_v1/src/hw_devices.cpp"
SOURCE_SHA256 = "5d4839b14390a37eda74fc4a5ae40d585073686de7dddea9c28247e918eab0c8"
FEATURES = ("count_at_onset", "delta_15ms_per_ms", "delta_30ms_per_ms",
            "delta_60ms_per_ms", "max_step_count_per_ms", "curvature_same_row")
ANCILLARY = ("lanes_above128", "lanes_saturated255")


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def verified_package(package):
    data = package.read_bytes()
    if sha256(data) != PACKAGE_SHA256:
        raise ValueError("not the frozen round90b package")
    with zipfile.ZipFile(io.BytesIO(data)) as archive:
        names = archive.namelist()
        if len(names) != len(set(names)) or archive.testzip() is not None:
            raise ValueError("duplicate member or ZIP CRC failure")
        for name in names:
            path = PurePosixPath(name)
            if path.is_absolute() or ".." in path.parts or "\\" in name or ":" in name:
                raise ValueError("unsafe archive member")
        manifest = json.loads(archive.read("manifest.json"))
        blobs = {}
        for item in manifest["files"]:
            name = item["path"]
            if name in blobs:
                raise ValueError("duplicate manifest member")
            payload = archive.read(name)
            if len(payload) != item["size"] or sha256(payload) != item["sha256"]:
                raise ValueError("frozen member mismatch: " + name)
            blobs[name] = payload
        if set(names) != set(blobs) | {"manifest.json", "REPLAY.txt"}:
            raise ValueError("unexpected package members")
    if sha256(blobs[PARSER_PATH]) != PARSER_SHA256 or sha256(blobs[SOURCE_PATH]) != SOURCE_SHA256:
        raise ValueError("frozen parser/source changed")
    return manifest, blobs


def causal_value(trial, feature):
    values = trial["features"]
    if feature.startswith("delta_") and feature.endswith("_per_ms"):
        prefix = feature[:-7]
        return Fraction(values[prefix], values[prefix + "_actual_dt"])
    return Fraction(str(values[feature]))


def identity(trial):
    return {key: trial[key] for key in ("file", "mode", "repeat", "session")}


def dominance_screen(trials, features):
    """A contact <= hover in every ON-positive feature obstructs monotone rules."""
    complete = [trial for trial in trials if all(key in trial["features"] for key in features)]
    contacts = [trial for trial in complete if "hover" not in trial["mode"]]
    hovers = [trial for trial in complete if "hover" in trial["mode"]]
    witnesses = []
    for contact in contacts:
        for hover in hovers:
            if all(causal_value(contact, key) <= causal_value(hover, key) for key in features):
                witnesses.append({
                    "contact_action": identity(contact), "hover_action": identity(hover),
                    "contact_features": {key: contact["features"][key] for key in features},
                    "hover_features": {key: hover["features"][key] for key in features},
                    "same_ancillary": {key: contact["features"][key] for key in ANCILLARY
                                       if key in contact["features"] and
                                       contact["features"][key] == hover["features"].get(key)},
                })
    return {
        "features_larger_means_more_likely_ON": list(features),
        "contact_action_onsets": len(contacts), "hover_action_onsets": len(hovers),
        "unassessable_trials": len(trials) - len(complete),
        "verdict": "obstructed" if witnesses else "no_obstruction_found_not_validated",
        "dominance_witnesses": witnesses,
        "scope": "preserve every observed first final ON in contact-labelled actions and reject every hover-labelled onset",
    }


def replay(package):
    if not __debug__:
        raise ValueError("frozen parser requires assertions; do not use Python -O")
    manifest, blobs = verified_package(package)
    trials, excluded = [], []
    folders = ("round90b_events_20261002", "round90b_sanity_20261002", "round90b_stress_20261002")
    # Reuse the original parser rather than invent a second trace decoder. Only
    # this exact known source is executed; all other archive scripts are data.
    with tempfile.TemporaryDirectory(prefix="touch_separation_") as temporary:
        root = Path(temporary)
        for name in (PARSER_PATH, SOURCE_PATH):
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(blobs[name])
        spec = importlib.util.spec_from_file_location("frozen_round90b", root / PARSER_PATH)
        parser = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(parser)
        for name, payload in sorted(blobs.items()):
            if not any(name.startswith("_dev_tools/" + folder + "/") for folder in folders) or not name.endswith(".json"):
                continue
            job = json.loads(payload)
            if "captures" not in job or "mode" not in job:
                continue
            if not job["completed"] or job["user_confirmed"] is not True:
                excluded.append({"file": name, "reason": job.get("error"), "captures": len(job["captures"])})
                continue
            target = root / name
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(payload)
            trials.extend(parser.analyze_job(target))

    bounds = {}
    for feature in FEATURES:
        contacts = [t for t in trials if "hover" not in t["mode"] and feature in t["features"]]
        hovers = [t for t in trials if "hover" in t["mode"] and feature in t["features"]]
        if contacts and hovers:
            contact_min = min(causal_value(t, feature) for t in contacts)
            hover_max = max(causal_value(t, feature) for t in hovers)
            bounds[feature] = {"contact_min": float(contact_min), "hover_max": float(hover_max),
                               "feasible_lower_bound_gate": hover_max < contact_min,
                               "requirement": "hover_max < cutoff <= contact_min"}
    return {
        "schema": "touch-separation-replay-v1", "hardware_access": False,
        "package_sha256": PACKAGE_SHA256, "verified_evidence_files": len(blobs),
        "frozen_firmware_commit": manifest["firmware_commit"],
        "frozen_parser_sha256": PARSER_SHA256, "frozen_mapping_source_sha256": SOURCE_SHA256,
        "confirmed_trials": len(trials), "frames": sum(t["frames"] for t in trials),
        "excluded_jobs": excluded, "single_feature_cutoff_bounds": bounds,
        "combinations": [dominance_screen(trials, FEATURES[:2]), dominance_screen(trials, FEATURES)],
        "trials": [{key: trial[key] for key in
                    ("file", "mode", "repeat", "session", "frames", "pre_ms", "post_ms",
                     "all_lanes_qualified", "target_qualified", "target_onset_index",
                     "feature_reason", "features")} for trial in trials],
        "limits": [
            "historical MBR/profile-ON evidence; not a current device read or MPR/MBR same-board comparison",
            "contact labels cover whole actions, not physical contact at the first ON",
            "larger-means-ON monotone rules only; not a proof against arbitrary history or new inputs",
            "combining already-viewed evidence is a falsification screen, not independent validation",
            "missing onset/quality is unassessable, never successful hover suppression",
            "host read/publish cadence is not independent chip sampling or mechanical speed",
        ],
    }


def integer(value, name, minimum, maximum):
    if type(value) is not int or not minimum <= value <= maximum:
        raise ValueError(name + " must be an integer in the supported range")
    return value


def interval(value, name, maximum):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(name + " must be [lower, upper]")
    low, high = (integer(v, name, -1023, maximum) for v in value)
    if low > high:
        raise ValueError(name + " has reversed bounds")
    return low, high


def margin_screen(document):
    if document.get("schema") != "touch-signal-margin-v1":
        raise ValueError("unsupported margin schema")
    if document.get("origin") not in ("user-observation", "capture-summary", "synthetic-example"):
        raise ValueError("explicit input origin required")
    strata = document.get("strata")
    if not isinstance(strata, list) or not strata:
        raise ValueError("at least one board/firmware/configuration/lane stratum required")
    outputs = []
    for stratum in strata:
        chip = stratum.get("chip")
        maximum = {"MPR121": 1023, "MBR3116": 255}.get(chip)
        if maximum is None:
            raise ValueError("unsupported chip")
        context = ("board_id", "firmware", "config_sha256", "lane", "scale", "quality")
        reasons = ["missing " + key for key in context if stratum.get(key) is None]
        scale = {"MPR121": "native-baseline-minus-filtered", "MBR3116": "native-button-diff"}[chip]
        if stratum.get("scale") != scale:
            reasons.append("native scale unconfirmed; do not convert a display value by assumption")
        if stratum.get("quality") != "valid-fresh-coherent":
            reasons.append("valid/fresh/coherent capture quality unconfirmed")
        config = stratum.get("config_sha256")
        if config is not None and (not isinstance(config, str) or len(config) != 64 or
                                   any(c not in "0123456789abcdef" for c in config)):
            raise ValueError("invalid configuration hash")
        if stratum.get("lane") is not None:
            integer(stratum["lane"], "lane", 0, 31)
        for key in ("board_id", "firmware"):
            if stratum.get(key) is not None and (not isinstance(stratum[key], str) or not stratum[key].strip()):
                raise ValueError(key + " must be nonempty text")
        quantities = ("idle_max", "hover_max", "contact_onset_min", "contact_hold_min", "released_max")
        values = {}
        for key in quantities:
            if stratum.get(key) is None:
                reasons.append("missing " + key)
            else:
                values[key] = interval(stratum[key], key, maximum)
                if chip == "MBR3116" and values[key][0] < 0:
                    raise ValueError("native button DIFF cannot be negative")
        margin = stratum.get("drift_margin_counts")
        if margin is None:
            reasons.append("drift margin not chosen")
        else:
            integer(margin, "drift margin", 0, maximum)
        result = {"chip": chip, "board_id": stratum.get("board_id"), "lane": stratum.get("lane"),
                  "missing_or_unconfirmed": reasons, "verdict": "unassessable"}
        if not reasons:
            # Abstract integer cutoff: ON if signal >= on; release if signal < off.
            # Lower ends protect contacts; upper ends protect hover and release.
            on_low = max(values["idle_max"][1], values["hover_max"][1]) + margin + 1
            on_high = min(maximum, values["contact_onset_min"][0] - margin)
            off_low = max(values["idle_max"][1], values["released_max"][1]) + margin + 1
            off_high = min(values["contact_hold_min"][0] - margin, on_high - 1)
            feasible = on_low <= on_high and off_low <= off_high
            result.update({"verdict": "signal_interval_exists_not_validated" if feasible else "no_robust_cutoff_interval",
                           "abstract_ON_cutoff_range": [on_low, on_high] if on_low <= on_high else None,
                           "abstract_release_cutoff_range": [off_low, off_high] if off_low <= off_high else None,
                           "cutoff_pair_constraint": "release_cutoff < ON_cutoff",
                           "ON_margin_gap_counts": on_high - on_low,
                           "hold_release_margin_gap_counts": off_high - off_low})
        outputs.append(result)
    return {"schema": "touch-signal-margin-screen-v1", "origin": document["origin"],
            "hardware_access": False, "deployment_ready": False, "strata": outputs,
            "limits": ["input summaries and quality attestations are not verified raw captures",
                       "integer signal cutoffs are not hardware register settings; native comparisons and firmware tiers differ",
                       "an interval requires causal production replay and independent slow-touch/multi-press/slide/HID acceptance",
                       "MPR baseline high-eight-bit uncertainty belongs in the signal interval, not an exact delta",
                       "different boards, chips, firmware, configuration and lanes must be separate strata"]}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subcommands = parser.add_subparsers(dest="command", required=True)
    historical = subcommands.add_parser("replay")
    historical.add_argument("--package", type=Path, default=REPO / "docs/MBR3116_round90b_事件判据证据包.zip")
    historical.add_argument("--out", type=Path)
    margins = subcommands.add_parser("margin")
    margins.add_argument("input", type=Path)
    margins.add_argument("--out", type=Path)
    args = parser.parse_args()
    source = args.package if args.command == "replay" else args.input
    if args.out and args.out.resolve() == source.resolve():
        parser.error("output must not overwrite the input evidence")
    result = replay(args.package) if args.command == "replay" else margin_screen(json.loads(args.input.read_text(encoding="utf-8-sig")))
    result["tool_sha256"] = sha256(Path(__file__).read_bytes())
    payload = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.write_text(payload, encoding="utf-8")
        print(json.dumps({key: value for key, value in result.items() if key not in ("trials", "combinations")}, ensure_ascii=False, indent=2))
    else:
        print(payload, end="")


if __name__ == "__main__":
    main()
