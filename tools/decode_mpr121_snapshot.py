"""Decode saved MPR121 register bytes using NXP MPR121 Rev. 4, sections 5.7-5.12.

Offline only. No device access, register writes, or defaults for missing bytes.
Input: {schema, origin, registers}; addresses are strings such as "0x5C".
Origin must distinguish hardware-readback, source-intent, and synthetic-test.
Sparse input does not establish signal coherence or the actual device identity.
"""

import argparse
import json
import re
from pathlib import Path

SCHEMA = "mpr121-register-snapshot-v1"
ORIGINS = ("hardware-readback", "source-intent", "synthetic-test")
FFI_SAMPLES = (6, 10, 18, 34)
SFI_SAMPLES = (4, 6, 10, 18)
CL_MEANING = (
    "tracking enabled; initial baseline uses current baseline register",
    "baseline tracking disabled",
    "tracking enabled; initial baseline uses high 5 bits of first electrode data",
    "tracking enabled; initial baseline uses all 10 bits of first electrode data",
)
SOURCE = "https://www.nxp.com/docs/en/data-sheet/MPR121.pdf"


def decode(document):
    if document.get("schema") != SCHEMA or document.get("origin") not in ORIGINS:
        raise ValueError("explicit supported schema and origin required")
    registers = document.get("registers")
    if not isinstance(registers, dict):
        raise ValueError("registers must be an address-to-byte object")
    values = {}
    for address, value in registers.items():
        if not isinstance(address, str) or not re.fullmatch(r"0x[0-9a-fA-F]{2}", address):
            raise ValueError("address must have the form 0xHH")
        index = int(address, 16)
        if index > 0x7F or index in values:
            raise ValueError("duplicate or unsupported register address")
        if type(value) is not int or not 0 <= value <= 255:
            raise ValueError("register values must be integer bytes")
        values[index] = value
    if any(address not in values for address in (0x5C, 0x5D, 0x5E)):
        raise ValueError("CONFIG1, CONFIG2, and ECR bytes required")

    config1, config2, ecr = (values[address] for address in (0x5C, 0x5D, 0x5E))
    global_cdc = config1 & 0x3F
    global_cdt_code = config2 >> 5
    sfi = SFI_SAMPLES[(config2 >> 3) & 3]
    esi = 1 << (config2 & 7)
    cl = ecr >> 6
    electrode_count = min(ecr & 15, 12)  # ELE_EN=11xx means all 12.
    proximity_code = (ecr >> 4) & 3
    debounce = values.get(0x5B)
    autoconfig0, autoconfig1 = values.get(0x7B), values.get(0x7C)
    warnings = [
        "origin is supplied by the input; device identity and read success are not verified",
        "sparse bytes do not establish a coherent 0x00..0x2A signal read",
        "nominal update period excludes scan overrun and is not touch-to-HID latency",
    ]
    if document["origin"] != "hardware-readback":
        warnings.append("this input is not a physical register readback")

    electrodes = []
    for electrode in range(13):
        cdc_byte = values.get(0x5F + electrode)
        cdt_byte = values.get(0x6C + electrode // 2)
        cdc = None if cdc_byte is None else cdc_byte & 0x3F
        cdt_code = None if cdt_byte is None else (cdt_byte >> (4 * (electrode % 2))) & 7
        effective_cdc = None if cdc is None else (cdc or global_cdc)
        effective_cdt_code = None if cdt_code is None else (cdt_code or global_cdt_code)
        effective_cdt = (None if effective_cdt_code is None else
                         (0.0 if effective_cdt_code == 0 else 2.0 ** (effective_cdt_code - 2)))
        low, high = values.get(0x04 + 2 * electrode), values.get(0x05 + 2 * electrode)
        filtered = None if low is None or high is None else low + ((high & 3) << 8)
        if high is not None and high & ~3:
            warnings.append(f"ELE{electrode} filtered high byte has reserved bits set; data discarded")
            filtered = None
        baseline = values.get(0x1E + electrode)
        floor = None if baseline is None else baseline << 2
        delta_range = None if floor is None or filtered is None else [floor - filtered, floor + 3 - filtered]
        electrodes.append({
            "electrode": electrode,
            "enabled": electrode < electrode_count if electrode < 12 else proximity_code != 0,
            "individual_cdc_code": cdc,
            "individual_cdt_code": cdt_code,
            "effective_cdc_uA": effective_cdc,
            "effective_cdt_us": effective_cdt,
            "charge_parameters_complete": effective_cdc is not None and effective_cdt is not None,
            "filtered_counts": filtered,
            "baseline_high8": baseline,
            "visible_baseline_floor_counts": floor,
            "internal_delta_range_if_same_instant": delta_range,
            "touch_threshold": values.get(0x41 + 2 * electrode),
            "release_threshold": values.get(0x42 + 2 * electrode),
        })
    if any(e["enabled"] and not e["charge_parameters_complete"] for e in electrodes):
        warnings.append("missing individual charge registers; global settings alone are insufficient")

    return {
        "schema": "mpr121-register-decoding-v1", "origin": document["origin"],
        "source": SOURCE, "register_count": len(values), "warnings": warnings,
        "global": {"ffi_samples": FFI_SAMPLES[config1 >> 6], "cdc_uA": global_cdc,
                   "cdt_us": 0.0 if global_cdt_code == 0 else 2.0 ** (global_cdt_code - 2),
                   "sfi_samples": sfi, "esi_ms": esi,
                   "nominal_output_update_ms": sfi * esi},
        "ecr": {"cl_code": cl, "cl_meaning": CL_MEANING[cl],
                "tracking_enabled": cl != 1, "electrode_count": electrode_count,
                "proximity_code": proximity_code, "run_mode": electrode_count > 0 or proximity_code > 0},
        "debounce": None if debounce is None else {"dt": debounce & 7, "dr": (debounce >> 4) & 7},
        "autoconfig": {"ace": None if autoconfig0 is None else bool(autoconfig0 & 1),
                       "are": None if autoconfig0 is None else bool(autoconfig0 & 2),
                       "bva_code": None if autoconfig0 is None else (autoconfig0 >> 2) & 3,
                       "retry_code": None if autoconfig0 is None else (autoconfig0 >> 4) & 3,
                       "scts": None if autoconfig1 is None else bool(autoconfig1 & 0x80)},
        "baseline_filters": {f"0x{address:02X}": values.get(address) for address in range(0x2B, 0x36)},
        "electrodes": electrodes,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("snapshot", type=Path)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args()
    output = decode(json.loads(args.snapshot.read_text(encoding="utf-8-sig")))
    payload = json.dumps(output, ensure_ascii=False, indent=2) + "\n"
    if args.out:
        args.out.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")


if __name__ == "__main__":
    main()
