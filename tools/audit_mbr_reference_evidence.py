#!/usr/bin/env python3
"""Read the frozen round90g package to audit generic sensing advice.

Offline only: validates saved bytes against the committed package hashes.
Does not connect to hardware or construct new device configuration tables.
"""
import hashlib
import json
from pathlib import Path
import zipfile


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def main():
    variant = Path(__file__).resolve().parents[1]
    package = variant / 'docs/MBR3116_round90g_MPR证据.zip'
    check = json.loads(package.with_suffix('.package_check.json').read_text(encoding='utf-8'))
    if sha256(package.read_bytes()) != check['sha256']:
        raise ValueError('Frozen evidence package hash changed')
    with zipfile.ZipFile(package) as archive:
        manifest = json.loads(archive.read('manifest.json'))

        def verified(name):
            data = archive.read(name)
            expected = manifest['files'][name]
            if sha256(data) != expected['sha256'] or len(data) != expected['size']:
                raise ValueError('Evidence member mismatch: ' + name)
            return data

        prefix = '_dev_archive/round90g_before_20261003/'
        snapshot = json.loads(verified(prefix + 'checked.json'))
        controller = verified(prefix + 'controller_config.bin')
        if len(controller) != 128 or sha256(controller) != snapshot['verified_sha256']['controller_config.bin']:
            raise ValueError('Controller backup does not match saved readback')
        result = {
            'origin': 'offline review of frozen pre-migration evidence; not a new device read',
            'usb_serial': snapshot['usb_serial'], 'version': snapshot['version'],
            'package_sha256': check['sha256'], 'controller_sha256': sha256(controller),
            'raw_status_when_backed_up': snapshot['raw_status'],
            'game_raw_enabled_in_saved_cfg2': bool(controller[5] & 0x02),
            'distance_profile_saved': snapshot['profile'], 'chips': [],
            'decode_basis': 'Infineon CY8CMBR3xxx Registers TRM 001-91082 Rev E; existing bytes only',
        }
        for address in (0x40, 0x41, 0x42):
            name = f'chip_{address:02x}.bin'
            data = verified(prefix + name)
            if (len(data) != 128 or data[0x51] != address
                    or sha256(data) != snapshot['verified_sha256'][name]):
                raise ValueError('Chip table length, address or readback hash mismatch')
            enabled_mask = int.from_bytes(data[:2], 'little')
            enabled = [e for e in range(16) if enabled_mask & (1 << e)]
            codes = [(data[0x08 + e // 4] >> (2 * (e % 4))) & 3 for e in enabled]
            result['chips'].append({
                'address': hex(address), 'sha256': sha256(data),
                'enabled_electrodes': enabled, 'prox_en_0x26': data[0x26],
                'sensitivity_codes_enabled': codes,
                'sensitivity_parameter_fF_enabled': [(100, 200, 300, 400)[code] for code in codes],
                'stored_finger_threshold_enabled': [data[0x0C + e] for e in enabled],
                'automatic_threshold_enabled': bool(data[0x4F] & 0x08),
                'stored_hysteresis_override_enabled': bool(data[0x1D] & 0x80),
                'effective_automatic_threshold_and_hysteresis': None,
                'automatic_parameters_not_inferred_from_stored_manual_fields': True,
            })
        page = json.loads(verified('_dev_tools/round90g_mpr_20261003/page_analysis.json'))
        result['mpr27_actions'] = []
        for job in page['jobs']:
            window = job['windows']['hold_late']
            electrode = window['channels'][8]
            result['mpr27_actions'].append({
                'mode': job['mode'], 'total_observations': job['samples'],
                'sampled_chip': '0x5A only', 'window_s': [9, 11.5],
                'observations': window['n'], 'ele8_native_on': electrode['native_on'],
                'ele8_baseline_floor_range': [electrode['baseline_min'], electrode['baseline_max']],
                'ele8_delta_interval_extent': [electrode['delta_low_min'], electrode['delta_high_max']],
                'independent_physical_contact_times_or_hid_validation': False,
            })
    result['source_sha256'] = {
        name: sha256((variant / name).read_bytes())
        for name in ('src/hw_devices.cpp', 'src/chuni_io.cpp', 'tools/audit_mbr_reference_evidence.py')}
    destination = variant / 'docs/MBR3116_round90h_核查.json'
    destination.write_text(json.dumps(result, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print('Frozen package and 4 saved configurations verified; no hardware access')
    for chip in result['chips']:
        print(chip['address'], 'PROX_EN', chip['prox_en_0x26'],
              'sensitivity_fF', sorted(set(chip['sensitivity_parameter_fF_enabled'])),
              'stored_FT', sorted(set(chip['stored_finger_threshold_enabled'])),
              'ATH', chip['automatic_threshold_enabled'])
    print('MPR saved actions:', len(result['mpr27_actions']),
          'observations:', sum(j['total_observations'] for j in result['mpr27_actions']))


if __name__ == '__main__':
    main()
