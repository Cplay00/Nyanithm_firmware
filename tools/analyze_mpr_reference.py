"""Offline reference-chain analysis of the frozen round90i / round90j packets.

No serial/device dependencies. Frame counts are observations, not independent
sensor scans or physical-touch success rates. Fixed windows predate this analysis.
"""
import argparse
from collections import Counter
import csv
import hashlib
import io
import json
from pathlib import Path
import statistics
import struct
import zipfile

DOCS = Path(__file__).resolve().parents[1] / 'docs'
WINDOWS = {'baseline': (.5, 3.5), 'hold': (6, 11.5), 'late': (9, 11.5), 'release': (14, 21.5)}
PACKETS = {
    'normal': ('MBR3116_round90j_普通模式三组实机证据.zip', 'da34280e77d351ee0920614c523c4330bf976f3e212daf3a69273f13038b82d4'),
    'native': ('MBR3116_round90i_同32寸六组证据.zip', 'b9228525840d1da1572df78cce5eda01642f6e607588a6f487079e7e28f26324'),
}


def require(condition, message):
    if not condition:
        raise ValueError(message)


def load_packet(path, expected_sha):
    blob = path.read_bytes()
    require(hashlib.sha256(blob).hexdigest() == expected_sha, 'packet hash mismatch')
    with zipfile.ZipFile(io.BytesIO(blob)) as archive:
        require(len(archive.namelist()) == len(set(archive.namelist())), 'duplicate packet member')
        require(archive.testzip() is None, 'packet CRC failure')
        files = {name: archive.read(name) for name in archive.namelist()}
    manifest = json.loads(files['manifest.json'])
    members = manifest.get('files', manifest.get('manifest'))
    require(isinstance(members, dict) and bool(members), 'missing member hash manifest')
    for name, value in members.items():
        expected = value['sha256'] if isinstance(value, dict) else value
        require(hashlib.sha256(files[name]).hexdigest() == expected, 'member hash mismatch: '+name)
    return files


def mapping_from(files):
    source = json.loads(files['source_mapping.json'])
    mapping = {(r['address'], r['electrode']): r['logical_lane'] for r in source['lanes']}
    require(len(mapping) == 32 and set(mapping.values()) == set(range(32)), 'invalid source mapping')
    require(all(a in (0x5A, 0x5B, 0x5C) and 0 <= e < 12 for a, e in mapping), 'invalid electrode')
    return mapping


def mapped(words, mapping):
    return sorted(lane for (a, e), lane in mapping.items() if words[a-0x5A] & (1 << e))


def decode_normal(row, mapping):
    chain = bytes.fromhex(row['chain_hex'])
    game = bytes.fromhex(row['game_hex'])
    require(len(chain) == 46 and chain[:2] == b'\xaa\x55', 'bad C0 frame')
    require(len(game) == 33, 'bad B1 frame')
    masks = struct.unpack_from('<6H', chain, 2)
    require(all(m & ~0xFFF == 0 for m in masks), 'non-MPR C0 mask')
    require(all(v in (0, 128) for v in chain[14:]+game[:32]), 'non-binary slider')
    stages = {'native': mapped(masks[:3], mapping), 'verified': mapped(masks[3:], mapping),
              'final': [i for i, v in enumerate(chain[14:]) if v],
              'game': [i for i, v in enumerate(game[:32]) if v]}
    for key, values in stages.items():
        require(values == json.loads(row[key+'_lanes_json']), 'stage field differs from raw '+key)
    times = [float(row[k]) for k in ('chain_start_s', 'chain_end_s', 'game_start_s', 'game_end_s')]
    require(times == sorted(times), 'host request time order')
    require(set(stages['verified']).issubset(stages['native']), 'MPR verification synthesizes a native bit')
    return {**stages, 'query_ms': (times[-1]-times[0])*1000}


def decode_native(row, mapping):
    words, channels = [], {}
    for a in (0x5A, 0x5B, 0x5C):
        data = bytes.fromhex(row[f'registers_{a:02x}'])
        require(len(data) == 128, 'incomplete C6 chip image')
        status, oor = struct.unpack_from('<2H', data)
        require(oor == 0 and status & 0x8000 == 0, 'native fault requires separate treatment')
        words.append(status & 0xFFF)
        for e in range(12):
            filtered = int.from_bytes(data[4+2*e:6+2*e], 'little')
            baseline = data[0x1E+e] << 2
            require(filtered < 1024, 'invalid 10-bit filtered data')
            if (a, e) in mapping:
                channels[mapping[(a, e)]] = {'filtered': filtered, 'baseline_floor': baseline,
                                             'delta_floor': baseline-filtered}
    lanes = mapped(words, mapping)
    require(lanes == json.loads(row['native_lanes_json']), 'native field differs from raw status')
    return {'native': lanes, 'channels': channels,
            'query_ms': (float(row['cycle_end_s'])-float(row['cycle_start_s']))*1000}


def stats(values):
    return {'min': min(values), 'median': statistics.median(values), 'max': max(values)} if values else None


def stage_stats(rows, stage):
    count = Counter(lane for r in rows for lane in r[stage])
    patterns = Counter(tuple(r[stage]) for r in rows)
    return {'any_on': sum(bool(r[stage]) for r in rows),
            'lane_on': {str(k): v for k, v in sorted(count.items())},
            'patterns': [{'lanes': list(k), 'frames': v} for k, v in patterns.most_common()]}


def sampled_on_runs(rows, stage, lane):
    runs, start = [], None
    for index, row in enumerate(rows):
        on = lane in row[stage]
        if on and start is None:
            start = index
        if start is not None and (not on or index == len(rows)-1):
            end = index-1 if not on else index
            runs.append({'first_on_s': rows[start]['time'], 'last_on_s': rows[end]['time'],
                         'previous_off_s': rows[start-1]['time'] if start else None,
                         'next_off_s': row['time'] if not on else None,
                         'on_observations': end-start+1})
            start = None
    return runs


def telemetry(meta):
    before, after = (bytes.fromhex(meta[k]['raw_hex']) for k in ('telemetry_before', 'telemetry_after'))
    require(len(before) == len(after) == 528, 'bad C1 length')
    b, a = struct.unpack_from('<10I', before), struct.unpack_from('<10I', after)
    rise_before, rise_after = struct.unpack_from('<32H', before, 40), struct.unpack_from('<32H', after, 40)
    fall_before, fall_after = struct.unpack_from('<32H', before, 104), struct.unpack_from('<32H', after, 104)
    edge_delta = (a[2]-b[2]) & 0xFFFFFFFF
    return {'served_delta': (a[1]-b[1]) & 0xFFFFFFFF, 'edge_delta_including_air': edge_delta,
            'ring_capacity': 32, 'ring_cannot_cover_whole_job': edge_delta > 32,
            'rise_delta': {str(i): (x-y) & 0xFFFF for i, (x,y) in enumerate(zip(rise_after, rise_before)) if x!=y},
            'fall_delta': {str(i): (x-y) & 0xFFFF for i, (x,y) in enumerate(zip(fall_after, fall_before)) if x!=y},
            'verify_fail_counter_changed_indices': [i for i in range(36) if before[168+i] != after[168+i]],
            'scope': 'this host B1 responses; saturated verify counters do not expose failures; not direct HID'}


def analyze(files, context):
    mapping = mapping_from(files)
    output = {'context': context, 'confirmed': [], 'discarded': [], 'jobs': []}
    for name in sorted(files):
        if not name.startswith('actions/') or not name.endswith('.json'):
            continue
        meta = json.loads(files[name])
        if 'mode' not in meta:
            continue
        require(meta['completed'] and isinstance(meta['user_confirmed'], bool), 'incomplete/unlabelled job')
        raw_rows = list(csv.DictReader(io.StringIO(files[name[:-5]+'.csv'].decode('utf-8'))))
        rows = []
        for raw in raw_rows:
            require(not raw['error'], 'capture error requires explicit exclusion')
            values = decode_normal(raw, mapping) if context == 'normal' else decode_native(raw, mapping)
            rows.append({'time': float(raw['elapsed_s']), 'valid': raw['phase_valid'] == '1', **values})
        require(all(rows[i]['time'] < rows[i+1]['time'] for i in range(len(rows)-1)), 'non-monotonic job time')
        job = {'id': meta['id'], 'mode': meta['mode'], 'confirmed': meta['user_confirmed'],
               'rows': len(rows), 'valid_phase_rows': sum(r['valid'] for r in rows),
               'query_ms': stats([r['query_ms'] for r in rows]), 'windows': {}}
        for key, (begin, end) in WINDOWS.items():
            selected = [r for r in rows if r['valid'] and begin <= r['time'] <= end]
            stages = ('native', 'verified', 'final', 'game') if context == 'normal' else ('native',)
            window = {'n': len(selected), 'stages': {s: stage_stats(selected, s) for s in stages}}
            if context == 'normal':
                window['native_verified_differ'] = sum(r['native'] != r['verified'] for r in selected)
                window['native_final_differ'] = sum(r['native'] != r['final'] for r in selected)
                window['C0_final_B1_differ'] = sum(r['final'] != r['game'] for r in selected)
            else:
                window['lanes'] = {str(lane): {field: stats([r['channels'][lane][field] for r in selected])
                                              for field in ('filtered', 'baseline_floor', 'delta_floor')} for lane in range(32)}
            job['windows'][key] = window
        if context == 'normal':
            job['telemetry'] = telemetry(meta)
            require(job['telemetry']['served_delta'] == len(rows), 'B1 count differs from C1 served delta')
            job['sampled_on_runs_lane14'] = {s: sampled_on_runs(rows, s, 14) for s in ('native', 'final', 'game')}
            job['all_rows_stage_differences'] = {s: sum(r[a] != r[b] for r in rows) for s, a, b in (
                ('native_verified', 'native', 'verified'), ('native_final', 'native', 'final'), ('final_game', 'final', 'game'))}
        output['jobs'].append(job)
        output['confirmed' if meta['user_confirmed'] else 'discarded'].append(meta['id'])
    output['rows'] = sum(j['rows'] for j in output['jobs'])
    output['confirmed_rows'] = sum(j['rows'] for j in output['jobs'] if j['confirmed'])
    return output


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path, required=True)
    parser.add_argument('--packet-dir', type=Path, default=DOCS)
    args = parser.parse_args()
    require(not args.out.exists(), 'output already exists')
    files = {key: load_packet(args.packet_dir / name, sha) for key, (name, sha) in PACKETS.items()}
    require(mapping_from(files['normal']) == mapping_from(files['native']), 'source maps differ')
    output = {'scope': 'fixed-window observed reference-chain comparison, no physical-contact error-rate or threshold fitting',
              'windows_s': WINDOWS, 'input_sha256': {k: sha for k, (name,sha) in PACKETS.items()},
              'physical_alignment': 'user reported possible landing/count-origin change; cross-run identical electrode not established',
              'normal_native_scope': 'published driver hardware masks; C0 has no read-quality/freshness flag; failed touched() reads may appear OFF',
              'normal_sampling_scope': 'sequential C0 then B1 observations; query_ms excludes the 10 ms host sleep; not independent sensor scans or direct HID',
              **{k: analyze(v, k) for k, v in files.items()}}
    require(len(output['normal']['confirmed']) == 3 and not output['normal']['discarded'], 'unexpected normal labels')
    require(len(output['native']['confirmed']) == 6 and len(output['native']['discarded']) == 3, 'unexpected native labels')
    args.out.write_text(json.dumps(output, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    for job in output['normal']['jobs']:
        late = job['windows']['late']
        print(job['mode'], 'late', late['n'], {s: v['lane_on'] for s,v in late['stages'].items()},
              'all stage differences', job['all_rows_stage_differences'], 'telemetry', job['telemetry'])
    print('Validated', output['normal']['rows'], 'normal pairs and', output['native']['rows'], 'native cycles')


if __name__ == '__main__':
    main()
