"""Regression checks against two immutable device captures, without serial I/O."""
import copy
import hashlib
import importlib.util
import io
import json
from pathlib import Path
import tempfile
import unittest
import warnings
import zipfile

spec = importlib.util.spec_from_file_location('mpr_reference', Path(__file__).with_name('analyze_mpr_reference.py'))
reference = importlib.util.module_from_spec(spec)
spec.loader.exec_module(reference)


class ReferenceAnalysisTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.files = {key: reference.load_packet(reference.DOCS / name, sha)
                     for key, (name, sha) in reference.PACKETS.items()}
        cls.results = {key: reference.analyze(files, key) for key, files in cls.files.items()}

    def test_native_windows_match_earlier_frozen_analysis(self):
        previous = json.loads(self.files['native']['actions_analysis.json'])
        jobs = {job['id']: job for job in previous['jobs']}
        self.assertEqual(len(jobs), 9)
        for job in self.results['native']['jobs']:
            old = jobs[job['id']]
            self.assertEqual(job['confirmed'], old['user_confirmed'])
            self.assertEqual(job['rows'], old['rows'])
            for name, window in job['windows'].items():
                prior = old['windows'][name]
                self.assertEqual(window['n'], prior['valid_cycles'])
                self.assertEqual(window['stages']['native']['any_on'], prior['any_native_on'])
                for lane, fields in window['lanes'].items():
                    self.assertEqual(window['stages']['native']['lane_on'].get(lane, 0), prior['lanes'][lane]['native_on'])
                    for field, value in fields.items():
                        self.assertEqual(value, prior['lanes'][lane][field])

    def test_normal_observations_and_independent_telemetry(self):
        expected = {'finger_contact': (1928, 219, 219, 1, 244),
                    'slow_contact': (1929, 199, 199, 5, 107),
                    'four_contact': (1927, 0, 0, 0, 53)}
        for job in self.results['normal']['jobs']:
            rows, native, final, rises, edges = expected[job['mode']]
            self.assertTrue(job['confirmed'])
            self.assertEqual(job['rows'], rows)
            self.assertEqual(job['windows']['late']['n'], 219)
            self.assertEqual(job['windows']['late']['stages']['native']['any_on'], native)
            self.assertEqual(job['windows']['late']['stages']['final']['any_on'], final)
            self.assertEqual(len(job['sampled_on_runs_lane14']['native']), rises)
            self.assertEqual(job['telemetry']['rise_delta'].get('14', 0), rises)
            self.assertEqual(job['telemetry']['fall_delta'].get('14', 0), rises)
            self.assertEqual(job['telemetry']['edge_delta_including_air'], edges)
            self.assertEqual(job['all_rows_stage_differences']['native_verified'], 0)
            self.assertEqual(job['all_rows_stage_differences']['final_game'], 0)
            for window in ('baseline', 'release'):
                for stage in job['windows'][window]['stages'].values():
                    self.assertEqual(stage['any_on'], 0)

    def test_labels_and_mapping_do_not_merge_discarded_jobs(self):
        self.assertEqual(self.results['native']['rows'], 2194)
        self.assertEqual(self.results['native']['confirmed_rows'], 1462)
        self.assertEqual(len(self.results['native']['discarded']), 3)
        self.assertEqual(self.results['normal']['rows'], 5784)
        self.assertEqual(len(self.results['normal']['confirmed']), 3)
        self.assertEqual(reference.mapping_from(self.files['normal']), reference.mapping_from(self.files['native']))

    def check_fake_packet(self, entries, expected_message):
        blob = io.BytesIO()
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', UserWarning)
            with zipfile.ZipFile(blob, 'w') as archive:
                for name, value in entries:
                    archive.writestr(name, value)
        with tempfile.TemporaryDirectory(prefix='mpr_reference_test_') as directory:
            path = Path(directory) / 'packet.zip'
            path.write_bytes(blob.getvalue())
            with self.assertRaisesRegex(ValueError, expected_message):
                reference.load_packet(path, hashlib.sha256(blob.getvalue()).hexdigest())

    def test_changed_input_packet_is_rejected(self):
        name, _ = reference.PACKETS['normal']
        with self.assertRaisesRegex(ValueError, 'packet hash'):
            reference.load_packet(reference.DOCS / name, '0' * 64)

    def test_bad_member_hash_is_rejected(self):
        self.check_fake_packet([('sample', b'changed'), ('manifest.json', json.dumps({'files': {'sample': '0' * 64}}))],
                               'member hash')

    def test_duplicate_member_is_rejected(self):
        self.check_fake_packet([('sample', b'a'), ('sample', b'b')], 'duplicate')

    def test_missing_manifest_map_is_rejected(self):
        self.check_fake_packet([('manifest.json', '{}')], 'missing member hash')

    def test_bad_raw_frames_and_unverified_native_bits_are_rejected(self):
        import csv
        name = next(name for name in self.files['normal'] if name.startswith('actions/') and name.endswith('.csv'))
        row = next(csv.DictReader(io.StringIO(self.files['normal'][name].decode('utf-8'))))
        mapping = reference.mapping_from(self.files['normal'])
        for field, value in (('chain_hex', 'aa55'), ('game_hex', '00'),
                             ('native_lanes_json', '[31]'), ('game_start_s', '-1')):
            bad = copy.deepcopy(row)
            bad[field] = value
            with self.assertRaises(ValueError):
                reference.decode_normal(bad, mapping)
        bad = copy.deepcopy(row)
        data = bytearray.fromhex(row['chain_hex'])
        data[8] = 1  # Verified chip 0 ELE0 with no corresponding native bit.
        bad['chain_hex'] = data.hex()
        bad['verified_lanes_json'] = json.dumps(reference.mapped((1, 0, 0), mapping))
        with self.assertRaisesRegex(ValueError, 'synthesizes'):
            reference.decode_normal(bad, mapping)

    def test_native_fault_is_not_treated_as_good_capacitance(self):
        import csv
        name = next(name for name in self.files['native'] if name.startswith('actions/') and name.endswith('.csv'))
        row = next(csv.DictReader(io.StringIO(self.files['native'][name].decode('utf-8'))))
        mapping = reference.mapping_from(self.files['native'])
        for offset, value in ((3, 1), (1, 0x80)):
            bad = copy.deepcopy(row)
            data = bytearray.fromhex(row['registers_5a'])
            data[offset] |= value
            bad['registers_5a'] = data.hex()
            with self.assertRaisesRegex(ValueError, 'native fault'):
                reference.decode_native(bad, mapping)


if __name__ == '__main__':
    unittest.main(verbosity=2)
