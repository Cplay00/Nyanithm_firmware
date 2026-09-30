// Offline checks run the production helpers directly in a VM, without serial I/O.
const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert');
const root = path.resolve(__dirname, '../..');
const context = vm.createContext({ TextDecoder });
vm.runInContext(fs.readFileSync(path.join(root, 'ControlPanel_v1_refit_dev/src/app_core.js'), 'utf8') +
    '\nglobalThis.api={validateMbrSignalProfile,decodeMbrSignalProfile,encodeMbrSignalProfile,evaluateMbrCalibration,calcXorSum,sanitizeMbrSignalProfile,supportsMbrSignalFirmware,cmpVersion};', context);
const api = context.api;
let checks = 0;
function check(label, run) { run(); checks++; console.log('PASS ' + label); }
const profile = { enabled: true, zero: 70, full: 180, onPercent: 70, offPercent: 50 };
check('valid manual signal parameters', () => assert(api.validateMbrSignalProfile(profile).valid));
for (const [name, value] of [['zero', -1], ['zero', 255], ['zero', 70.2], ['full', 70], ['full', 256],
    ['onPercent', 0], ['onPercent', 101], ['offPercent', -1], ['offPercent', 70], ['offPercent', 101], ['enabled', 1]]) {
    check('reject invalid ' + name + '=' + value, () => assert(!api.validateMbrSignalProfile({ ...profile, [name]: value }).valid));
}
check('missing profile fails closed', () => assert(!api.validateMbrSignalProfile(null).valid));
check('legacy zero extension defaults off', () => {
    const p = api.decodeMbrSignalProfile(new Uint8Array(128));
    assert(!p.valid && !p.enabled && p.legacy);
});
const buf = Uint8Array.from({ length: 128 }, (_, i) => i);
const original = buf.slice();
api.encodeMbrSignalProfile(buf, profile);
check('exact schema bytes 91..97', () => assert.deepStrictEqual([...buf.slice(91, 98)], [0xD6, 1, 1, 70, 180, 70, 50]));
check('extension checksum at byte 98', () => assert.equal(buf[98], [...buf.slice(91, 98)].reduce((a, b) => a ^ b, 0xA7)));
check('legacy gate and all unrelated bytes preserved', () => {
    assert.deepStrictEqual([...buf.slice(0, 91)], [...original.slice(0, 91)]);
    assert.deepStrictEqual([...buf.slice(99)], [...original.slice(99)]);
});
check('encode and decode roundtrip', () => {
    const decoded = api.decodeMbrSignalProfile(buf);
    assert(decoded.valid && decoded.enabled && decoded.zero === 70 && decoded.full === 180);
});
for (const offset of [91, 92, 93, 94, 95, 96, 97, 98]) {
    check('corrupt extension byte ' + offset + ' defaults off', () => {
        const corrupt = buf.slice(); corrupt[offset] ^= 0x04;
        assert(!api.decodeMbrSignalProfile(corrupt).enabled);
    });
}
check('unknown flags rejected even with matching checksum', () => {
    const corrupt = buf.slice(); corrupt[93] = 3; corrupt[98] ^= 2;
    assert(!api.decodeMbrSignalProfile(corrupt).valid);
});
check('disabled valid profile keeps calibration', () => {
    const off = buf.slice(); api.encodeMbrSignalProfile(off, { ...profile, enabled: false });
    const p = api.decodeMbrSignalProfile(off);
    assert(p.valid && !p.enabled && p.zero === 70 && p.full === 180 && off[90] === original[90]);
});
check('config XOR still covers all 127 bytes', () => {
    buf[127] = api.calcXorSum(buf);
    assert.equal([...buf].reduce((a, b) => a ^ b, 0), 0);
});
function inputFor(hover = 40, contact = 160, laneList = [0, 1]) {
    return { quantity: 'DIFF', requiredLanes: laneList, minSamplesPerLane: 20, noiseMargin: 5,
        prohibitedSamples: laneList.flatMap(lane => Array.from({ length: 20 }, () => ({ lane, targetLane: lane, diff: hover }))),
        contactSamples: laneList.flatMap(lane => Array.from({ length: 20 }, () => ({ lane, targetLane: lane, diff: contact }))) };
}
check('separated labelled samples yield disabled proposal only', () => {
    const result = api.evaluateMbrCalibration(inputFor());
    assert(result.valid && result.proposal.zero === 45 && result.proposal.full === 155 && !result.proposal.enabled);
});
check('all-zero contact fails', () => assert(!api.evaluateMbrCalibration(inputFor(0, 0)).valid));
check('overlapping distributions fail', () => assert(!api.evaluateMbrCalibration(inputFor(120, 100)).valid));
check('noise margins consume narrow separation', () => assert(!api.evaluateMbrCalibration(inputFor(90, 100)).valid));
check('clipped 255 DIFF contact fails', () => assert(!api.evaluateMbrCalibration(inputFor(40, 255)).valid));
check('explicit clipping flag fails', () => {
    const input = inputFor(); input.contactSamples[0].clipped = true;
    assert(!api.evaluateMbrCalibration(input).valid);
});
check('RAW quantity cannot masquerade as DIFF', () => {
    const input = inputFor(); input.quantity = 'RAW';
    assert(!api.evaluateMbrCalibration(input).valid);
});
check('wrong physical lane hit fails', () => {
    const input = inputFor(); input.contactSamples[0].targetLane = 1;
    assert(!api.evaluateMbrCalibration(input).valid);
});
check('missing physical lane label fails', () => {
    const input = inputFor(); delete input.contactSamples[0].targetLane;
    assert(!api.evaluateMbrCalibration(input).valid);
});
check('missing lane sample count fails', () => {
    const input = inputFor(); input.contactSamples.pop();
    assert(!api.evaluateMbrCalibration(input).valid);
});
check('global inter-lane overlap fails despite individually separated lanes', () => {
    const input = inputFor();
    input.prohibitedSamples.filter(s => s.lane === 1).forEach(s => s.diff = 180);
    input.contactSamples.filter(s => s.lane === 1).forEach(s => s.diff = 230);
    assert(!api.evaluateMbrCalibration(input).valid);
});
for (const diff of [-1, 256, 1.5, NaN]) {
    check('invalid DIFF=' + diff + ' fails', () => {
        const input = inputFor(); input.contactSamples[0].diff = diff;
        assert(!api.evaluateMbrCalibration(input).valid);
    });
}
check('duplicate required lanes fail', () => assert(!api.evaluateMbrCalibration(inputFor(40, 160, [0, 0])).valid));
check('empty samples fail', () => {
    const input = inputFor(); input.prohibitedSamples = [];
    assert(!api.evaluateMbrCalibration(input).valid);
});
check('sample inputs are not mutated', () => {
    const input = inputFor(); const before = JSON.stringify(input);
    api.evaluateMbrCalibration(input); assert.equal(JSON.stringify(input), before);
});
for (const [firmware, buildId, expected] of [
    ['1.6.5', 'round88f', false], ['1.6.5', 'round89a', false],
    ['1.6.4-beta1', 'round89a', false], ['1.6.6-beta1', 'round89', false],
    ['1.6.6-beta1', 'round89a', true], ['1.6.6', 'round89a', true],
    ['1.6.6-beta2', 'round89b', true], ['1.6.7-beta1', 'round90', true],
    ['1.6.6-beta0', 'round89a', false], ['1.6.6-beta1', null, false],
    ['1.6.6-beta1', 'round89a-junk', false], ['1.6.6-beta1', 'garbage', false],
    ['nonsense', 'round89a', false], [null, 'round89a', false]]) {
    check('feature version/build gate ' + firmware + '/' + buildId, () =>
        assert.equal(api.supportsMbrSignalFirmware(firmware, buildId), expected));
}
check('beta patch version keeps native debug feature available', () =>
    assert(api.cmpVersion('1.6.6-beta1', '1.6.5') > 0));
check('older beta cannot pass newer numeric feature floor', () =>
    assert(api.cmpVersion('1.6.1-beta2', '1.6.2-beta1') < 0));
for (const hwVer of [1, 2, 3, 4]) {
    check('sanitizer MPR/MBR hardware selection hwVer=' + hwVer, () => {
        const cfg = new Uint8Array(128); cfg[2] = hwVer;
        api.encodeMbrSignalProfile(cfg, profile);
        api.sanitizeMbrSignalProfile(cfg);
        const decoded = api.decodeMbrSignalProfile(cfg);
        assert(decoded.valid && decoded.enabled === (hwVer >= 3));
        assert.equal(decoded.zero, profile.zero); assert.equal(decoded.full, profile.full);
    });
}
check('MBR bit retains profile on v1 layout', () => {
    const cfg = new Uint8Array(128); cfg[2] = 2; cfg[3] = 2;
    api.encodeMbrSignalProfile(cfg, profile); api.sanitizeMbrSignalProfile(cfg);
    assert(api.decodeMbrSignalProfile(cfg).enabled);
});
check('invalid extension clears only flags and extension check', () => {
    const cfg = original.slice(); const before = cfg.slice();
    api.sanitizeMbrSignalProfile(cfg);
    for (let i=0; i<128; i++) assert.equal(cfg[i], i === 93 || i === 98 ? 0 : before[i]);
});
check('shared header schema constants match panel production helpers', () => {
    const header=fs.readFileSync(path.join(root,'Nyanithm_firmware_hw_v1/include/share/nyanithm_shared.h'),'utf8');
    for (const [name,value] of [['MBR_DISTANCE_PROFILE_MAGIC',0xD6],['MBR_DISTANCE_PROFILE_VERSION',1],
        ['MBR_DISTANCE_PROFILE_CHECK_SEED',0xA7]]) {
        const match=header.match(new RegExp(name + ' = (0x[0-9A-Fa-f]+|[0-9]+);'));
        assert(match && Number(match[1]) === value);
    }
    assert(header.includes('offsetof(controller_config, mbrDistanceMagic) == 91'));
    assert(header.includes('offsetof(controller_config, xorSum) == 127'));
});
console.log(`${checks}/${checks} offline production-helper checks passed`);
