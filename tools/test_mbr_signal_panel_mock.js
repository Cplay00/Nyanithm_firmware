// Browser plugin not available. Use the installed Playwright package and Edge.
// Target flow: connect -> config -> validate -> save/reboot -> reconnect/readback.
const fs = require('fs');
const path = require('path');
const assert = require('assert');
const { chromium } = require('C:/Users/HP/.cache/codex-runtimes/codex-primary-runtime/dependencies/node/node_modules/playwright');
const root = path.resolve(__dirname, '../..');
const output = path.join(root, '_dev_tools/round89_panel_mock');
fs.mkdirSync(output, { recursive: true });
let checks = 0;
const errors = [];
const expectedConsole = [];
function check(label, value) { assert(value, label); checks++; console.log('PASS ' + label); }
const url = process.env.MBR_PANEL_URL || 'http://127.0.0.1:8765/Nyanithm_firmware_hw_v1/ControlPanel_v1.html?mock=1';

async function readyPage(browser, oldFirmware = false, gen2 = false) {
    const page = await browser.newPage({ viewport: { width: 1440, height: 1080 } });
    page.on('pageerror', e => errors.push(e.message));
    page.on('console', m => {
        if (m.type() !== 'error') return;
        const item = { message: m.text(), url: m.location().url };
        if (/\/favicon\.ico(?:\?|$)/.test(item.url) ||
            (gen2 && item.message.startsWith('设备门禁未通过: 硬件探测到 3 颗 ToF'))) expectedConsole.push(item);
        else errors.push(item);
    });
    page.on('dialog', d => d.dismiss());
    if (oldFirmware) {
        await page.route('**/ControlPanel*.html*', async route => {
            const response = await route.fetch();
            const html = await response.text();
            const replaced = html.replace('const s = "FW:1.6.6-beta1 API:0x10 HW:v1 Built:Jan 01 2026 12:00:00 SDK:2.2.0 VAR:hw_v1 NYANFW1;1.6.6-beta1;hw_v1;BID=round89a";',
                'const s = "FW:1.6.5 API:0x10 HW:v1 Built:Jan 01 2026 12:00:00 SDK:2.2.0 VAR:hw_v1 NYANFW1;1.6.5;hw_v1;BID=round88f";');
            assert(replaced !== html, 'Mock version injection marker exists');
            await route.fulfill({ response, body: replaced });
        });
    }
    await page.goto(gen2 ? url.replace('mock=1', 'mock=gen2') : url);
    check('page identity and nonblank application', (await page.title()).includes('NYANITHM') && await page.locator('#connectBtn').count() === 1);
    // Mock's production startup clicks connect itself after a short delay.
    // A second click would disconnect the just-opened port.
    await page.waitForFunction(() => !document.querySelector('#btnEnterConfig').disabled);
    await page.waitForFunction(() => document.querySelector('#mainLog').textContent.includes('设备信息:'));
    return page;
}

async function enter(page) {
    await page.locator('#btnEnterConfig').click();
    await page.waitForFunction(() => document.querySelector('#btnEnterConfig').textContent.includes('退出') &&
        !document.querySelector('#ckMbr3116').disabled);
    if (await page.locator('#mbrSignalSection').isVisible()) {
        if (!await page.locator('#mbrSignalSection').evaluate(e => e.open)) {
            await page.locator('#mbrSignalSection > summary').click();
        }
    }
}

async function save(page) {
    await page.evaluate(() => { window.__oldSavedPort = window.__mockPort; });
    await page.locator('#btnSaveAndReboot').click();
    await page.locator('#btnSaveAndReboot').click();
    await page.waitForFunction(() => window.__mockPort !== window.__oldSavedPort &&
        !document.querySelector('#btnEnterConfig').disabled &&
        document.querySelector('#mainLog').textContent.includes('已自动重连'), null, { timeout: 20000 })
        .catch(async e => { console.log(await page.locator('#mainLog').innerText()); throw e; });
    await page.waitForFunction(() => !document.querySelector('#mbrSignalStatus').textContent.includes('未确认支持'));
}

(async () => {
    const browser = await chromium.launch({ channel: 'msedge', headless: true });
    try {
        const page = await readyPage(browser);
        check('beta firmware handshake exposes native debug support', await page.locator('#mainLog').innerText().then(t => t.includes('1.6.6-beta1') && t.includes('round89a')));
        await page.locator('#toggleInputBtn').click();
        await page.waitForFunction(() => Number(document.querySelector('#pktVal').textContent) > 3);
        await page.locator('#toggleInputBtn').click();
        check('ordinary input polling produces frames', Number(await page.locator('#pktVal').innerText()) > 3);
        await page.evaluate(() => { window.__ledSent = false; const old=window.__mockPort.onHostWrite.bind(window.__mockPort);
            window.__mockPort.onHostWrite=function(bytes) { if (bytes[0]===0xB2) window.__ledSent=true; return old(bytes); }; });
        await page.locator('#ledBlue').click();
        await page.waitForFunction(() => window.__ledSent);
        check('ordinary LED command passes mock transport', await page.evaluate(() => window.__ledSent));
        await page.evaluate(() => { const c = window.__mockPort.cfg; c[2] = 2; c[3] |= 2; c[127] = calcXorSum(c); });
        await enter(page);
        check('default legacy profile stays off and editable', !await page.locator('#ckMbrSignalGate').isChecked() && await page.locator('#mbrSignalZero').isEnabled());
        check('beta patch retains C7 diagnostic feature', await page.locator('#btnMbrDbgRead').isEnabled());
        await page.locator('#mbrSignalZero').fill('200');
        await page.locator('#mbrSignalFull').fill('180');
        check('invalid zero/full disables save', await page.locator('#btnSaveAndReboot').isDisabled());
        for (const [id, value] of [['mbrSignalZero', '100'], ['mbrSignalFull', '250'], ['mbrSignalOn', '80'], ['mbrSignalOff', '60']]) await page.locator('#' + id).fill(value);
        await page.locator('label:has(#ckMbrSignalGate)').click();
        await page.waitForFunction(() => document.querySelector('#ckMbrSignalGate').checked);
        check('valid profile enables save and count-space preview', await page.locator('#btnSaveAndReboot').isEnabled() &&
            await page.locator('#mbrSignalStatus').innerText().then(t => t.includes('220') && t.includes('190')));
        await page.locator('#mbrSignalContainer').screenshot({ path: path.join(output, 'signal_controls.png') });
        await page.evaluate(() => {
            window.__writes = []; const old = window.__mockPort.onHostWrite.bind(window.__mockPort);
            window.__mockPort.onHostWrite = function(bytes) { window.__writes.push(Array.from(bytes)); return old(bytes); };
        });
        await save(page);
        const saved = await page.evaluate(() => Array.from(window.__mockPort.cfg));
        check('save reconnect retains enabled profile and exact points', saved.slice(91, 98).join(',') === [0xD6, 1, 1, 100, 250, 80, 60].join(','));
        check('saved 128B XOR and extension check valid', saved.reduce((a,b)=>a^b,0) === 0 && saved.slice(91, 98).reduce((a,b)=>a^b,0xA7) === saved[98]);
        const writes = await page.evaluate(() => window.__writes);
        const setAt = writes.findIndex(w => w.length === 1 && w[0] === 0xB7);
        check('CFG_SET command and 128B payload remain separate writes', setAt >= 0 && writes[setAt + 1].length === 128);
        check('signal gate save does not program any chip table', !writes.some(w => w[0] === 0xBA));
        await enter(page);
        check('configuration readback renders persisted profile', await page.locator('#ckMbrSignalGate').isChecked() && await page.locator('#mbrSignalZero').inputValue() === '100');
        await page.locator('label:has(#ckMbr3116)').click();
        await page.waitForFunction(() => !document.querySelector('#ckMbr3116').checked);
        check('MPR selection clears opt-in in UI', !await page.locator('#ckMbrSignalGate').isChecked());
        await save(page);
        check('MPR save clears only profile opt-in and keeps points', await page.evaluate(() => {
            const p = decodeMbrSignalProfile(window.__mockPort.cfg); return p.valid && !p.enabled && p.zero === 100 && p.full === 250;
        }));
        await page.evaluate(() => { const c = window.__mockPort.cfg; c[2] = 3; c[3] &= ~2;
            window.__mockPort.tofCount=4; window.__mockPort.mockMbrMask=0b000110;
            encodeMbrSignalProfile(c, { enabled: true, zero: 100, full: 250, onPercent: 80, offPercent: 60 }); c[127] = calcXorSum(c); });
        await enter(page);
        check('hw3 cfg0=0 shows enabled editable MBR profile', await page.locator('#ckMbrSignalGate').isChecked() && await page.locator('#ckMbrSignalGate').isEnabled());
        await save(page);
        check('hw3 read-save preserves opt-in and signal points', await page.evaluate(() => {
            const p=decodeMbrSignalProfile(window.__mockPort.cfg);
            return window.__mockPort.cfg[2] === 3 && p.enabled && p.zero===100 && p.full===250;
        }));
        await page.evaluate(() => {
            const old = window.__mockPort.onHostWrite.bind(window.__mockPort);
            window.__mockPort.onHostWrite = function(bytes) {
                if (bytes.length === 1 && bytes[0] === 0xBD) { this.send([3]); this.sendText('bad', 0); } else old(bytes);
            };
        });
        await page.locator('#btnDevInfo').click();
        await page.waitForFunction(() => document.querySelector('#mainLog').textContent.includes('设备信息: bad'));
        await enter(page);
        check('malformed re-handshake clears previous feature capability', await page.locator('#ckMbrSignalGate').isDisabled());
        await page.close();

        const old = await readyPage(browser, true);
        await old.evaluate(() => { const c=window.__mockPort.cfg; c[2]=2; c[3]|=2; c[127]=calcXorSum(c); });
        await enter(old);
        check('old 1.6.5 firmware cannot enable signal gate', await old.locator('#ckMbrSignalGate').isDisabled() &&
            await old.locator('#mbrSignalStatus').innerText().then(t => t.includes('未确认支持')));
        await old.close();

        const gen2 = await readyPage(browser, false, true);
        await gen2.locator('#btnEnterConfig').click();
        await gen2.waitForFunction(() => document.querySelector('#mainLog').textContent.includes('拦截') ||
            document.querySelector('#mainLog').textContent.includes('非一代'));
        check('gen2 device configuration rejected', await gen2.locator('#ckMbrSignalGate').isDisabled());
        await gen2.close();
        if (errors.length) console.log('Browser console errors:', JSON.stringify(errors));
        check('no application runtime errors', errors.length === 0);
        fs.writeFileSync(path.join(output, 'results.json'), JSON.stringify({ checks, errors, expectedConsole, url, kind: 'Mock only; no physical serial device' }, null, 2));
        console.log(`${checks}/${checks} rendered Mock checks passed`);
    } finally { await browser.close(); }
})().catch(e => { console.error(e); process.exitCode = 1; });
