#!/usr/bin/env python3
"""Exercise exact production flash/CDC/driver/LED functions offline (no devices)."""
import hashlib
import json
from pathlib import Path
import re
import subprocess
import test_mbr_distance_pipeline as host

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT.parent / '_dev_tools/round90y_audit_host'

STUBS = r'''
static controller_config ControllerConfig{}, defaultConfig{};
static uint8_t currentPage = 0xff;
static uint8_t flashSector[4096];
#define FLASH_STORAGE_START 0
#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGE_SIZE 256
#define XIP_BASE ((uintptr_t)flashSector)
static unsigned eraseCount, programCount;
static void flash_range_erase(unsigned offset, unsigned size) {
    CHECK(offset == 0 && size == 4096); ++eraseCount; memset(flashSector, 0xff, size);
}
static void flash_range_program(unsigned offset, const uint8_t* data, unsigned size) {
    CHECK(offset % 256 == 0 && size == 256 && offset + size <= 4096);
    for (unsigned i = 0; i < size; ++i) {
        CHECK(flashSector[offset+i] == 0xff);
        flashSector[offset+i] &= data[i];
    }
    ++programCount;
}
static uint8_t g_lampCount = 31;
static unsigned ledFlushes;
struct Led {
    uint32_t pixels[31]{};
    void setColor(unsigned i, uint8_t r, uint8_t g, uint8_t b) {
        CHECK(i < 31); pixels[i] = (uint32_t(r)<<16) | (uint32_t(g)<<8) | b;
    }
    void fill(uint8_t r, uint8_t g, uint8_t b, unsigned first, unsigned count) {
        for (unsigned i = first; i < first+count; ++i) setColor(i,r,g,b);
    }
    void fill(uint8_t r,uint8_t g,uint8_t b) { fill(r,g,b,0,31); }
    void flush() { ++ledFlushes; }
} RGB_LED;
static bool game_connected = false, startupLedOwned = true, s_dirty = false;
static constexpr unsigned kMaxLampCount = 31;
static uint8_t s_rgb[31][3]{};
static unsigned watchdogFeeds;
static void watchdog_update() { ++watchdogFeeds; }
static void sleep_ms(unsigned n) { fakeNow += n; }
static bool connected = true, txDrains = true;
static bool mounted = true, discardUntilDtr = false;
static uint32_t configSessionEpoch = 0;
static uint8_t inputDelayHead, inputDelayCount, inputDelayMs, inputDelayFormat;
static struct { uint8_t slider[32], air; } inputState;
static uint8_t rawReportLevel;
static bool gameRawEnabled;
static bool flashingArmed;
static struct { unsigned cdcTxBytes; } g_tele;
static uint32_t txSpace = 32, txTotal;
static uint8_t rx[128]{};
static uint32_t rxCount, rxPos;
static bool tud_cdc_connected() { return connected; }
static bool tud_mounted() { return mounted; }
static void tud_cdc_read_flush() { rxPos = rxCount; }
static void tud_task() { ++fakeNow; if (txDrains) txSpace = 32; }
static unsigned tud_cdc_write(const void*, unsigned n) {
    unsigned sent = n < txSpace ? n : txSpace; txSpace -= sent; txTotal += sent; return sent;
}
static unsigned tud_cdc_write_flush() { return 0; }
static unsigned tud_cdc_read(void* dst, unsigned n) {
    unsigned available = rxCount-rxPos, take = n < available ? n : available;
    memcpy(dst, rx+rxPos, take); rxPos += take; return take;
}
static int busReturn = -99;
static uint8_t busBytes[256];
static unsigned busWrites, lastBusLength;
static bool lastNostop;
static int i2c_write_read(uint8_t,uint8_t,uint8_t*,size_t,uint8_t* dst,size_t n) {
    int ret = busReturn == -99 ? int(n) : busReturn;
    if (ret > 0) memcpy(dst,busBytes,ret < int(n) ? ret : n);
    return ret;
}
static int i2c_write_stop_read(uint8_t,uint8_t,uint8_t,uint8_t* dst,size_t n) {
    return i2c_write_read(0,0,nullptr,0,dst,n);
}
static int i2c_read(uint8_t,uint8_t,uint8_t* dst,size_t n,bool) {
    return i2c_write_read(0,0,nullptr,0,dst,n);
}
static int i2c_write(uint8_t,uint8_t,uint8_t* data,size_t n,bool nostop) {
    ++busWrites; lastBusLength = n; lastNostop = nostop; memcpy(busBytes,data,n); return int(n);
}
static void sleep_us(unsigned) {}
static const unsigned MPR121_TOUCHSTATUS_L = 0;
class MPR121 {
public:
    uint8_t port=0, addr=0x5a;
    bool readRegisters(uint8_t,uint8_t*,uint8_t);
    uint8_t readRegister8(uint8_t);
    uint16_t readRegister16(uint8_t);
    uint16_t touched();
    bool readTouchStatus(uint16_t*);
};
class VL53L0X {
public:
    uint8_t port=1, address=0x29;
    uint8_t readReg(uint8_t);
    uint16_t readReg16Bit(uint8_t);
    uint32_t readReg32Bit(uint8_t);
    void writeMulti(uint8_t,uint8_t*,uint8_t);
    void readMulti(uint8_t,uint8_t*,uint8_t);
};
class CY8CMBR3116 {
public:
    uint8_t i2c_port=0, DEVICE_I2C_ADDRESS=0x40;
    uint8_t requestData(uint8_t,uint8_t*);
    uint8_t requestDataFromAddress(uint8_t,uint8_t,uint8_t*);
};
class TCA9539 {
public:
    uint8_t _i2c_bus=1, _i2c_address=0x74;
    uint8_t readReg(uint8_t);
    bool isConnected();
};
namespace std { enum memory_order { memory_order_release }; }
struct Owner { bool value=false; void store(bool v,std::memory_order) { value=v; } } core0_owns_usb;
static unsigned usbParked, semPolls;
static bool semStuck;
static void tight_loop_contents() {}
static void watchdog_reboot(unsigned pc,unsigned sp,unsigned delay) {
    CHECK(pc==0 && sp==0 && delay==0 && semStuck && semPolls==200 && watchdogFeeds==199);
    writeText("PASS checks=");writeNumber(checks);writeText("\n");ExitProcess(0);
}
static bool sem_acquire_timeout_ms(unsigned*,unsigned ms) {
    CHECK(core0_owns_usb.value); fakeNow += ms; ++semPolls; return !semStuck && semPolls == 3;
}
'''

CASES = r'''
static void initializeConfig() {
    memset(flashSector,0xff,sizeof(flashSector));
    defaultConfig.magic=CONTROLLER_CONFIG_MAGIC; defaultConfig.cfgVer=CONTROLLER_CONFIG_VERSION;
    defaultConfig.hwVer=1; defaultConfig.th_touch=6; defaultConfig.th_release=4;
    defaultConfig.airMin=200; defaultConfig.airMax=500; defaultConfig.lightLimit=255;
    ControllerConfig=defaultConfig; recomputeXorSum(&ControllerConfig);
}
static void putConfig(unsigned page,unsigned threshold) {
    ControllerConfig.th_touch=threshold; recomputeXorSum(&ControllerConfig);
    memcpy(flashSector+page*256,&ControllerConfig,128);
}
extern "C" void auditEntry() {
    const char* command=GetCommandLineA(); unsigned n=99;
    for (unsigned i=0; command[i]; ++i) if (command[i]=='=') {
        n=0; while(command[++i]>='0' && command[i]<='9') n=n*10+command[i]-'0'; break;
    }
    CHECK(n<21); initializeConfig(); normalLed = {};
    if (n<6) {
        if (n==0) { readConfigSafe(nullptr); CHECK(currentPage==0 && programCount==1); }
        else {
            putConfig(0,6);
            if (n==1 || n==2) {
                flashSector[256]=n==1 ? CONTROLLER_CONFIG_MAGIC : 0;
                flashSector[270]=0; // interrupted header/body with invalid checksum
            }
            if (n==3) { flashSector[256]=0; putConfig(2,9); }
            if (n==4) for(unsigned p=1;p<16;++p) putConfig(p,6+p);
            if (n==5) { flashSector[512+250]=0; } // occupied padding with erased magic
            readConfigSafe(nullptr);
            CHECK(ControllerConfig.th_touch==(n==3 ? 9 : n==4 ? 21 : 6));
        }
        ControllerConfig.th_touch=31; recomputeXorSum(&ControllerConfig); saveConfigSafe(nullptr);
        CHECK(currentPage==(n==4 ? 0 : n==3 || n==5 ? 3 : n==0 ? 1 : 2));
        memset(&ControllerConfig,0,sizeof(ControllerConfig)); readConfigSafe(nullptr);
        CHECK(ControllerConfig.th_touch==31 && validateConfigQuiet(&ControllerConfig));
        CHECK(eraseCount==(n==4 ? 1u : 0u));
    } else if (n==6) {
        MPR121 m; VL53L0X v; CY8CMBR3116 c; uint8_t data[12]; memset(data,0xaa,sizeof(data));
        busReturn=1;
        CHECK(!m.readRegisters(0,data,12)); for(unsigned i=0;i<12;++i) CHECK(data[i]==0);
        CHECK(m.readRegister16(0)==0 && m.touched()==0 && v.readReg16Bit(0)==8190 && v.readReg32Bit(0)==0);
        CHECK(c.requestDataFromAddress(0,12,data)==1); for(unsigned i=0;i<12;++i) CHECK(data[i]==0);
        busReturn=-1; memset(data,0xaa,sizeof(data)); CHECK(c.requestData(12,data)==1);
        for(unsigned i=0;i<12;++i) CHECK(data[i]==0);
        CHECK(v.readReg(0)==0 && m.readRegister8(0)==0);
        TCA9539 t; CHECK(t.readReg(0)==0 && !t.isConnected());
        busReturn=-99; busBytes[0]=0x55; CHECK(t.readReg(0)==0x55 && t.isConnected());
    } else if (n==7) {
        VL53L0X v; uint8_t data[6]={1,2,3,4,5,6}; v.writeMulti(0xb0,data,6);
        CHECK(busWrites==1 && lastBusLength==7 && !lastNostop && busBytes[0]==0xb0);
        for(unsigned i=0;i<6;++i) CHECK(busBytes[i+1]==data[i]);
        busBytes[0]=0xff; busBytes[1]=0x81; busBytes[2]=0x72; busBytes[3]=0x63;
        CHECK(v.readReg32Bit(0)==0xff817263u);
    } else if (n==8 || n==9) {
        uint8_t data[128]{}; txDrains=n==8;
        const uint32_t start=fakeNow;
        CHECK(writeCdcPayload(data,sizeof(data))==(n==8));
        CHECK(fakeNow-start<=252 && watchdogFeeds>0);
        if(n==8) CHECK(txTotal==128); else CHECK(txTotal==32);
    } else if (n==10 || n==11) {
        normalLed.pending=true; normalLed.started=fakeNow;
        normalLed.startedWithDtr=true;
        memset(rx,0x80,sizeof(rx)); rxCount=64; pollNormalLedPayload();
        CHECK(normalLed.pending && normalLed.received==64 && ledFlushes==0);
        if(n==10) fakeNow+=501;
        rx[64]=0xb3; rx[65]=0xbb; rx[66]=0xa5; rxCount=96; pollNormalLedPayload();
        CHECK(!normalLed.pending && normalLed.received==96 && rxPos==96);
        CHECK(ledFlushes==(n==10 ? 0u : 1u));
        normalLed.pending=true; normalLed.received=5; connected=false; pollNormalLedPayload();
        CHECK(!normalLed.pending && normalLed.received==0);
    } else if (n==12) {
        s_dirty=true; ControllerConfig.lightLimit=255; g_lampCount=16;
        for(unsigned i=0;i<31;++i) { s_rgb[i][0]=90; RGB_LED.pixels[i]=0xffffffffu; }
        lamp_array_apply(); CHECK(ledFlushes==0 && s_dirty);
        lamp_array_finish_startup(); lamp_array_apply(); CHECK(ledFlushes==1 && !s_dirty);
        for(unsigned i=16;i<31;++i) CHECK(RGB_LED.pixels[i]==0);
    } else if (n==13) {
        acquireUSBForCore0(); CHECK(semPolls==3 && watchdogFeeds==2);
    } else if (n==14) {
        // Existing hosts without DTR may still send a complete LED transaction.
        connected=false; normalLed.pending=true; normalLed.started=fakeNow;
        normalLed.startedWithDtr=false; rxCount=96; pollNormalLedPayload();
        CHECK(!normalLed.pending && ledFlushes==1);
    } else if (n==15) {
        normalLed.pending=true; normalLed.startedWithDtr=true; normalLed.started=fakeNow;
        rxCount=64; pollNormalLedPayload(); CHECK(normalLed.pending);
        rx[64]=0xb3; rx[65]=0xbb; rx[66]=0xa5; rxCount=96;
        connected=false; cdcSessionStateChanged(false);
        CHECK(!normalLed.pending && discardUntilDtr && rxPos==rxCount && ledFlushes==0);
        // Late bytes during low DTR are flushed at the next session boundary.
        rx[96]=0xb3; rxCount=97; connected=true; cdcSessionStateChanged(true);
        CHECK(!discardUntilDtr && rxPos==97);
        rx[97]=0xb1; rxCount=98; uint8_t command=0;
        CHECK(tud_cdc_read(&command,1)==1 && command==0xb1);
    } else if (n==17 || n==18) {
        normalLed.pending=true; normalLed.raw=true; normalLed.length=1;
        normalLed.started=fakeNow; normalLed.startedWithDtr=true;
        pollNormalLedPayload(); CHECK(normalLed.pending && txTotal==0);
        if(n==18) fakeNow+=101;
        rx[0]=n==17 ? 2 : 0xb3; rxCount=1; pollNormalLedPayload();
        CHECK(!normalLed.pending && rxPos==1);
        CHECK(rawReportLevel==(n==17 ? 2 : 0) && txTotal==(n==17 ? 6u : 0u));
    } else if (n==19) {
        normalLed.pending=true; normalLed.startedWithDtr=false; normalLed.started=fakeNow;
        rxCount=64; pollNormalLedPayload(); CHECK(normalLed.pending);
        connected=true; cdcSessionStateChanged(true);
        CHECK(!normalLed.pending && !discardUntilDtr && rxPos==rxCount);
    } else if (n==20) {
        normalLed.pending=true; normalLed.raw=true; normalLed.length=1;
        normalLed.started=fakeNow; rawReportLevel=2;
        inputDelayCount=3; inputDelayFormat=2; rxCount=1;
        resetCdcSession();
        CHECK(!normalLed.pending && normalLed.length==96 && !normalLed.raw);
        CHECK(rawReportLevel==0 && inputDelayCount==0 && inputDelayFormat==0xff && rxPos==rxCount);
    } else {
        semStuck=true; acquireUSBForCore0(); CHECK(false);
    }
    writeText("PASS checks=");writeNumber(checks);writeText("\n");ExitProcess(0);
}
'''

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    pieces, hashes = [], {}
    for file, signatures in {
        'controller_config.cpp': ['static bool validateConfigQuiet(', 'static void sanitizeConfig(',
                                  'static void recomputeXorSum(', 'void saveConfigSafe(', 'void readConfigSafe('],
        'mpr121.cpp': ['bool MPR121::readRegisters(', 'uint16_t MPR121::touched(',
                      'bool MPR121::readTouchStatus(',
                      'uint8_t MPR121::readRegister8(', 'uint16_t MPR121::readRegister16('],
        'vl53l0x.cpp': ['uint8_t VL53L0X::readReg(', 'uint16_t VL53L0X::readReg16Bit(',
                       'uint32_t VL53L0X::readReg32Bit(', 'void VL53L0X::writeMulti(', 'void VL53L0X::readMulti('],
        'cy8cmbr3116.cpp': ['uint8_t CY8CMBR3116::requestData(', 'uint8_t CY8CMBR3116::requestDataFromAddress('],
        'tca9539.cpp': ['uint8_t TCA9539::readReg(', 'bool TCA9539::isConnected()'],
        'app_link.cpp': ['static bool writeCdcPayload('],
        'chuni_io.cpp': ['void resetCdcSession(', 'void cdcSessionStateChanged(', 'static void pollNormalLedPayload('],
        'lamp_array.cpp': ['void lamp_array_finish_startup()', 'void lamp_array_apply('],
        'usb_device.cpp': ['void acquireUSBForCore0()'],
    }.items():
        text = (ROOT / 'src' / file).read_text(encoding='utf-8-sig')
        if file == 'chuni_io.cpp':
            a = text.index('static struct {\n    uint8_t data[96]')
            pieces.append(text[a:text.index('} normalLed{};', a)+len('} normalLed{};')])
        for signature in signatures:
            body, line = host.extract_function(text, signature)
            pieces.append(body)
            hashes[signature] = {'file': file, 'line': line, 'sha256': hashlib.sha256(body.encode()).hexdigest()}
    prelude = host.PRELUDE[:host.PRELUDE.index('int i2c_write_stop_read')]
    generated = OUT / 'audit.cpp'
    generated.write_text(prelude + STUBS + '\n'.join(pieces) + CASES, encoding='utf-8')
    exe = OUT / 'audit.exe'
    command = [str(host.CLANG), '-std=c++11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-function',
               '-Wno-unused-variable', '-Wno-unused-parameter', '-ffreestanding', '-fno-builtin',
               '-fno-stack-protector', '-fuse-ld=lld', '-nostdlib',
               f'-I{ROOT / "include/share"}', f'-I{ROOT / "include/software"}',
               '-ID:/pico-sdk/src/common/pico_base_headers/include', str(generated),
               '-Wl,/entry:auditEntry', '-Wl,/subsystem:console', str(host.KERNEL32), '-o', str(exe)]
    compiled = subprocess.run(command, capture_output=True, text=True, timeout=60)
    if compiled.returncode: raise SystemExit(compiled.stdout + compiled.stderr)
    results = []
    for n in range(21):
        r = subprocess.run([str(exe), f'--case={n}'], capture_output=True, text=True, timeout=10)
        match = re.search(r'PASS checks=(\d+)', r.stdout)
        passed = r.returncode == 0 and match is not None
        results.append({'case': n, 'passed': passed, 'checks': int(match[1]) if match else 0, 'output': r.stdout+r.stderr})
        print(('PASS' if passed else 'FAIL') + f' {n}: ' + r.stdout.strip())
    (OUT / 'results.json').write_text(json.dumps({'functions': hashes, 'results': results}, indent=2)+'\n', encoding='utf-8')
    total = sum(r['passed'] for r in results)
    print(f'Firmware audit: {total}/21 scenarios, {sum(r["checks"] for r in results)} checks')
    return 0 if total == 21 else 1

if __name__ == '__main__': raise SystemExit(main())
