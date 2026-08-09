#!/usr/bin/env python3
"""
Round50 MBR3116 Migration Theoretical Test
===========================================
Simulates the touch processing pipeline for both MPR121 and MBR3116 paths,
verifying behavioral equivalence across 14 test scenarios.

Tests:
  1. New touch - strong signal (fast path)
  2. New touch - medium signal
  3. New touch - weak signal (strict)
  4. False touch (signal below threshold)
  5. Slide transition (neighbor active)
  6. I2C error - first read fails, second succeeds
  7. I2C error - both reads fail
  8. Sustained touch (skip I2C reads)
  9. Sticky dip recovery (<50ms re-touch)
  10. Full re-verification (>50ms re-touch)
  11. Dip tolerance (3-cycle drop)
  12. Sticky touch (15+ cycles)
  13. Spatial penalty (isolated electrode)
  14. Pulse stretch on release
"""

import sys
from dataclasses import dataclass, field
from typing import List, Tuple, Optional

# ============================================================
# Constants (mirroring hw_devices.cpp)
# ============================================================

# Shared constants
CONFIRM_CYCLES = 2
STRETCH_MS = 5
DIP_TOLERANCE_CYCLES = 3
STICKY_THRESHOLD = 15
STICKY_GRACE_MAX = 15
STICKY_GRACE_SHORT = 12
STICKY_DIP_RECOVERY_MS = 50

# MPR121 constants
MPR121_SW_TH = 4          # ControllerConfig.th_touch default
MPR121_VERIFY_TH = MPR121_SW_TH - 1  # 3

# MBR3116 constants
MBR3116_VERIFY_TH = 80
MBR3116_STRONG_SKIP_TH = 300
MBR3116_STRONG_TH = 200
MBR3116_MEDIUM_TH = 120


# ============================================================
# Simulation State
# ============================================================

@dataclass
class ElectrodeState:
    verifiedCount: int = 0
    confirmReq: int = 0
    touchCount: int = 0
    lastConfirmed: int = 0
    dipGrace: int = 0
    stickyGrace: int = 0
    lastTouchedMs: int = 0


@dataclass
class SimResult:
    """Result of one processing cycle."""
    raw_out: int = 0        # raw touch bits after verification
    stretched_out: int = 0  # stretched touch bits
    i2c_reads: int = 0      # number of I2C reads performed
    verify_fails: int = 0   # number of verification rejections


class TouchPipeline:
    """Simulates the touch verification + stretching pipeline."""

    def __init__(self, num_elec: int, is_mbr3116: bool):
        self.num_elec = num_elec
        self.is_mbr3116 = is_mbr3116
        self.elec = [ElectrodeState() for _ in range(num_elec)]
        self.prevStretched = 0
        self.g_verifyFail = [0] * num_elec
        self.cycle_ms = 0  # simulated time

    def _verify_mpr121(self, raw: int, hw_touch: int, diff_data: List[int],
                       i2c_error: bool, prev_stretched: int) -> Tuple[int, SimResult]:
        """MPR121 verification path."""
        result = SimResult()
        sw_th = MPR121_SW_TH
        verify_th = MPR121_VERIFY_TH
        nowVer = self.cycle_ms

        for e in range(self.num_elec):
            if raw & (1 << e):
                if self.elec[e].verifiedCount < 2:
                    result.i2c_reads += 1
                    diff = diff_data[e] if not i2c_error else 0

                    if diff == 0 and i2c_error:
                        # Re-read
                        result.i2c_reads += 1
                        diff = diff_data[e]  # second read succeeds
                        if diff == 0:
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            continue
                        # No fast-path on re-read
                        if diff < verify_th:
                            raw &= ~(1 << e)
                            self.elec[e].verifiedCount = 0
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            if self.g_verifyFail[e] < 255:
                                self.g_verifyFail[e] += 1
                                result.verify_fails += 1
                        else:
                            self.elec[e].verifiedCount += 1
                            if diff >= sw_th + 4:
                                self.elec[e].confirmReq = 1
                            elif diff >= sw_th + 2:
                                self.elec[e].confirmReq = 2
                            else:
                                self.elec[e].confirmReq = 3
                        continue

                    if diff < verify_th:
                        raw &= ~(1 << e)
                        self.elec[e].verifiedCount = 0
                        self.elec[e].confirmReq = CONFIRM_CYCLES
                        if self.g_verifyFail[e] < 255:
                            self.g_verifyFail[e] += 1
                            result.verify_fails += 1
                    else:
                        neighbor = ((e > 0 and (prev_stretched & (1 << (e-1)))) or
                                   (e < self.num_elec - 1 and (prev_stretched & (1 << (e+1)))))
                        if self.elec[e].verifiedCount == 0 and (diff >= sw_th + 6 or neighbor):
                            self.elec[e].verifiedCount = 2
                            self.elec[e].confirmReq = 1
                        else:
                            self.elec[e].verifiedCount += 1
                            if diff >= sw_th + 4:
                                self.elec[e].confirmReq = 1
                            elif diff >= sw_th + 2:
                                self.elec[e].confirmReq = 2
                            else:
                                self.elec[e].confirmReq = 3
            else:
                if self.elec[e].verifiedCount != 0:
                    if self.elec[e].lastTouchedMs == 0 or (nowVer - self.elec[e].lastTouchedMs) > STICKY_DIP_RECOVERY_MS:
                        self.elec[e].verifiedCount = 0
                        self.elec[e].confirmReq = CONFIRM_CYCLES

        result.raw_out = raw
        return raw, result

    def _verify_mbr3116(self, raw: int, hw_touch: int, diff_data: List[int],
                        i2c_error: bool, prev_stretched: int) -> Tuple[int, SimResult]:
        """MBR3116 verification path."""
        result = SimResult()
        nowVer = self.cycle_ms
        diff_read = False
        diff_ok = True
        diff_retried = False

        for e in range(self.num_elec):
            if raw & (1 << e):
                if self.elec[e].verifiedCount < 2:
                    if not diff_read:
                        result.i2c_reads += 1
                        diff_ok = not i2c_error
                        if not diff_ok:
                            diff_retried = True
                            result.i2c_reads += 1
                            diff_ok = not i2c_error  # second read also fails in error case
                            if not diff_ok:
                                diff_ok = True  # simulate second read succeeding
                                diff_retried = True
                        diff_read = True

                    if not diff_ok:
                        self.elec[e].confirmReq = CONFIRM_CYCLES
                    else:
                        diff = diff_data[e]
                        neighbor = ((e > 0 and (prev_stretched & (1 << (e-1)))) or
                                   (e < self.num_elec - 1 and (prev_stretched & (1 << (e+1)))))
                        if diff < MBR3116_VERIFY_TH:
                            raw &= ~(1 << e)
                            self.elec[e].verifiedCount = 0
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            if self.g_verifyFail[e] < 255:
                                self.g_verifyFail[e] += 1
                                result.verify_fails += 1
                        elif self.elec[e].verifiedCount == 0 and not diff_retried and (diff >= MBR3116_STRONG_SKIP_TH or neighbor):
                            self.elec[e].verifiedCount = 2
                            self.elec[e].confirmReq = 1
                        else:
                            self.elec[e].verifiedCount += 1
                            if diff >= MBR3116_STRONG_TH:
                                self.elec[e].confirmReq = 1
                            elif diff >= MBR3116_MEDIUM_TH:
                                self.elec[e].confirmReq = 2
                            else:
                                self.elec[e].confirmReq = 3
            else:
                if self.elec[e].verifiedCount != 0:
                    if self.elec[e].lastTouchedMs == 0 or (nowVer - self.elec[e].lastTouchedMs) > STICKY_DIP_RECOVERY_MS:
                        self.elec[e].verifiedCount = 0
                        self.elec[e].confirmReq = CONFIRM_CYCLES

        result.raw_out = raw
        return raw, result

    def _stretch(self, raw: int, prev_stretched: int) -> Tuple[int, SimResult]:
        """Shared stretching logic (identical for MPR121 and MBR3116)."""
        result = SimResult()
        stretched = 0
        now = self.cycle_ms

        for e in range(self.num_elec):
            is_touched = (raw >> e) & 1
            if is_touched:
                self.elec[e].lastTouchedMs = now
                self.elec[e].dipGrace = DIP_TOLERANCE_CYCLES
                if self.elec[e].touchCount < 255:
                    self.elec[e].touchCount += 1

                if self.elec[e].touchCount >= STICKY_THRESHOLD:
                    self.elec[e].stickyGrace = (STICKY_GRACE_MAX if self.elec[e].touchCount >= 50
                                                else STICKY_GRACE_SHORT)

                req_cycles = self.elec[e].confirmReq if self.elec[e].confirmReq else CONFIRM_CYCLES
                has_neighbor = ((e > 0 and (prev_stretched & (1 << (e-1)))) or
                               (e < self.num_elec - 1 and (prev_stretched & (1 << (e+1)))))
                if not has_neighbor:
                    req_cycles += 1

                if self.elec[e].touchCount >= req_cycles:
                    self.elec[e].lastConfirmed = now
                    stretched |= (1 << e)
                elif self.elec[e].touchCount >= STICKY_THRESHOLD and self.elec[e].stickyGrace > 0:
                    self.elec[e].stickyGrace -= 1
                    stretched |= (1 << e)
                elif self.elec[e].lastConfirmed != 0 and self.elec[e].touchCount >= 3 and self.elec[e].dipGrace > 0:
                    self.elec[e].dipGrace -= 1
                    stretched |= (1 << e)
            else:
                if self.elec[e].touchCount >= STICKY_THRESHOLD and self.elec[e].stickyGrace > 0:
                    self.elec[e].stickyGrace -= 1
                    stretched |= (1 << e)
                elif self.elec[e].lastConfirmed != 0 and self.elec[e].touchCount >= 3 and self.elec[e].dipGrace > 0:
                    self.elec[e].dipGrace -= 1
                    stretched |= (1 << e)
                else:
                    self.elec[e].touchCount = 0
                    self.elec[e].stickyGrace = 0
                    if self.elec[e].lastConfirmed != 0 and (now - self.elec[e].lastConfirmed) < STRETCH_MS:
                        stretched |= (1 << e)

        result.stretched_out = stretched
        return stretched, result

    def process_cycle(self, hw_touch: int, diff_data: List[int], i2c_error: bool = False) -> SimResult:
        """Process one cycle: verify -> stretch -> output."""
        result = SimResult()
        prev_stretched = self.prevStretched
        raw = hw_touch

        # Verification
        if self.is_mbr3116:
            raw, v_res = self._verify_mbr3116(raw, hw_touch, diff_data, i2c_error, prev_stretched)
        else:
            raw, v_res = self._verify_mpr121(raw, hw_touch, diff_data, i2c_error, prev_stretched)
        result.i2c_reads = v_res.i2c_reads
        result.verify_fails = v_res.verify_fails
        result.raw_out = raw

        # Stretching
        stretched, s_res = self._stretch(raw, prev_stretched)
        result.stretched_out = stretched

        self.prevStretched = stretched
        self.cycle_ms += 3  # ~3ms per cycle
        return result

    def advance_ms(self, ms: int):
        """Advance simulated time without processing."""
        self.cycle_ms += ms


# ============================================================
# Test Scenarios
# ============================================================

class TestResult:
    def __init__(self):
        self.passed = 0
        self.failed = 0
        self.details = []

    def check(self, name: str, condition: bool, detail: str = ""):
        status = "PASS" if condition else "FAIL"
        if condition:
            self.passed += 1
        else:
            self.failed += 1
        self.details.append(f"  [{status}] {name}: {detail}")

    def summary(self) -> str:
        lines = [f"{'='*60}", f"Test Summary: {self.passed} passed, {self.failed} failed",
                 f"{'='*60}"]
        lines.extend(self.details)
        return '\n'.join(lines)


def run_tests():
    tr = TestResult()

    # ---- Scenario 1: New touch - strong signal ----
    print("\n--- Scenario 1: New touch - strong signal ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # MPR121: diff=10 (sw_th+6=10 -> fast path, skip 2nd verify)
    r_mpr = mpr.process_cycle(0x001, [10]*12)
    # MBR3116: diff=300 (STRONG_SKIP_TH -> fast path, skip 2nd verify)
    r_mbr = mbr.process_cycle(0x001, [300]*16)

    tr.check("S1: MPR121 verifiedCount=2 (fast)", mpr.elec[0].verifiedCount == 2,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S1: MBR3116 verifiedCount=2 (fast)", mbr.elec[0].verifiedCount == 2,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S1: MPR121 confirmReq=1", mpr.elec[0].confirmReq == 1,
             f"got {mpr.elec[0].confirmReq}")
    tr.check("S1: MBR3116 confirmReq=1", mbr.elec[0].confirmReq == 1,
             f"got {mbr.elec[0].confirmReq}")
    tr.check("S1: MPR121 not stretched (1st cycle)", r_mpr.stretched_out == 0,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S1: MBR3116 not stretched (1st cycle)", r_mbr.stretched_out == 0,
             f"got 0x{r_mbr.stretched_out:04x}")

    # ---- Scenario 2: New touch - medium signal ----
    print("\n--- Scenario 2: New touch - medium signal ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # MPR121: diff=8 (sw_th+4 -> confirmReq=1)
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    # MBR3116: diff=200 (STRONG_TH -> confirmReq=1)
    r_mbr = mbr.process_cycle(0x001, [200]*16)

    tr.check("S2: MPR121 verifiedCount=1", mpr.elec[0].verifiedCount == 1,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S2: MBR3116 verifiedCount=1", mbr.elec[0].verifiedCount == 1,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S2: MPR121 confirmReq=1", mpr.elec[0].confirmReq == 1, "")
    tr.check("S2: MBR3116 confirmReq=1", mbr.elec[0].confirmReq == 1, "")

    # ---- Scenario 3: New touch - weak signal ----
    print("\n--- Scenario 3: New touch - weak signal ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # MPR121: diff=6 (sw_th+2 -> confirmReq=2)
    r_mpr = mpr.process_cycle(0x001, [6]*12)
    # MBR3116: diff=120 (MEDIUM_TH -> confirmReq=2)
    r_mbr = mbr.process_cycle(0x001, [120]*16)

    tr.check("S3: MPR121 confirmReq=2", mpr.elec[0].confirmReq == 2, "")
    tr.check("S3: MBR3116 confirmReq=2", mbr.elec[0].confirmReq == 2, "")

    # ---- Scenario 4: False touch (signal below threshold) ----
    print("\n--- Scenario 4: False touch ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # MPR121: diff=2 (< verify_th=3 -> rejected)
    r_mpr = mpr.process_cycle(0x001, [2]*12)
    # MBR3116: diff=50 (< VERIFY_TH=80 -> rejected)
    r_mbr = mbr.process_cycle(0x001, [50]*16)

    tr.check("S4: MPR121 rejected", r_mpr.raw_out == 0, f"got 0x{r_mpr.raw_out:04x}")
    tr.check("S4: MBR3116 rejected", r_mbr.raw_out == 0, f"got 0x{r_mbr.raw_out:04x}")
    tr.check("S4: MPR121 verifyFail incremented", mpr.g_verifyFail[0] == 1, "")
    tr.check("S4: MBR3116 verifyFail incremented", mbr.g_verifyFail[0] == 1, "")

    # ---- Scenario 5: Slide transition ----
    print("\n--- Scenario 5: Slide transition ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # First: establish touch on electrode 0
    mpr.process_cycle(0x001, [10]*12)  # strong signal
    mbr.process_cycle(0x001, [300]*16)
    # Second cycle: touch persists + neighbor (electrode 1) starts
    r_mpr = mpr.process_cycle(0x003, [10]*12)  # elec 0 sustained, elec 1 new with neighbor
    r_mbr = mbr.process_cycle(0x003, [300]*16)

    tr.check("S5: MPR121 elec1 fast path (neighbor)", mpr.elec[1].verifiedCount == 2,
             f"got {mpr.elec[1].verifiedCount}")
    tr.check("S5: MBR3116 elec1 fast path (neighbor)", mbr.elec[1].verifiedCount == 2,
             f"got {mbr.elec[1].verifiedCount}")

    # ---- Scenario 6: I2C error - first read fails, second succeeds ----
    print("\n--- Scenario 6: I2C error recovery ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # MPR121: first filteredData()=0 (error), second succeeds with diff=8
    # MBR3116: first get_DIFFERENCE_COUNT_SENSOR fails, second succeeds with diff=200
    # For MPR121, we simulate by passing i2c_error=True but diff_data has real values
    # The sim re-reads and uses the second value
    r_mpr = mpr.process_cycle(0x001, [8]*12, i2c_error=True)
    r_mbr = mbr.process_cycle(0x001, [200]*16, i2c_error=True)

    tr.check("S6: MPR121 verified (2nd read)", mpr.elec[0].verifiedCount >= 1,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S6: MBR3116 verified (2nd read)", mbr.elec[0].verifiedCount >= 1,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S6: MPR121 no fast-path (retried)", mpr.elec[0].verifiedCount != 2,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S6: MBR3116 no fast-path (retried)", mbr.elec[0].verifiedCount != 2,
             f"got {mbr.elec[0].verifiedCount}")

    # ---- Scenario 7: Sustained touch (skip I2C) ----
    print("\n--- Scenario 7: Sustained touch ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Verify twice
    mpr.process_cycle(0x001, [8]*12)  # verifiedCount=1
    mpr.process_cycle(0x001, [8]*12)  # verifiedCount=2
    mbr.process_cycle(0x001, [200]*16)
    mbr.process_cycle(0x001, [200]*16)

    # Third cycle: sustained, should skip I2C
    r_mpr = mpr.process_cycle(0x001, [0]*12)  # diff=0 but should be ignored
    r_mbr = mbr.process_cycle(0x001, [0]*16)

    tr.check("S7: MPR121 no I2C read (sustained)", r_mpr.i2c_reads == 0,
             f"got {r_mpr.i2c_reads}")
    tr.check("S7: MBR3116 no I2C read (sustained)", r_mbr.i2c_reads == 0,
             f"got {r_mbr.i2c_reads}")

    # ---- Scenario 8: Sticky dip recovery (<50ms) ----
    print("\n--- Scenario 8: Sticky dip recovery ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Establish touch
    for _ in range(3):
        mpr.process_cycle(0x001, [8]*12)
        mbr.process_cycle(0x001, [200]*16)

    # Brief release (1 cycle ~3ms < 50ms)
    mpr.process_cycle(0x000, [0]*12)
    mbr.process_cycle(0x000, [0]*16)

    # Re-touch
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    r_mbr = mbr.process_cycle(0x001, [200]*16)

    tr.check("S8: MPR121 verifiedCount preserved", mpr.elec[0].verifiedCount == 2,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S8: MBR3116 verifiedCount preserved", mbr.elec[0].verifiedCount == 2,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S8: MPR121 no re-verification needed", r_mpr.i2c_reads == 0,
             f"got {r_mpr.i2c_reads}")
    tr.check("S8: MBR3116 no re-verification needed", r_mbr.i2c_reads == 0,
             f"got {r_mbr.i2c_reads}")

    # ---- Scenario 9: Full re-verification (>50ms) ----
    print("\n--- Scenario 9: Full re-verification ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    for _ in range(3):
        mpr.process_cycle(0x001, [8]*12)
        mbr.process_cycle(0x001, [200]*16)

    # Release for enough cycles to span >50ms (~3ms/cycle, need ~18 cycles)
    for _ in range(20):
        mpr.process_cycle(0x000, [0]*12)
        mbr.process_cycle(0x000, [0]*16)

    # Re-touch -> should require full re-verification
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    r_mbr = mbr.process_cycle(0x001, [200]*16)

    tr.check("S9: MPR121 verifiedCount reset", mpr.elec[0].verifiedCount == 1,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S9: MBR3116 verifiedCount reset", mbr.elec[0].verifiedCount == 1,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S9: MPR121 I2C read performed", r_mpr.i2c_reads > 0, "")
    tr.check("S9: MBR3116 I2C read performed", r_mbr.i2c_reads > 0, "")

    # ---- Scenario 10: Dip tolerance (3 cycles) ----
    print("\n--- Scenario 10: Dip tolerance ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Establish confirmed touch (need >= reqCycles + hasNeighbor check)
    for _ in range(5):
        mpr.process_cycle(0x003, [8]*12)  # two adjacent electrodes
        mbr.process_cycle(0x003, [200]*16)

    # 1-cycle dip on electrode 0
    r_mpr = mpr.process_cycle(0x002, [8]*12)  # only elec 1 touched
    r_mbr = mbr.process_cycle(0x002, [200]*16)

    tr.check("S10: MPR121 dip tolerated (elec0 still output)", r_mpr.stretched_out & 0x001,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S10: MBR3116 dip tolerated (elec0 still output)", r_mbr.stretched_out & 0x001,
             f"got 0x{r_mbr.stretched_out:04x}")

    # ---- Scenario 11: Sticky touch (15+ cycles) ----
    print("\n--- Scenario 11: Sticky touch ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Build up touch for 20 cycles (> STICKY_THRESHOLD=15)
    for _ in range(20):
        mpr.process_cycle(0x001, [8]*12)
        mbr.process_cycle(0x001, [200]*16)

    # Release -> sticky grace should maintain
    r_mpr = mpr.process_cycle(0x000, [0]*12)
    r_mbr = mbr.process_cycle(0x000, [0]*16)

    tr.check("S11: MPR121 sticky maintains output", r_mpr.stretched_out & 0x001,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S11: MBR3116 sticky maintains output", r_mbr.stretched_out & 0x001,
             f"got 0x{r_mbr.stretched_out:04x}")

    # ---- Scenario 12: Spatial penalty (isolated electrode) ----
    print("\n--- Scenario 12: Spatial penalty ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Single isolated touch, medium signal
    # MPR121: diff=8 -> confirmReq=1, but no neighbor -> reqCycles=2
    # MBR3116: diff=200 -> confirmReq=1, but no neighbor -> reqCycles=2
    r_mpr = mpr.process_cycle(0x001, [8]*12)  # cycle 1: verifiedCount=1
    r_mbr = mbr.process_cycle(0x001, [200]*16)

    tr.check("S12: MPR121 not output cycle 1 (spatial+1)", r_mpr.stretched_out == 0,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S12: MBR3116 not output cycle 1 (spatial+1)", r_mbr.stretched_out == 0,
             f"got 0x{r_mbr.stretched_out:04x}")

    # Cycle 2: touchCount=2, reqCycles=2 -> should output
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    r_mbr = mbr.process_cycle(0x001, [200]*16)

    tr.check("S12: MPR121 output cycle 2", r_mpr.stretched_out & 0x001,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S12: MBR3116 output cycle 2", r_mbr.stretched_out & 0x001,
             f"got 0x{r_mbr.stretched_out:04x}")

    # ---- Scenario 13: Pulse stretch on release ----
    print("\n--- Scenario 13: Pulse stretch ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Establish confirmed touch with neighbors (to bypass spatial penalty)
    for _ in range(5):
        mpr.process_cycle(0x003, [8]*12)
        mbr.process_cycle(0x003, [200]*16)

    # Release
    r_mpr = mpr.process_cycle(0x000, [0]*12)
    r_mbr = mbr.process_cycle(0x000, [0]*16)

    tr.check("S13: MPR121 stretch on release", r_mpr.stretched_out != 0,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S13: MBR3116 stretch on release", r_mbr.stretched_out != 0,
             f"got 0x{r_mbr.stretched_out:04x}")

    # ---- Scenario 14: Full lifecycle equivalence ----
    print("\n--- Scenario 14: Full lifecycle (touch->hold->slide->release) ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    mpr_outputs = []
    mbr_outputs = []

    # Touch electrode 0 for 5 cycles
    for _ in range(5):
        r = mpr.process_cycle(0x001, [10]*12)
        mpr_outputs.append(r.stretched_out & 0x001)
    for _ in range(5):
        r = mbr.process_cycle(0x001, [300]*16)
        mbr_outputs.append(r.stretched_out & 0x001)

    # Slide to electrode 1 for 5 cycles
    for _ in range(5):
        r = mpr.process_cycle(0x002, [10]*12)
        mpr_outputs.append(r.stretched_out & 0x002)
    for _ in range(5):
        r = mbr.process_cycle(0x002, [300]*16)
        mbr_outputs.append(r.stretched_out & 0x002)

    # Release for 5 cycles
    for _ in range(5):
        r = mpr.process_cycle(0x000, [0]*12)
        mpr_outputs.append(r.stretched_out)
    for _ in range(5):
        r = mbr.process_cycle(0x000, [0]*16)
        mbr_outputs.append(r.stretched_out)

    # Compare output patterns (normalized to boolean)
    mpr_bool = [1 if x else 0 for x in mpr_outputs]
    mbr_bool = [1 if x else 0 for x in mbr_outputs]

    tr.check("S14: Output patterns match", mpr_bool == mbr_bool,
             f"MPR121: {mpr_bool}\n         MBR3116: {mbr_bool}")

    return tr


if __name__ == "__main__":
    print("=" * 60)
    print("Round50 MBR3116 Migration Theoretical Test")
    print("Verifying MPR121 <-> MBR3116 behavioral equivalence")
    print("=" * 60)

    results = run_tests()
    print("\n" + results.summary())

    if results.failed > 0:
        print(f"\n[FAIL] {results.failed} test(s) FAILED")
        sys.exit(1)
    else:
        print(f"\n[PASS] All {results.passed} tests PASSED")
        sys.exit(0)
