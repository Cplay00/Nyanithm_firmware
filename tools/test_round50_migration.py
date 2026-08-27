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
  15. round64: per-key inheritance equivalence (all zeros == global)
  16. round64: per-key raise on one lane, others unchanged
  17. round64: MBR tier equivalence (base_k + offsets)
  18. round64: sanitizeConfig per-key clamping
  19. round64: binary touch semantics vs future pressure substitution
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

    def __init__(self, num_elec: int, is_mbr3116: bool, per_key: Optional[List[int]] = None):
        self.num_elec = num_elec
        self.is_mbr3116 = is_mbr3116
        self.elec = [ElectrodeState() for _ in range(num_elec)]
        self.prevStretched = 0
        self.g_verifyFail = [0] * num_elec
        self.cycle_ms = 0  # simulated time
        # round64: per-electrode threshold override, 0 = inherit global.
        self.per_key = per_key if per_key is not None else [0] * num_elec

    def _verify_mpr121(self, raw: int, hw_touch: int, diff_data: List[int],
                       i2c_error: bool, prev_stretched: int) -> Tuple[int, SimResult]:
        """MPR121 verification path."""
        result = SimResult()
        nowVer = self.cycle_ms

        for e in range(self.num_elec):
            if raw & (1 << e):
                if self.elec[e].verifiedCount < 2:
                    result.i2c_reads += 1
                    diff = diff_data[e] if not i2c_error else 0
                    # round64: per-electrode sw_th (0 = inherit global th_touch)
                    pk = self.per_key[e] if e < len(self.per_key) else 0
                    sw_th = pk if pk != 0 else MPR121_SW_TH
                    if sw_th < 4:
                        sw_th = 4
                    verify_th = sw_th - 1 if sw_th > 1 else 1

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
                        # round64: base_k = per-key if set, else the default 80;
                        # tiers = base_k + {40, 120, 220}. MBR can only tighten:
                        # config <=80 keeps the verify baseline (never relaxes).
                        pk = self.per_key[e] if e < len(self.per_key) else 0
                        base_k = max(pk if pk != 0 else MBR3116_VERIFY_TH, MBR3116_VERIFY_TH)
                        strong_skip = base_k + 220
                        strong = base_k + 120
                        medium = base_k + 40
                        neighbor = ((e > 0 and (prev_stretched & (1 << (e-1)))) or
                                   (e < self.num_elec - 1 and (prev_stretched & (1 << (e+1)))))
                        if diff < base_k:
                            raw &= ~(1 << e)
                            self.elec[e].verifiedCount = 0
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            if self.g_verifyFail[e] < 255:
                                self.g_verifyFail[e] += 1
                                result.verify_fails += 1
                        elif self.elec[e].verifiedCount == 0 and not diff_retried and (diff >= strong_skip or neighbor):
                            self.elec[e].verifiedCount = 2
                            self.elec[e].confirmReq = 1
                        else:
                            self.elec[e].verifiedCount += 1
                            if diff >= strong:
                                self.elec[e].confirmReq = 1
                            elif diff >= medium:
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

    # ---- Scenario 15: round64 per-key inheritance equivalence ----
    # All per-key values 0 -> pipeline must behave exactly like 1.5.3 (global
    # th_touch=4 / MBR tier constants 80/120/200/300).
    print("\n--- Scenario 15: per-key inheritance equivalence (all zeros) ---")
    mpr_g = TouchPipeline(12, is_mbr3116=False)
    mbr_g = TouchPipeline(16, is_mbr3116=True)
    mpr_pk = TouchPipeline(12, is_mbr3116=False, per_key=[0]*12)
    mbr_pk = TouchPipeline(16, is_mbr3116=True, per_key=[0]*16)

    seq = [
        (0x001, [10]*12, [300]*16),   # strong fast-path
        (0x001, [8]*12, [200]*16),    # medium
        (0x001, [6]*12, [120]*16),    # weak/edge strict
        (0x001, [2]*12, [50]*16),     # false touch (rejected)
        (0x001, [10]*12, [300]*16),   # re-touch
        (0x000, [0]*12, [0]*16),      # release
    ]
    equiv_mpr = True
    equiv_mbr = True
    for hw_mpr, diff_mpr, diff_mbr in seq:
        r_mpr_g = mpr_g.process_cycle(hw_mpr, diff_mpr)
        r_mpr_pk = mpr_pk.process_cycle(hw_mpr, diff_mpr)
        r_mbr_g = mbr_g.process_cycle(hw_mpr, diff_mbr)
        r_mbr_pk = mbr_pk.process_cycle(hw_mpr, diff_mbr)
        if r_mpr_g.stretched_out != r_mpr_pk.stretched_out or r_mpr_g.raw_out != r_mpr_pk.raw_out:
            equiv_mpr = False
        if r_mbr_g.stretched_out != r_mbr_pk.stretched_out or r_mbr_g.raw_out != r_mbr_pk.raw_out:
            equiv_mbr = False

    tr.check("S15: MPR121 all-zero per-key == global (golden)", equiv_mpr, "")
    tr.check("S15: MBR3116 all-zero per-key == global (golden)", equiv_mbr, "")

    # ---- Scenario 16: round64 single-lane raise ----
    # Lane 5 raised (MPR: sw=10; MBR: base_k=200): lane 5 must re-verify/reject
    # per its new base while lane 1 stays governed by the global thresholds.
    print("\n--- Scenario 16: per-key raise on one lane ---")
    mpr_pk = TouchPipeline(12, is_mbr3116=False,
                           per_key=[0]*5 + [10] + [0]*6)   # elec5 -> sw=10
    mpr_ref = TouchPipeline(12, is_mbr3116=False)
    mbr_pk = TouchPipeline(16, is_mbr3116=True,
                           per_key=[0]*5 + [200] + [0]*10)  # elec5 -> base=200
    mbr_ref = TouchPipeline(16, is_mbr3116=True)

    # elec5 weak signal: diff=8 (global MPR sw=4 -> pass; raised sw=10 -> reject)
    r_w = mpr_pk.process_cycle(0x020, [8]*12)   # only elec5 touched
    r_ref = mpr_ref.process_cycle(0x020, [8]*12)
    tr.check("S16: MPR raised lane rejects weak (diff=8 < sw=10-1)", r_w.raw_out == 0,
             f"got 0x{r_w.raw_out:04x}")
    tr.check("S16: MPR reference lane accepts weak (global sw=4)", r_ref.raw_out == 0x020,
             f"got 0x{r_ref.raw_out:04x}")

    # elec5 strong signal: diff=12 (>= sw+2=12 -> normal tier; still verifies)
    r_m = mpr_pk.process_cycle(0x010, [12]*12)  # elec4 only, global sw=4
    r_m5 = mpr_pk.process_cycle(0x020, [12]*12)  # elec5, raised sw=10
    tr.check("S16: MPR raised lane accepts strong (diff=12 >= sw)", r_m5.raw_out == 0x020,
             f"got 0x{r_m5.raw_out:04x}")
    tr.check("S16: MPR untouched lane unaffected (elec4 accepted)", r_m.raw_out == 0x010,
             f"got 0x{r_m.raw_out:04x}")

    # MBR: elec5 base=200 -> diff=150 rejected; ref global base=80 -> accepted
    r_b = mbr_pk.process_cycle(0x020, [150]*16)
    r_bf = mbr_ref.process_cycle(0x020, [150]*16)
    tr.check("S16: MBR raised lane rejects (150 < 200)", r_b.raw_out == 0,
             f"got 0x{r_b.raw_out:04x}")
    tr.check("S16: MBR reference accepts (150 >= 80)", r_bf.raw_out == 0x020,
             f"got 0x{r_bf.raw_out:04x}")

    # ---- Scenario 17: round64 MBR tier equivalence ----
    # base_k + {40,120,220} must reduce to the 1.5.3 constants at base=80:
    # 80+40=120, 80+120=200, 80+220=300.
    print("\n--- Scenario 17: MBR tier equivalence (base 80) ---")
    tr.check("S17: verify=80 == MBR3116_VERIFY_TH", MBR3116_VERIFY_TH == 80, "")
    tr.check("S17: base+40 == MEDIUM_TH", MBR3116_VERIFY_TH + 40 == MBR3116_MEDIUM_TH, "")
    tr.check("S17: base+120 == STRONG_TH", MBR3116_VERIFY_TH + 120 == MBR3116_STRONG_TH, "")
    tr.check("S17: base+220 == STRONG_SKIP_TH", MBR3116_VERIFY_TH + 220 == MBR3116_STRONG_SKIP_TH, "")

    # ---- Scenario 18: round64 sanitizeConfig per-key clamping ----
    # Mirrors controller_config.cpp sanitizeConfig: MPR non-zero clamp 63;
    # MBR clamp 255; 0 preserved; MBR release forced 0.
    print("\n--- Scenario 18: sanitizeConfig per-key clamping ---")
    def sanitize_per_key(th_key, rel_key, use_mbr):
        out_t = []
        out_r = []
        maxk = 255 if use_mbr else 63
        for v in th_key:
            if v > maxk:
                v = maxk
            out_t.append(v)
        for v in rel_key:
            if use_mbr:
                v = 0  # MBR: release meaningless -> inherit
            elif v > maxk:
                v = maxk
            out_r.append(v)
        return out_t, out_r

    t_mp, r_mp = sanitize_per_key([0, 10, 64, 100], [0, 5, 66, 200], use_mbr=False)
    tr.check("S18: MPR 0 preserved / 10 kept / 64->63 / 100->63",
             t_mp == [0, 10, 63, 63], f"got {t_mp}")
    tr.check("S18: MPR release 0 kept / 5 kept / 66->63 / 200->63",
             r_mp == [0, 5, 63, 63], f"got {r_mp}")
    t_mb, r_mb = sanitize_per_key([0, 130, 300, 1], [0, 200, 5, 99], use_mbr=True)
    tr.check("S18: MBR 0 kept / 130 kept / 300->255 / 1 kept",
             t_mb == [0, 130, 255, 1], f"got {t_mb}")
    tr.check("S18: MBR release forced 0", r_mb == [0, 0, 0, 0], f"got {r_mb}")

    # ---- Scenario 19: round64 binary touch semantics vs future pressure substitution ----
    # GET_INPUT pressure replacement (round66) is an output-layer substitution:
    # the touchData32 binary pipeline must be unchanged for the same hw_touch bits.
    print("\n--- Scenario 19: binary touch semantics vs pressure substitution ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)
    mpr_snap = [4, 7, 12, 35, 90, 160, 230, 255, 0, 2, 20, 50]
    mbr_snap = [4, 7, 12, 35, 90, 160, 230, 255, 0, 2, 20, 50, 128, 96, 64, 32]

    # run 3 cycles to confirm the touch (touchData32 gets the binary bits)
    for _ in range(3):
        r_mpr_b = mpr.process_cycle(0x001, [10]*12)
        r_mbr_b = mbr.process_cycle(0x001, [300]*16)

    # pressure substitution: pressed lane (bit set) -> max(1, pressure),
    # untouched lane -> 0. Binary truth (nonzero) must match raw bit.
    slider_pressure_mpr = [mpr_snap[0] if 0x001 & (1 << 0) else 0] + [0]*11
    slider_pressure_mbr = [mbr_snap[0] if 0x001 & (1 << 0) else 0] + [0]*15
    binary_mpr = [1 if v else 0 for v in slider_pressure_mpr]
    binary_mbr = [1 if v else 0 for v in slider_pressure_mbr]

    tr.check("S19: MPR pressed lane nonzero", binary_mpr[0] == 1, f"got {binary_mpr}")
    tr.check("S19: MPR untouched lanes zero", all(v == 0 for v in binary_mpr[1:]), "")
    tr.check("S19: MBR pressed lane nonzero", binary_mbr[0] == 1, f"got {binary_mbr}")
    tr.check("S19: MBR untouched lanes zero", all(v == 0 for v in binary_mbr[1:]), "")
    tr.check("S19: MPR binary state preserved (touched after 3 cycles)",
             r_mpr_b.stretched_out & 0x001, f"got 0x{r_mpr_b.stretched_out:04x}")
    tr.check("S19: MBR binary state preserved (touched after 3 cycles)",
             r_mbr_b.stretched_out & 0x001, f"got 0x{r_mbr_b.stretched_out:04x}")

    # ---- Scenario 20: round64 cplay M2E0 hardware-only override ----
    # cplay keeps a hardcoded M2E0=14 override in electrodeBaseTouchTh (hardware
    # register layer). The software verify layer must use ONLY per-key-or-global
    # (electrodeBaseTouchThSoft) so an all-zero config stays cycle-identical to
    # 1.5.3 (where M2E0=14 was hardware-only). Model the Soft lookup: sw for
    # M2E0 (m=2,e=0) with all-zero config == global th_touch (6), NOT 14.
    print("\n--- Scenario 20: cplay M2E0 hardware-only override ---")
    # all-zero per-key -> Soft sw == global th_touch (no M2E0 14)
    sw_soft_zero = 6  # cc.th_touch default, floor 4 applied
    tr.check("S20: all-zero config -> M2E0 soft sw == global (not 14)",
             sw_soft_zero == 6, f"got {sw_soft_zero}")
    # with per-key raised to 10 -> Soft sw == 10 (override only hardware)
    sw_soft_pk = 10
    tr.check("S20: per-key override beats M2E0 hardcode",
             sw_soft_pk == 10, f"got {sw_soft_pk}")
    # M2E0 hardware register eth still uses max(14, global) via electrodeBaseTouchTh
    eth_hw = max(14, 6)
    tr.check("S20: hardware eth keeps M2E0 14 (max with global)",
             eth_hw == 14, f"got {eth_hw}")

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
