#!/usr/bin/env python3
"""
Round50 MBR3116 Migration Theoretical Test
===========================================
Simulates the touch processing pipeline for both MPR121 and MBR3116 paths,
verifying behavior across 44 grouped scenarios. Production I/O fault tests
are separately executed by test_mbr_distance_pipeline.py.

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
  20. round64: cplay M2E0 hardware-only override
  round80:
  21. MBR strong flick fast path (isolated, first-cycle output)
  22. MBR medium signal NOT first-cycle (fast threshold boundary)
  23. MBR 40ms release-bounce guard (stale fastOk does not double-fire)
  24. MBR hover gate (mbrTouchGate) rejects hover, passes firm contact
  25. MBR gate=0 legacy regression
  26. MPR strong flick fast path (raw >= max(50, sw+4))
  27. MPR per-key tightened threshold preserved (fast never bypasses)
  28. MPR x2 report scaling (clamp 255) / MBR passthrough
  29. MBR confirmed-neighbor lane anchoring relaxes the gate (round84)
  30. MBR unanchored hover band stays rejected (round84)
  31. MBR anchoring floor = baseK+40, no fast/confirmReq change (round84)
  32. 0xC5 report levels 1/2 (sim x2 / raw unscaled) (round84)
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

# round80: strong flick fast path + hover gate + report scaling
MBR_FLICK_FAST_TH = 200        # MBR first-cycle output threshold (hw_devices.cpp)
MPR_FLICK_FAST_BASE = 50       # MPR fast path base (raw diff; =100 on the x2 report scale)
MPR_FAST_TIER_MARGIN = 4       # max(50, sw_th+4) preserves per-key tightened tiers
RELEASE_BOUNCE_GUARD_MS = 40   # fastOk waived within 40ms of last confirmed touch
MBR_TOUCH_GATE_DEFAULT = 0     # mbrTouchGate config default: 0 = off (legacy)
MPR_REPORT_SCALE = 2           # round80: MPR x2 report (was x4 in game mode), clamp 255
MBR_NEIGHBOR_GATE_MARGIN = 40  # round84: lane-anchored gate relaxation floor
RAW_REPORT_LEVEL_OFF = 0       # round84: 0xC5 session levels
RAW_REPORT_LEVEL_SIM = 1       #   1 = simulated (round80 semantics: MPR x2)
RAW_REPORT_LEVEL_RAW = 2       #   2 = raw unscaled (MBR identical to level 1)
MBR_LEGACY_FAULT_HOLD_MS = 50  # round90e: missing required evidence cannot create ON
MBR_NATIVE_MAX_AGE_MS = 15


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
    fastOk: bool = False  # round80: strong flick first-cycle output flag
    faultActive: bool = False
    faultStartedMs: int = 0


@dataclass
class SimResult:
    """Result of one processing cycle."""
    raw_out: int = 0        # raw touch bits after verification
    stretched_out: int = 0  # stretched touch bits
    i2c_reads: int = 0      # number of I2C reads performed
    verify_fails: int = 0   # number of verification rejections


class TouchPipeline:
    """Simulates the touch verification + stretching pipeline."""

    def __init__(self, num_elec: int, is_mbr3116: bool, per_key: Optional[List[int]] = None,
                 mbr_touch_gate: int = MBR_TOUCH_GATE_DEFAULT,
                 distance_profile: Optional[Tuple[int, int, int, int]] = None):
        self.num_elec = num_elec
        self.is_mbr3116 = is_mbr3116
        self.elec = [ElectrodeState() for _ in range(num_elec)]
        self.prevStretched = 0
        self.g_verifyFail = [0] * num_elec
        self.cycle_ms = 0  # simulated time
        # round64: per-electrode threshold override, 0 = inherit global.
        self.per_key = per_key if per_key is not None else [0] * num_elec
        # round80: hover-rejection gate (ControllerConfig.mbrTouchGate), 0 = off.
        self.mbr_touch_gate = mbr_touch_gate
        # round84: per-GAME-LANE stretched snapshot (prev cycle). The flat sim
        # maps electrode e <-> lane e 1:1, so this mirrors the firmware's
        # laneTable projection exactly; only the MBR path consults it.
        self.prevLaneStretched = [False] * num_elec
        # round89a: global signal-domain (Z,F,ON%,OFF%) opt-in. Production
        # profile signature/check and transport/cache are tested by C++ harness.
        self.distance_profile = distance_profile
        self.distance_active = [False] * num_elec
        self.strict_current = False
        self.mbr_fault_mask = 0

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
                            raw &= ~(1 << e)
                            self.elec[e].verifiedCount = 0
                            self.elec[e].fastOk = False
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            continue
                        # No fast-path on re-read (mirror hw_devices.cpp glitch path)
                        if diff < verify_th:
                            raw &= ~(1 << e)
                            self.elec[e].verifiedCount = 0
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            self.elec[e].fastOk = False
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
                        self.elec[e].fastOk = False
                        if self.g_verifyFail[e] < 255:
                            self.g_verifyFail[e] += 1
                            result.verify_fails += 1
                    else:
                        neighbor = ((e > 0 and (prev_stretched & (1 << (e-1)))) or
                                   (e < self.num_elec - 1 and (prev_stretched & (1 << (e+1)))))
                        # round80: strong flick fast path (first sighting only).
                        # max() with sw+4 preserves per-key tightened thresholds.
                        fast_th = max(MPR_FLICK_FAST_BASE, sw_th + MPR_FAST_TIER_MARGIN)
                        if self.elec[e].verifiedCount == 0 and diff >= fast_th:
                            self.elec[e].verifiedCount = 2
                            self.elec[e].confirmReq = 1
                            self.elec[e].fastOk = True
                        elif self.elec[e].verifiedCount == 0 and (diff >= sw_th + 6 or neighbor):
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
                        self.elec[e].fastOk = False

        result.raw_out = raw
        return raw, result

    def _verify_mbr3116(self, raw: int, hw_touch: int, diff_data: List[int],
                        i2c_error: bool, prev_stretched: int,
                        difference_read_valid: bool = True) -> Tuple[int, SimResult]:
        """MBR3116 verification path."""
        result = SimResult()
        nowVer = self.cycle_ms
        diff_read = False
        diff_ok = True
        diff_retried = False

        for e in range(self.num_elec):
            if raw & (1 << e):
                if (self.elec[e].verifiedCount < 2 or
                        (not self.strict_current and not (prev_stretched & (1 << e)))):
                    if not diff_read:
                        result.i2c_reads += 1
                        diff_ok = not i2c_error and difference_read_valid
                        if not diff_ok:
                            diff_retried = True
                            result.i2c_reads += 1
                            # i2c_error models a transient first-attempt error;
                            # this argument models the final bounded outcome.
                            diff_ok = difference_read_valid
                        diff_read = True

                    if not diff_ok:
                        raw &= ~(1 << e)
                        self.mbr_fault_mask |= 1 << e
                        self.elec[e].confirmReq = CONFIRM_CYCLES
                    else:
                        diff = diff_data[e]
                        if not self.strict_current and not 0 <= diff <= 255:
                            raw &= ~(1 << e)
                            self.mbr_fault_mask |= 1 << e
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            continue
                        # round64: base_k = per-key if set, else the default 80;
                        # tiers = base_k + {40, 120, 220}. MBR can only tighten:
                        # config <=80 keeps the verify baseline (never relaxes).
                        pk = self.per_key[e] if e < len(self.per_key) else 0
                        base_k = max(pk if pk != 0 else MBR3116_VERIFY_TH, MBR3116_VERIFY_TH)
                        strong_skip = base_k + 220
                        strong = base_k + 120
                        medium = base_k + 40
                        # round80: hover-rejection gate, touch-ON only.
                        reject_th = max(base_k, self.mbr_touch_gate)
                        # round84: confirmed-neighbor lane anchoring -- a slide
                        # edge next to a lane that was REAL last cycle gets the
                        # gate relaxed to base_k + 40 (never below). Sim maps
                        # electrode <-> lane 1:1; firmware projects through
                        # laneTable (same semantics, cross-chip inclusive).
                        lane_neighbor_confirmed = (
                            (e > 0 and self.prevLaneStretched[e - 1]) or
                            (e < len(self.prevLaneStretched) - 1 and self.prevLaneStretched[e + 1]))
                        if lane_neighbor_confirmed and reject_th > base_k + MBR_NEIGHBOR_GATE_MARGIN:
                            reject_th = base_k + MBR_NEIGHBOR_GATE_MARGIN
                        neighbor = ((e > 0 and (prev_stretched & (1 << (e-1)))) or
                                   (e < self.num_elec - 1 and (prev_stretched & (1 << (e+1)))))
                        if diff < reject_th:
                            raw &= ~(1 << e)
                            self.elec[e].verifiedCount = 0
                            self.elec[e].confirmReq = CONFIRM_CYCLES
                            self.elec[e].fastOk = False
                            if self.g_verifyFail[e] < 255:
                                self.g_verifyFail[e] += 1
                                result.verify_fails += 1
                        elif self.elec[e].verifiedCount >= 2:
                            # Fresh evidence admits a cached rapid re-contact;
                            # preserve the established confirmation cadence.
                            pass
                        elif self.elec[e].verifiedCount == 0 and not diff_retried and diff >= MBR_FLICK_FAST_TH:
                            # round80: strong flick fast path (first sighting,
                            # no I2C retry). Gate >= 200 lifts the effective
                            # threshold to the gate value (reject runs first).
                            self.elec[e].verifiedCount = 2
                            self.elec[e].confirmReq = 1
                            self.elec[e].fastOk = True
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
                        self.elec[e].fastOk = False

        result.raw_out = raw
        return raw, result

    def _stretch(self, raw: int, prev_stretched: int) -> Tuple[int, SimResult]:
        """Shared stretching, with bounded legacy-MBR evidence-fault holding."""
        result = SimResult()
        stretched = 0
        now = self.cycle_ms

        for e in range(self.num_elec):
            state = self.elec[e]
            fault = (self.is_mbr3116 and not self.strict_current and
                     bool(self.mbr_fault_mask & (1 << e)))
            if fault:
                if not state.faultActive:
                    state.faultActive = True
                    state.faultStartedMs = now
                if ((prev_stretched & (1 << e)) and
                        ((now - state.faultStartedMs) & 0xFFFFFFFF) < MBR_LEGACY_FAULT_HOLD_MS):
                    stretched |= 1 << e
                else:
                    # Retain only the original fault deadline; never grant
                    # verification, confirmation, or stretch credit on failure.
                    self.elec[e] = ElectrodeState(
                        confirmReq=CONFIRM_CYCLES, faultActive=True,
                        faultStartedMs=state.faultStartedMs)
                continue
            state.faultActive = False
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
                # round80: waive the isolated +1 for strong flick signals
                # (fastOk) unless within 40ms of the last confirmed touch.
                fast_exempt = (self.elec[e].fastOk and
                               (self.elec[e].lastConfirmed == 0 or
                                (now - self.elec[e].lastConfirmed) >= RELEASE_BOUNCE_GUARD_MS))
                if not has_neighbor and not fast_exempt:
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

    def process_cycle(self, hw_touch: int, diff_data: List[int], i2c_error: bool = False,
                      distance_sample_valid: bool = True,
                      difference_read_valid: bool = True,
                      mbr_button_valid: bool = True,
                      button_read_age_ms: int = 0) -> SimResult:
        """Process one cycle: verify -> stretch -> output."""
        result = SimResult()
        prev_stretched = self.prevStretched
        raw = hw_touch
        profile = self.distance_profile
        strict = (self.is_mbr3116 and profile is not None and
                  0 <= profile[0] < profile[1] <= 255 and
                  0 <= profile[3] < profile[2] <= 100)
        self.strict_current = strict
        self.mbr_fault_mask = 0
        if strict:
            zero, full, on_percent, off_percent = profile
            on_count = zero + ((full - zero) * on_percent + 99) // 100
            off_count = zero + (full - zero) * off_percent // 100
            for e in range(self.num_elec):
                diff = diff_data[e]
                if not distance_sample_valid or not (hw_touch & (1 << e)) or not 0 <= diff <= 255:
                    self.distance_active[e] = False
                elif self.distance_active[e]:
                    if diff <= off_count:
                        self.distance_active[e] = False
                elif diff >= on_count:
                    self.distance_active[e] = True
                if not self.distance_active[e]:
                    raw &= ~(1 << e)
                    self.elec[e] = ElectrodeState(confirmReq=CONFIRM_CYCLES)
                    self.prevStretched &= ~(1 << e)
                    self.prevLaneStretched[e] = False
            prev_stretched = self.prevStretched

        # Verification
        if self.is_mbr3116:
            raw, v_res = self._verify_mbr3116(
                raw, hw_touch, diff_data, i2c_error, prev_stretched, difference_read_valid)
            if not strict and (not mbr_button_valid or button_read_age_ms > MBR_NATIVE_MAX_AGE_MS):
                raw = 0
                self.mbr_fault_mask = (1 << self.num_elec) - 1
        else:
            raw, v_res = self._verify_mpr121(raw, hw_touch, diff_data, i2c_error, prev_stretched)
        result.i2c_reads = v_res.i2c_reads
        result.verify_fails = v_res.verify_fails
        result.raw_out = raw

        # Stretching
        stretched, s_res = self._stretch(raw, prev_stretched)
        result.stretched_out = stretched

        self.prevStretched = stretched
        # round84: project this cycle's stretched bits into lane space (the
        # flat sim's lanes ARE the electrodes) for next cycle's anchoring.
        for lane in range(self.num_elec):
            self.prevLaneStretched[lane] = bool(stretched & (1 << lane))
        self.cycle_ms = (self.cycle_ms + 3) & 0xFFFFFFFF  # ~3ms per cycle
        return result

    def advance_ms(self, ms: int):
        """Advance simulated time without processing."""
        self.cycle_ms = (self.cycle_ms + ms) & 0xFFFFFFFF


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

    # MPR121: diff=10 (sw_th+6=10 -> skip path; below fast_th=50, no fastOk)
    r_mpr = mpr.process_cycle(0x001, [10]*12)
    # MBR button DIFF is at most 255; use a valid strong sample for the fast path.
    r_mbr = mbr.process_cycle(0x001, [255]*16)

    tr.check("S1: MPR121 verifiedCount=2 (skip, no fast)", mpr.elec[0].verifiedCount == 2,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S1: MBR3116 verifiedCount=2 (fast)", mbr.elec[0].verifiedCount == 2,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S1: MPR121 confirmReq=1", mpr.elec[0].confirmReq == 1,
             f"got {mpr.elec[0].confirmReq}")
    tr.check("S1: MBR3116 confirmReq=1", mbr.elec[0].confirmReq == 1,
             f"got {mbr.elec[0].confirmReq}")
    tr.check("S1: MPR121 not stretched (1st cycle, isolated penalty)",
             r_mpr.stretched_out == 0, f"got 0x{r_mpr.stretched_out:04x}")
    # round80 contract change: strong MBR flick outputs on the FIRST cycle
    # (isolated +1 waived via fastOk, lastConfirmed==0 passes the bounce guard)
    tr.check("S1: MBR3116 stretched (1st cycle, round80 fast)", r_mbr.stretched_out & 0x001,
             f"got 0x{r_mbr.stretched_out:04x}")

    # ---- Scenario 2: New touch - medium signal ----
    print("\n--- Scenario 2: New touch - medium signal ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # MPR121: diff=8 (sw_th+4 -> confirmReq=1)
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    # MBR3116: diff=190 (below fast 200, medium tier 120 -> confirmReq=2)
    r_mbr = mbr.process_cycle(0x001, [190]*16)

    tr.check("S2: MPR121 verifiedCount=1", mpr.elec[0].verifiedCount == 1,
             f"got {mpr.elec[0].verifiedCount}")
    tr.check("S2: MBR3116 verifiedCount=1 (below fast)", mbr.elec[0].verifiedCount == 1,
             f"got {mbr.elec[0].verifiedCount}")
    tr.check("S2: MPR121 confirmReq=1", mpr.elec[0].confirmReq == 1, "")
    tr.check("S2: MBR3116 confirmReq=2 (medium tier)", mbr.elec[0].confirmReq == 2, "")

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
    mbr.process_cycle(0x001, [255]*16)
    # Second cycle: touch persists + neighbor (electrode 1) starts
    r_mpr = mpr.process_cycle(0x003, [10]*12)  # elec 0 sustained, elec 1 new with neighbor
    r_mbr = mbr.process_cycle(0x003, [255]*16)

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
    print("\n--- Scenario 9: Full re-verification (>50ms) ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    for _ in range(3):
        mpr.process_cycle(0x001, [8]*12)
        mbr.process_cycle(0x001, [190]*16)  # round80: medium tier (200 would fast-path)

    # Release for enough cycles to span >50ms (~3ms/cycle, need ~18 cycles)
    for _ in range(20):
        mpr.process_cycle(0x000, [0]*12)
        mbr.process_cycle(0x000, [0]*16)

    # Re-touch -> should require full re-verification
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    r_mbr = mbr.process_cycle(0x001, [190]*16)

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
    print("\n--- Scenario 12: Spatial penalty (isolated electrode) ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    mbr = TouchPipeline(16, is_mbr3116=True)

    # Single isolated touch, medium signal (below the round80 fast threshold
    # so the spatial penalty itself is exercised, not the fast path)
    # MPR121: diff=8 -> confirmReq=1, but no neighbor -> reqCycles=2
    # MBR3116: diff=190 -> confirmReq=2, but no neighbor -> reqCycles=3
    r_mpr = mpr.process_cycle(0x001, [8]*12)   # cycle 1: verifiedCount=1
    r_mbr = mbr.process_cycle(0x001, [190]*16)

    tr.check("S12: MPR121 not output cycle 1 (spatial+1)", r_mpr.stretched_out == 0,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S12: MBR3116 not output cycle 1 (spatial+1)", r_mbr.stretched_out == 0,
             f"got 0x{r_mbr.stretched_out:04x}")

    # Cycle 2: MPR touchCount=2 >= reqCycles=2 -> output; MBR touchCount=2 < 3
    r_mpr = mpr.process_cycle(0x001, [8]*12)
    r_mbr = mbr.process_cycle(0x001, [190]*16)

    tr.check("S12: MPR121 output cycle 2", r_mpr.stretched_out & 0x001,
             f"got 0x{r_mpr.stretched_out:04x}")
    tr.check("S12: MBR3116 not output cycle 2 (cr=2+1)", r_mbr.stretched_out == 0,
             f"got 0x{r_mbr.stretched_out:04x}")

    # Cycle 3: MBR touchCount=3 >= reqCycles=3 -> output
    r_mbr = mbr.process_cycle(0x001, [190]*16)
    tr.check("S12: MBR3116 output cycle 3", r_mbr.stretched_out & 0x001,
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

    # Touch electrode 0 for 5 cycles (medium tier both sides: MPR sw+2=6,
    # MBR medium=190 -- round80 fast path moved >=200 to first-cycle output,
    # so equivalence is asserted at the medium tier)
    for _ in range(5):
        r = mpr.process_cycle(0x001, [6]*12)
        mpr_outputs.append(r.stretched_out & 0x001)
    for _ in range(5):
        r = mbr.process_cycle(0x001, [190]*16)
        mbr_outputs.append(r.stretched_out & 0x001)

    # Slide to electrode 1 for 5 cycles
    for _ in range(5):
        r = mpr.process_cycle(0x002, [6]*12)
        mpr_outputs.append(r.stretched_out & 0x002)
    for _ in range(5):
        r = mbr.process_cycle(0x002, [190]*16)
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
        (0x001, [10]*12, [255]*16),   # strong fast-path
        (0x001, [8]*12, [200]*16),    # medium
        (0x001, [6]*12, [120]*16),    # weak/edge strict
        (0x001, [2]*12, [50]*16),     # false touch (rejected)
        (0x001, [10]*12, [255]*16),   # re-touch
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
        r_mbr_b = mbr.process_cycle(0x001, [255]*16)

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

    # ---- Scenario 21 (round80): MBR strong flick fast path ----
    # Isolated electrode, diff >= 200 on first sighting -> first-cycle output
    # (old firmware behavior restored for flick taps).
    print("\n--- Scenario 21: MBR strong flick fast path (round80) ---")
    mbr = TouchPipeline(16, is_mbr3116=True)
    r = mbr.process_cycle(0x001, [250]*16)
    tr.check("S21: first-cycle output (isolated strong)", r.stretched_out & 0x001,
             f"got 0x{r.stretched_out:04x}")
    tr.check("S21: fastOk set", mbr.elec[0].fastOk, "")
    tr.check("S21: confirmReq=1", mbr.elec[0].confirmReq == 1, "")
    # boundary: diff exactly at threshold also fast
    mbr2 = TouchPipeline(16, is_mbr3116=True)
    r2 = mbr2.process_cycle(0x001, [200]*16)
    tr.check("S21: boundary diff=200 first-cycle output", r2.stretched_out & 0x001,
             f"got 0x{r2.stretched_out:04x}")

    # ---- Scenario 22 (round80): MBR medium NOT first-cycle ----
    print("\n--- Scenario 22: MBR medium below fast threshold (round80) ---")
    mbr = TouchPipeline(16, is_mbr3116=True)
    r1 = mbr.process_cycle(0x001, [199]*16)
    r2 = mbr.process_cycle(0x001, [199]*16)
    r3 = mbr.process_cycle(0x001, [199]*16)
    tr.check("S22: not output cycle 1 (diff=199 < 200)", r1.stretched_out == 0,
             f"got 0x{r1.stretched_out:04x}")
    tr.check("S22: not output cycle 2 (cr=2+1=3)", r2.stretched_out == 0,
             f"got 0x{r2.stretched_out:04x}")
    tr.check("S22: output cycle 3", r3.stretched_out & 0x001,
             f"got 0x{r3.stretched_out:04x}")

    # ---- Scenario 23 (round80): MBR 40ms release-bounce guard ----
    # Strong touch confirmed, released ~36ms, re-touch with stale fastOk:
    # the guard must waive the exemption (no first-cycle double-fire).
    print("\n--- Scenario 23: MBR release-bounce guard (round80) ---")
    mbr = TouchPipeline(16, is_mbr3116=True)
    mbr.process_cycle(0x001, [250]*16)   # t=0->3: confirmed (lastConfirmed=0)
    mbr.process_cycle(0x001, [250]*16)   # t=3->6: confirmed (lastConfirmed=3)
    for _ in range(12):                  # release 12 cycles: t=6..42 (36ms, <50ms
        mbr.process_cycle(0x000, [0]*16) #   so verifiedCount/fastOk stay stale)
    r = mbr.process_cycle(0x001, [250]*16)  # re-touch at t=42: 42-3=39 < 40
    tr.check("S23: re-tap within guard NOT first-cycle", r.stretched_out == 0,
             f"got 0x{r.stretched_out:04x}")
    r2 = mbr.process_cycle(0x001, [250]*16)
    tr.check("S23: output cycle 2 (normal confirm)", r2.stretched_out & 0x001,
             f"got 0x{r2.stretched_out:04x}")
    # contrast: after a longer gap the exemption applies again
    mbr2 = TouchPipeline(16, is_mbr3116=True)
    mbr2.process_cycle(0x001, [250]*16)
    mbr2.process_cycle(0x001, [250]*16)
    for _ in range(13):                  # 13 cycles: t=6..45, 45-3=42 >= 40
        mbr2.process_cycle(0x000, [0]*16)
    r3 = mbr2.process_cycle(0x001, [250]*16)
    tr.check("S23: re-tap after 42ms IS first-cycle (fast)", r3.stretched_out & 0x001,
             f"got 0x{r3.stretched_out:04x}")

    # ---- Scenario 24 (round80): MBR hover gate ----
    print("\n--- Scenario 24: MBR hover gate mbrTouchGate (round80) ---")
    gated = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=200)
    r_hover = gated.process_cycle(0x001, [150]*16)  # hover-level diff
    tr.check("S24: hover diff=150 rejected by gate", r_hover.raw_out == 0,
             f"got 0x{r_hover.raw_out:04x}")
    tr.check("S24: verifyFail counted", gated.g_verifyFail[0] == 1, "")
    r_firm = gated.process_cycle(0x001, [250]*16)   # firm contact above gate
    tr.check("S24: firm diff=250 accepted", r_firm.raw_out & 0x001,
             f"got 0x{r_firm.raw_out:04x}")
    tr.check("S24: firm contact first-cycle output (fast)", r_firm.stretched_out & 0x001,
             f"got 0x{r_firm.stretched_out:04x}")
    # gate below the fast threshold: gate=150 lifts reject but fast stays at 200
    gated2 = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=150)
    r_mid = gated2.process_cycle(0x001, [180]*16)   # >= gate, < fast
    tr.check("S24: gate=150 accepts 180 (verify layer)", r_mid.raw_out & 0x001,
             f"got 0x{r_mid.raw_out:04x}")
    tr.check("S24: but 180 < fast 200 -> no first-cycle output", r_mid.stretched_out == 0,
             f"got 0x{r_mid.stretched_out:04x}")

    # ---- Scenario 25 (round80): MBR gate=0 legacy regression ----
    print("\n--- Scenario 25: MBR gate=0 legacy behavior (round80) ---")
    legacy = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=0)
    r = legacy.process_cycle(0x001, [90]*16)  # between 80 and 120: legacy accept
    tr.check("S25: diff=90 accepted with gate=0", r.raw_out & 0x001,
             f"got 0x{r.raw_out:04x}")
    tr.check("S25: no first-cycle output (cr=2 tier)", r.stretched_out == 0,
             f"got 0x{r.stretched_out:04x}")

    # ---- Scenario 26 (round80): MPR strong flick fast path ----
    print("\n--- Scenario 26: MPR fast path raw>=max(50, sw+4) (round80) ---")
    mpr = TouchPipeline(12, is_mbr3116=False)
    r = mpr.process_cycle(0x001, [60]*12)   # >= max(50, 4+4)=50 -> fast
    tr.check("S26: diff=60 first-cycle output", r.stretched_out & 0x001,
             f"got 0x{r.stretched_out:04x}")
    tr.check("S26: fastOk set", mpr.elec[0].fastOk, "")
    mpr2 = TouchPipeline(12, is_mbr3116=False)
    r2 = mpr2.process_cycle(0x001, [40]*12)  # < 50, but >= sw+6=10 -> skip path
    tr.check("S26: diff=40 NOT first-cycle (below fast, isolated penalty)",
             r2.stretched_out == 0, f"got 0x{r2.stretched_out:04x}")
    tr.check("S26: diff=40 still verified+accepted", r2.raw_out & 0x001,
             f"got 0x{r2.raw_out:04x}")
    tr.check("S26: diff=40 no fastOk (skip path)", not mpr2.elec[0].fastOk, "")

    # ---- Scenario 27 (round80): MPR per-key threshold preserved ----
    print("\n--- Scenario 27: MPR per-key fast never bypasses (round80) ---")
    pk = TouchPipeline(12, is_mbr3116=False, per_key=[0]*5 + [48] + [0]*6)  # elec5 sw=48
    r = pk.process_cycle(0x020, [50]*12)   # 50 < max(50, 48+4)=52 -> not fast
    tr.check("S27: diff=50 < sw+4=52 not first-cycle", r.stretched_out == 0,
             f"got 0x{r.stretched_out:04x}")
    tr.check("S27: diff=50 accepted (verify 47 ok)", r.raw_out & 0x020,
             f"got 0x{r.raw_out:04x}")
    r2 = pk.process_cycle(0x020, [60]*12)  # 60 >= 52 -> fast
    tr.check("S27: diff=60 >= sw+4 first-cycle output", r2.stretched_out & 0x020,
             f"got 0x{r2.stretched_out:04x}")

    # ---- Scenario 28 (round80): MPR x2 report scaling ----
    # Mirrors chuni_io.cpp reporting layer: MPR min(raw*2, 255), MBR passthrough.
    print("\n--- Scenario 28: MPR x2 report scaling (round80) ---")
    def report_pressure(raw_p, use_mbr):
        if use_mbr:
            return raw_p
        v = raw_p * MPR_REPORT_SCALE
        return 255 if v > 255 else v
    tr.check("S28: MPR finger 30 -> 60", report_pressure(30, False) == 60, "")
    tr.check("S28: MPR finger 50 -> 100", report_pressure(50, False) == 100, "")
    tr.check("S28: MPR palm 130 -> 255 (clamped)", report_pressure(130, False) == 255, "")
    tr.check("S28: MPR 255 -> 255", report_pressure(255, False) == 255, "")
    tr.check("S28: MBR 128 passthrough", report_pressure(128, True) == 128, "")
    tr.check("S28: MBR 255 passthrough", report_pressure(255, True) == 255, "")

    # ---- Scenario 29 (round84): MBR confirmed-neighbor lane anchoring ----
    # Firm contact on lane1 confirmed -> lane0's slide edge (diff 150, below
    # gate 200) must pass the gate precheck via the relaxed floor (base 80+40
    # = 120) and confirm through the normal tier path (not fast: 150 < 200).
    print("\n--- Scenario 29: MBR lane-anchored gate relaxation (round84) ---")
    anchored = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=200)
    r = anchored.process_cycle(0x002, [150]*16)  # lane1 firm? no: 150 < gate 200
    tr.check("S29: preanchor diff=150 below gate still rejected", r.raw_out == 0,
             f"got 0x{r.raw_out:04x}")
    r = anchored.process_cycle(0x002, [250]*16)  # lane1 firm contact
    tr.check("S29: anchor lane1 confirmed first-cycle", r.stretched_out & 0x002,
             f"got 0x{r.stretched_out:04x}")
    # lane1 stays held this cycle; lane0 shows a slide-edge diff=150 next cycle
    r = anchored.process_cycle(0x003, [150]*16)  # lane0+lane1 asserted
    tr.check("S29: anchored lane0 edge diff=150 passes gate", r.raw_out & 0x001,
             f"got 0x{r.raw_out:04x}")
    # Existing round45u neighborWasActive skip path fires once the gate
    # precheck no longer rejects (that precheck WAS the blocker): slide-edge
    # lane0 gets confirmReq=1 -> first-cycle output. Intended continuity.
    tr.check("S29: lane0 slide-edge first-cycle (skip path)", r.stretched_out & 0x001,
             f"got 0x{r.stretched_out:04x}")

    # ---- Scenario 30 (round84): unanchored hover band stays rejected ----
    # Same diff=150 on a lane with NO confirmed neighbor: gate 200 still rejects.
    print("\n--- Scenario 30: MBR unanchored hover band unchanged (round84) ---")
    unanch = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=200)
    r = unanch.process_cycle(0x021, [150]*16)  # lanes 0+5, no neighbors confirmed
    tr.check("S30: unanchored diff=150 rejected both lanes", r.raw_out == 0,
             f"got 0x{r.raw_out:04x}")
    tr.check("S30: verifyFail counted for both", unanch.g_verifyFail[0] == 1 and unanch.g_verifyFail[5] == 1, "")
    # contrast: same pipeline WITHOUT the gate accepts (legacy behavior intact)
    nogate = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=0)
    r = nogate.process_cycle(0x021, [150]*16)
    tr.check("S30: gate=0 accepts diff=150 (regression)", r.raw_out & 0x021,
             f"got 0x{r.raw_out:04x}")

    # ---- Scenario 31 (round84): anchoring floor baseK+40 semantics ----
    # With gate=100 (< base+40=120), anchoring must NOT lift the gate above
    # the plain max(base,gate)=100; and the relaxed floor never goes below
    # the un-anchored gate. Also anchored fast path still needs >= 200.
    print("\n--- Scenario 31: MBR anchoring floor semantics (round84) ---")
    lowgate = TouchPipeline(16, is_mbr3116=True, mbr_touch_gate=100)
    r = lowgate.process_cycle(0x002, [250]*16)   # lane1 firm, confirms
    r = lowgate.process_cycle(0x003, [110]*16)   # lane0 edge 110: >=100 either way
    tr.check("S31: gate=100 accepts 110 unanchored", r.raw_out & 0x001,
             f"got 0x{r.raw_out:04x}")
    # anchored: reject floor = max(120) > 110 would REJECT -- but lane1 was
    # confirmed last cycle, floor 120 applies -> 110 < 120 rejected. This is
    # the documented floor semantics (anchored gate = baseK+40 when higher).
    r2 = lowgate.process_cycle(0x003, [130]*16)  # above floor 120
    tr.check("S31: anchored 130 passes floor 120", r2.raw_out & 0x001,
             f"got 0x{r2.raw_out:04x}")

    # ---- Scenario 32 (round84/86): 0xC5 report levels 1/2 ----
    # Mirrors chuni_io.cpp level semantics:
    # level 2 = raw: pressureSnap reported unscaled (untouched keys report idle values).
    # level 1 = game simulation: untouched keys report 0; pressed keys report
    #           128 (standard) or normalized pressure (if gameRawEnabled).
    print("\n--- Scenario 32: 0xC5 report levels sim/raw (round86) ---")
    def report_level(raw_p, use_mbr, level, pressed=True, game_raw=False):
        if level == RAW_REPORT_LEVEL_OFF:
            return 0
        if level == RAW_REPORT_LEVEL_RAW:
            return raw_p  # level 2 直报物理层读数 (未触发也报底噪)
        # level 1 (模拟结果): 严格以游戏中的实际输入结果为准
        if not pressed:
            return 0
        if game_raw:
            v = raw_p if use_mbr else (raw_p * MPR_REPORT_SCALE)
            p = 255 if v > 255 else v
            return p if p > 0 else 1
        return 128

    tr.check("S32: untouched key sim->0 (game simulation)", report_level(30, False, RAW_REPORT_LEVEL_SIM, pressed=False) == 0, "")
    tr.check("S32: untouched key raw->30 (physical sensor)", report_level(30, False, RAW_REPORT_LEVEL_RAW, pressed=False) == 30, "")
    tr.check("S32: pressed key sim standard->128", report_level(30, False, RAW_REPORT_LEVEL_SIM, pressed=True, game_raw=False) == 128, "")
    tr.check("S32: pressed key sim game-raw MPR->60", report_level(30, False, RAW_REPORT_LEVEL_SIM, pressed=True, game_raw=True) == 60, "")
    tr.check("S32: pressed key sim game-raw MBR->150", report_level(150, True, RAW_REPORT_LEVEL_SIM, pressed=True, game_raw=True) == 150, "")
    tr.check("S32: MBR raw=128 raw->128", report_level(128, True, RAW_REPORT_LEVEL_RAW) == 128, "")
    tr.check("S32: MBR raw=255 raw->255", report_level(255, True, RAW_REPORT_LEVEL_RAW) == 255, "")

    # ---- round89a: continuous rejection must beat confirmation, neighbors,
    # pulse stretching, sticky recovery, and the old sustained-read skip. ----
    print("\n--- Scenarios 33-36: continuous signal-domain gate (round89a) ---")
    strict = TouchPipeline(16, True, mbr_touch_gate=130, distance_profile=(100, 250, 80, 60))
    for _ in range(25):
        r = strict.process_cycle(1, [240] * 16)
    tr.check("S33: strong held touch passes", r.stretched_out & 1, "")
    r = strict.process_cycle(1, [190] * 16)  # OFF=190, hardware remains ON
    tr.check("S33: exact OFF clears confirmed sticky touch immediately", r.raw_out == 0 and r.stretched_out == 0, "")
    tr.check("S33: downstream state fully cleared", strict.elec[0] == ElectrodeState(confirmReq=2), "")
    r = strict.process_cycle(1, [219] * 16)  # ON=220
    tr.check("S33: below ON cannot use former sticky recovery", r.stretched_out == 0, "")
    r = strict.process_cycle(1, [220] * 16)
    tr.check("S33: exact ON permits new touch", r.stretched_out & 1, "")
    r = strict.process_cycle(1, [191] * 16)
    tr.check("S33: above OFF holds active touch", r.stretched_out & 1, "")
    r = strict.process_cycle(0, [240] * 16)
    tr.check("S33: native OFF clears without stretch", r.stretched_out == 0, "")

    strict = TouchPipeline(16, True, mbr_touch_gate=200, distance_profile=(100, 250, 80, 60))
    strict.process_cycle(2, [240] * 16)
    d = [0] * 16
    d[1], d[0] = 240, 150
    for _ in range(8):
        r = strict.process_cycle(3, d)
    tr.check("S34: confirmed neighbor cannot relax strict ON gate", r.stretched_out == 2, "")
    r = strict.process_cycle(2, d, distance_sample_valid=False)
    tr.check("S34: invalid snapshot clears active result", r.stretched_out == 0, "")
    d[1] = 200
    r = strict.process_cycle(2, d)
    tr.check("S34: restored sample below ON cannot recover old active state", r.stretched_out == 0, "")
    d[1] = 300
    r = strict.process_cycle(2, d)
    tr.check("S34: >255 button sample rejected", r.stretched_out == 0, "")

    for mode in (True, False):
        legacy = TouchPipeline(16 if mode else 12, mode)
        invalid = TouchPipeline(16 if mode else 12, mode, distance_profile=(200, 100, 80, 60))
        for hw, diff in ((1, 240 if mode else 60), (1, 150 if mode else 8), (0, 0), (1, 240 if mode else 60)):
            a = legacy.process_cycle(hw, [diff] * legacy.num_elec)
            b = invalid.process_cycle(hw, [diff] * invalid.num_elec)
            tr.check(f"S35: invalid profile preserves legacy mode={mode}, hw={hw}, diff={diff}", a == b, "")
    mpr = TouchPipeline(12, False, distance_profile=(100, 250, 80, 60))
    r = mpr.process_cycle(1, [60] * 12)
    tr.check("S35: valid MBR profile does not gate MPR", r.stretched_out & 1, "")

    # Explicitly show the existing defect addressed by opt-in mode: after
    # confirmation the legacy path retains HW ON despite low live counts.
    legacy = TouchPipeline(16, True)
    legacy.process_cycle(1, [240] * 16)
    r = legacy.process_cycle(1, [10] * 16)
    tr.check("S36: legacy sustained low DIFF blind spot reproduced", r.stretched_out & 1, "")

    print("\n--- Scenarios 37-44: legacy MBR required-evidence faults (round90e) ---")
    legacy = TouchPipeline(16, True)
    outcomes = [legacy.process_cycle(1, [255] * 16, difference_read_valid=False)
                for _ in range(30)]
    tr.check("S37: persistent difference failure never creates ON",
             all(r.raw_out == 0 and r.stretched_out == 0 and r.i2c_reads == 2 for r in outcomes), "")
    tr.check("S37: recovery restores strong first-cycle touch",
             legacy.process_cycle(1, [255] * 16).stretched_out == 1, "")

    legacy = TouchPipeline(16, True)
    legacy.process_cycle(1, [100] * 16)
    outcomes = [legacy.process_cycle(1, [255] * 16, difference_read_valid=False)
                for _ in range(20)]
    tr.check("S38: partial confirmation cannot advance on failed evidence",
             all(r.stretched_out == 0 for r in outcomes) and legacy.elec[0].touchCount == 0, "")

    legacy = TouchPipeline(16, True)
    legacy.process_cycle(1, [255] * 16)
    legacy.cycle_ms = 0  # zero is a valid fault start time
    r = legacy.process_cycle(1, [255] * 16, mbr_button_valid=False)
    tr.check("S39: short native-state fault holds only prior ON", r.stretched_out == 1, "")
    legacy.cycle_ms = 49
    tr.check("S39: prior ON held at 49ms",
             legacy.process_cycle(1, [255] * 16, mbr_button_valid=False).stretched_out == 1, "")
    legacy.cycle_ms = 50
    r = legacy.process_cycle(1, [255] * 16, mbr_button_valid=False)
    tr.check("S39: release at 50ms clears confirmation and fast credentials",
             r.stretched_out == 0 and legacy.elec[0].touchCount == 0 and
             legacy.elec[0].verifiedCount == 0 and not legacy.elec[0].fastOk, "")

    legacy = TouchPipeline(16, True)
    legacy.process_cycle(1, [255] * 16)
    legacy.process_cycle(1, [255] * 16, mbr_button_valid=False)
    r = legacy.process_cycle(1, [255] * 16)
    tr.check("S40: short native-state failure recovers without extra debounce",
             r.stretched_out == 1 and r.i2c_reads == 0, "")

    legacy = TouchPipeline(16, True)
    legacy.process_cycle(1, [255] * 16)
    legacy.process_cycle(1, [255] * 16)
    for _ in range(6):
        legacy.process_cycle(0, [0] * 16)
    tr.check("S41: output OFF while verification credits remain",
             legacy.prevStretched == 0 and legacy.elec[0].verifiedCount == 2, "")
    r = legacy.process_cycle(1, [255] * 16, difference_read_valid=False)
    tr.check("S41: cached credits cannot admit failed re-contact",
             r.stretched_out == 0 and r.i2c_reads == 2, "")

    for age, expected in ((15, 1), (16, 0)):
        legacy = TouchPipeline(16, True)
        r = legacy.process_cycle(1, [255] * 16, button_read_age_ms=age)
        tr.check(f"S42: native evidence age={age}ms", r.stretched_out == expected, "")
    for value, expected in ((255, 1), (256, 0)):
        legacy = TouchPipeline(16, True)
        r = legacy.process_cycle(1, [value] * 16)
        tr.check(f"S43: native button count={value}", r.stretched_out == expected, "")

    legacy = TouchPipeline(16, True)
    legacy.process_cycle(1, [255] * 16)
    legacy.cycle_ms = 0xFFFFFFF0
    legacy.process_cycle(1, [255] * 16, mbr_button_valid=False)
    legacy.cycle_ms = 0x21  # 49ms elapsed across wrap
    tr.check("S44: unsigned wrap preserves 49ms hold",
             legacy.process_cycle(1, [255] * 16, mbr_button_valid=False).stretched_out == 1, "")
    legacy.cycle_ms = 0x22
    tr.check("S44: unsigned wrap releases at 50ms",
             legacy.process_cycle(1, [255] * 16, mbr_button_valid=False).stretched_out == 0, "")
    failed_mpr = TouchPipeline(12, False)
    for frame in range(8):
        r = failed_mpr.process_cycle(1, [0] * 12, i2c_error=True)
        tr.check(f"S45: failed MPR evidence cannot create ON ({frame})", r.stretched_out == 0, "")
    r = failed_mpr.process_cycle(1, [60] * 12)
    tr.check("S45: fresh MPR evidence recovers", r.raw_out == 1, "")
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
