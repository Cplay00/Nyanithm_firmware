/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. https://mozilla.org/MPL/2.0/ */
#include <mbr_history.h>
#include <pico/critical_section.h>
#include <pico/stdlib.h>
#include <pico/util/queue.h>
#include <cstring>

namespace {
queue_t frames;
critical_section_t guard;
MbrHistoryStatus status{};
MbrHistoryState state = MBR_HISTORY_OFF; // Core0 only.
uint32_t startMs, triggerMs, previousSlider;
uint16_t previousHardware[3], previousVerified[3];
uint8_t chipCount;
bool havePrevious = false;
bool allocated = false;
bool disconnected = false; // Guard-protected cancellation survives ARM setup.

void freeze(uint8_t reason, uint32_t ms) {
    state = MBR_HISTORY_FROZEN;
    critical_section_enter_blocking(&guard);
    status.state = state;
    status.reason |= reason;
    status.endedMs = ms;
    critical_section_exit(&guard);
}
}

void initMbrHistory() {
    critical_section_init(&guard);
    queue_init(&frames, sizeof(MbrTouchTrace), MBR_HISTORY_CAPACITY);
    allocated = frames.data != nullptr;
    status.tag = CMD_MBR_HISTORY_STATUS;
    status.version = MBR_HISTORY_VERSION;
    status.capacity = MBR_HISTORY_CAPACITY;
    status.preMs = MBR_HISTORY_PRE_MS;
    status.postMs = MBR_HISTORY_POST_MS;
    status.state = allocated ? MBR_HISTORY_OFF : MBR_HISTORY_FAILED;
}

bool requestMbrHistory(uint8_t command) {
    if (command != CMD_MBR_HISTORY_ARM && command != CMD_MBR_HISTORY_FREEZE) return false;
    critical_section_enter_blocking(&guard);
    bool accepted = allocated && status.pending == 0;
    if (accepted) {
        if (command == CMD_MBR_HISTORY_ARM) disconnected = false;
        status.pending = command;
    }
    critical_section_exit(&guard);
    return accepted;
}

void disconnectMbrHistory() {
    critical_section_enter_blocking(&guard);
    disconnected = true;
    if (allocated && (status.pending == CMD_MBR_HISTORY_ARM ||
        status.state == MBR_HISTORY_ARMED || status.state == MBR_HISTORY_POST)) {
        status.pending = CMD_MBR_HISTORY_FREEZE;
    }
    critical_section_exit(&guard);
}

MbrHistoryStatus getMbrHistoryStatus() {
    critical_section_enter_blocking(&guard);
    MbrHistoryStatus copy = status;
    if (allocated) copy.queued = queue_get_level(&frames);
    critical_section_exit(&guard);
    return copy;
}

bool readMbrHistory(MbrHistoryFrame& frame) {
    critical_section_enter_blocking(&guard);
    bool ready = allocated && !status.pending && status.state == MBR_HISTORY_FROZEN;
    if (ready) ready = queue_try_remove(&frames, &frame.trace);
    if (ready) {
        frame.tag = CMD_MBR_HISTORY_READ;
        frame.version = MBR_HISTORY_VERSION;
        frame.reserved[0] = frame.reserved[1] = 0;
        frame.session = status.session;
    }
    critical_section_exit(&guard);
    return ready;
}

void recordMbrHistory(const MbrTouchTrace& trace) {
    const uint32_t hookStart = time_us_32();
    critical_section_enter_blocking(&guard);
    const uint8_t command = status.pending;
    critical_section_exit(&guard);
    if (!allocated) return;
    if (command == CMD_MBR_HISTORY_ARM) {
        while (queue_try_remove(&frames, nullptr)) {}
        startMs = trace.publishedMs;
        triggerMs = 0;
        chipCount = trace.chipCount;
        havePrevious = false;
        state = (trace.flags & MBR_TRACE_PROFILE) && chipCount >= 2 && chipCount <= 3
            ? MBR_HISTORY_ARMED : MBR_HISTORY_UNSUPPORTED;
        critical_section_enter_blocking(&guard);
        if (disconnected) state = MBR_HISTORY_FROZEN;
        const uint32_t session = status.session + 1;
        status = MbrHistoryStatus{};
        status.tag = CMD_MBR_HISTORY_STATUS;
        status.version = MBR_HISTORY_VERSION;
        status.state = state;
        status.session = session;
        status.startedMs = startMs;
        status.capacity = MBR_HISTORY_CAPACITY;
        status.preMs = MBR_HISTORY_PRE_MS;
        status.postMs = MBR_HISTORY_POST_MS;
        if (disconnected) status.reason = MBR_HISTORY_MANUAL;
        critical_section_exit(&guard);
    } else if (command == CMD_MBR_HISTORY_FREEZE) {
        freeze(MBR_HISTORY_MANUAL, trace.publishedMs);
        critical_section_enter_blocking(&guard);
        status.pending = 0;
        critical_section_exit(&guard);
    }
    if (state != MBR_HISTORY_ARMED && state != MBR_HISTORY_POST) return;
    if (!(trace.flags & MBR_TRACE_PROFILE) || trace.chipCount != chipCount) {
        freeze(MBR_HISTORY_PROFILE_CHANGED, trace.publishedMs);
        return;
    }
    if (uint32_t(trace.publishedMs - startMs) >= 10000) {
        freeze(MBR_HISTORY_TIMEOUT, trace.publishedMs);
        return;
    }
    bool overwritten = false;
    if (!queue_try_add(&frames, &trace)) {
        queue_try_remove(&frames, nullptr);
        overwritten = true;
        queue_try_add(&frames, &trace);
    }
    uint32_t slider = 0;
    for (uint8_t i = 0; i < 32; ++i) if (trace.slider[i]) slider |= uint32_t(1) << i;
    // Capture changes even on failed reads; quality stays in the recorded frame.
    bool edge = havePrevious &&
        (slider != previousSlider || std::memcmp(trace.hardware, previousHardware, 6) ||
         std::memcmp(trace.verified, previousVerified, 6));
    if (state == MBR_HISTORY_ARMED && edge && uint32_t(trace.publishedMs - startMs) >= MBR_HISTORY_PRE_MS) {
        triggerMs = trace.publishedMs;
        state = MBR_HISTORY_POST;
        critical_section_enter_blocking(&guard);
        status.triggerFrame = trace.frameId;
        status.triggerMs = triggerMs;
        status.reason = MBR_HISTORY_EDGE;
        status.state = state;
        critical_section_exit(&guard);
    }
    previousSlider = slider;
    std::memcpy(previousHardware, trace.hardware, 6);
    std::memcpy(previousVerified, trace.verified, 6);
    havePrevious = true;
    const uint32_t hookUs = time_us_32() - hookStart;
    critical_section_enter_blocking(&guard);
    ++status.stored;
    status.overwritten += overwritten;
    ++status.hookCount;
    status.hookSumUs += hookUs;
    if (hookUs > status.hookMaxUs) status.hookMaxUs = hookUs;
    status.endedMs = trace.publishedMs;
    critical_section_exit(&guard);
    if (state == MBR_HISTORY_POST && uint32_t(trace.publishedMs - triggerMs) >= MBR_HISTORY_POST_MS) {
        freeze(0, trace.publishedMs);
    }
}
