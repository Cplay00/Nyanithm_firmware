/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. https://mozilla.org/MPL/2.0/ */
#ifndef __MBR_HISTORY_H__
#define __MBR_HISTORY_H__
#include <nyanithm_shared.h>

void initMbrHistory();
void recordMbrHistory(const MbrTouchTrace& trace);
bool requestMbrHistory(uint8_t command);
void disconnectMbrHistory();
MbrHistoryStatus getMbrHistoryStatus();
bool readMbrHistory(MbrHistoryFrame& frame);
#endif
