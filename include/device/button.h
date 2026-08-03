/* This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *
 * Copyright (c) 2026 Catium2006
 */

#ifndef __BUTTON_H__
#define __BUTTON_H__

enum Button{
    BUTTON_UP,
    BUTTON_DOWN,
    BUTTON_PUSH
};

void initButtons();

bool getButtonState(Button button);

#endif