#!/usr/bin/env python3

import selfdrive.crash as crash

try:
    1 / 0
except ZeroDivisionError:
    crash.capture_exception()
