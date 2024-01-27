#!/usr/bin/env python3

import sys, time

import navio2.rcinput
import navio2.util

navio2.util.check_apm()

rcin = navio2.rcinput.RCInput()

while (True):
    period = rcin.read(9)
    print(period)
    time.sleep(1)