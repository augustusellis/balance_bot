#!/usr/bin/env python3

import pigpio
import time

pi = pigpio.pi()

data_gpio = 14
clock_gpio = 15
preset_gpio = 8
clear_gpio = 25


Q_gpio = 23

Qn_gpio = 24

pi.set_mode(Q_gpio, pigpio.INPUT)
pi.set_mode(Qn_gpio, pigpio.INPUT)
pi.set_mode(data_gpio, pigpio.OUTPUT)
pi.set_mode(clock_gpio, pigpio.OUTPUT)
pi.set_mode(clear_gpio, pigpio.OUTPUT)
pi.set_mode(preset_gpio, pigpio.OUTPUT)


pi.set_pull_up_down(Q_gpio, pigpio.PUD_UP)
pi.set_pull_up_down(Qn_gpio, pigpio.PUD_UP)
#pi.set_pull_up_down(data_gpio, pigpio.PUD_UP)
#pi.set_pull_up_down(clock_gpio, pigpio.PUD_UP)
#pi.set_pull_up_down(clear_gpio, pigpio.PUD_UP)
#pi.set_pull_up_down(preset_gpio, pigpio.PUD_UP)

pi.write(data_gpio, 0)
pi.write(clock_gpio, 0)
time.sleep(0.01)

pi.write(preset_gpio, 1)
pi.write(clear_gpio, 1)
time.sleep(0.01)

pi.write(preset_gpio, 0)
pi.write(clear_gpio, 1)
time.sleep(0.01)

pi.write(preset_gpio, 1)
pi.write(clear_gpio, 1)
time.sleep(0.01)


while True:
    #pi.write(clock_gpio, 1)
    D = int(input('D: '))
    PRE = int(input('PRE: '))
    CLR = int(input('CLR: '))
    CLK = int(input('CLK: '))

    pi.write(data_gpio, D)
    pi.write(preset_gpio, PRE)
    pi.write(clear_gpio, CLR)
    pi.write(clock_gpio, CLK)
    time.sleep(0.01)

    #pi.write(clock_gpio, 1)
    #time.sleep(0.01)
    #pi.write(clock_gpio, 0)
    #time.sleep(0.01)

    Q = bool(pi.read(Q_gpio))
    Qn = bool(pi.read(Qn_gpio))

    if Q == 1:
        print("\033[92m Q: {0:<d} Qn: {1:<d} D: {2:<d},  CLK: {3:<d} \033[0m".format(Q, Qn, D, CLK))

    else:
        print("\033[91m Q: {0:<d} Qn: {1:<d} D: {2:<d} \033[0m".format(Q, Qn, D))
