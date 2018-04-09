#MicroPython v1.9.2 on 2017-08-23; PYBv1.0 with STM32F405RG

import time
import machine
from machine import Pin, I2C, UART
##import ssd1306
from pyb import LED
from pyb import Pin
from pyb import Timer
from pyb import ADC
import math as m
import ujson

# print ("ready for code")
#  pyb.LED(1)


def collect():
    """
    collect() takes a 20 sample average over 0.5 s from each ADC pin
    :return: p_all: a vector of the averaged values
    """

    p0 = pyb.ADC('X3')
    p1 = pyb.ADC('X4')
    p2 = pyb.ADC('X5')
    p3 = pyb.ADC('X6')
    p4 = pyb.ADC('X7')
    p5 = pyb.ADC('X8')

    nav = 20
    npots = 6
    p_all = [0] * npots

    for i in range(nav):   # sum loop
        p_all[0] += p0.read()
        p_all[1] += p1.read()
        p_all[2] += p2.read()
        p_all[3] += p3.read()
        p_all[4] += p4.read()
        p_all[5] += p5.read()
        pyb.delay(25)

    for i2 in range(npots):   # divide loop
        p_all[i2] = p_all[i2] / nav

    return p_all


def calibrate():
    """
    calibrate() is triggered by the USR switch. Once triggered, the user should bring each joint to its
    lower bound and press the USR switch again. This saves these values in a vector called p_lo. The user
    should then bring each joint to the other extreme and press the USR switch once more. This create the
    p_hi vector and the code returns to normal operation
    :return:
    """

    f = open('bounds_file.txt', 'w')
    trig = False
    pyb.delay(1000)
    while not trig:  # waiting for USR input
        if sw():
            pyb.delay(100)
            if sw():
                trig = True

    p_lo = collect()
    print('lower bound collected!')
    trig = False
    pyb.delay(500)
    while not trig:  # waiting for USR input
        if sw():
            pyb.delay(100)
            if sw():
                trig = True

    p_hi = collect()
    print('upper bound collected!')

    p_list = p_lo + p_hi
    a = ujson.dumps(p_list)
    f.write(a)
    print('wrote')
    pyb.delay(3000)
    f.close()
    print('closed')
    pyb.delay(3000)

    return p_lo, p_hi


def convert(p_val, p_list, change_bool):
    """

    :param p_val: p_list; change_bool
    :param change_bool:
    :return:
    """
    if change_bool:
        f = open('bounds_file.txt', 'r')
        a = f.read()
        if len(a) == 0:
            f.close()
            f = open('bounds_file.txt','w')
            substitute = [0]*6 + [4056]*6
            usub = ujson.dumps(substitute)
            f.write(usub)
            pyb.delay(3000)
            f.close()
            print('bounds reset, need to recalibrate')
            f = open('bounds_file.txt', 'r')
            a = f.read()
        print(a)
        p_list = ujson.loads(a)
        p_lo = p_list[0:6]
        p_hi = p_list[6:13]
        change_bool = False
        f.close()
        # pyb.delay(3000)

    else:
        p_lo = p_list[0:6]
        p_hi = p_list[6:13]

    n = len(p_val)
    deci = [0] * n
    deg = [0] * n
    rad = [0] * n

    for i in range(n):
        deci[i] = (p_val[i] - p_lo[i])/(p_hi[i] - p_lo[i])
        rad[i] = deci[i] * m.pi - m.pi/2
        deg[i] = round(deci[i] * 180 - 90)

    return deg, rad, p_list, change_bool


def calculate(rad):

    l = [1, 1, 1, 1, 1, 1, 1] # length of each segment
    a = [0] + rad   # angle vector formatted for this function
    ortho = m.pi/2
    r = [0, 0, ortho, ortho, ortho, 0, ortho]   # rotational angle vector, r[0,1] must be 0

    n = len(a)
    x = 0
    y = 0
    z = l[0]
    crt = [0, 0, 1]
    # print('dist to node 0 :', l[0])
    # vecmap = [[0] * 3] * n
    # vecmap[0][2] = 1

    for it in range(1, n):
        # this is where the sph->crt transform happens for each arm wrt its base arm
        # print('it:',it)

        crt[0] = m.sin(a[it]) * m.cos(r[it])
        crt[1] = m.sin(a[it]) * m.sin(r[it])
        crt[2] = m.cos(a[it])

        if it > 1:
            for it2 in range(it - 1, 0, -1):
                # this is where the c.sys(n)->c.sys(0) transform happens for each unit vec
                # print('it2:',it2)
                T = [[m.cos(r[it2]) * m.cos(a[it2]), -m.sin(r[it2]), m.cos(r[it2]) * m.sin(a[it2])], [
                    m.sin(r[it2]) * m.cos(a[it2]), m.cos(r[it2]), m.sin(r[it2]) * m.sin(a[it2])], [
                         -m.sin(a[it2]), 0, m.cos(a[it2])]]

                crt[0] = crt[0] * T[0][0] + crt[1] * T[0][1] + crt[2] * T[0][2]
                crt[1] = crt[0] * T[1][0] + crt[1] * T[1][1] + crt[2] * T[1][2]
                crt[2] = crt[0] * T[2][0] + crt[1] * T[2][1] + crt[2] * T[2][2]

        x += crt[0] * l[it]
        y += crt[1] * l[it]
        z += crt[2] * l[it]

        # vecmap[it] = crt  # logs the unit vecs for each arm wrt c.sys(0)
        # node_dist = round(((x ** 2 + y ** 2 + z ** 2) ** 0.5), 4)
        # print('dist to node', it + 1, ':', node_dist)

    dist = (x ** 2 + y ** 2 + z ** 2) ** 0.5
    pdist = round(dist, 2)
    # print('\narm unit vectors: [X Y Z]\n', vecmap)
    # print('\ndistance:', pdist, 'units')

    return pdist


########### main code goes below ############

sw = pyb.Switch()
change = True
p_bounds = 0

for k in range(180):

    if sw():
        pyb.delay(100)
        if sw():
            print('USR triggered!')
            calibrate()
            change = True
    pot_vals = collect()
    degrees, radians, p_bounds, change = convert(pot_vals, p_bounds, change)
    distance = calculate(radians)
    print('distance: ', distance)


