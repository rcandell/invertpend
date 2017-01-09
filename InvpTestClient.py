import socket
import sys
from InvpStateMsg import *
import math
import numpy as np
import random
import time

class InvpStateBroadcaster(object):

    bcast_addr = ('127.0.0.1',24777)

    def __init__(self):
        self.s=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def sendState(self, state_str):
        self.s.sendto(state_str.encode('utf-8'),self.bcast_addr)

if __name__ == "__main__":
    bcast = InvpStateBroadcaster()
    t = 0
    dt = 0.2
    dx = 1.2
    da = 0.05*math.pi
    pos = 0.0
    angle = math.pi/4.0
    while True:
        dxi = random.random()-0.5
        pos = pos + dx*dxi
        dai = math.pi*(random.random()-0.5)
        angle = angle + da*dai
        msg = InvpStateMsg(t,pos,angle)
        print(msg)
        print(msg.tojson())
        bcast.sendState(msg.tojson())
        time.sleep(dt)
        t += dt

