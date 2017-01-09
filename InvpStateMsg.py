#    Inverted Pendumlum simulator
#    Authors: Rick Candell, Yongkang Liu
#    Organization: US NIST

import json
import time

__TEST__ = False

class InvpStateMsg(object):
    """Inv. Pend. state message"""
    def __init__(self, TheTime=None, BasePos=None, PendAngle=None):
        self.t = TheTime
        self.pos = BasePos
        self.angle = PendAngle

    def tojson(self):
        js = {"time":self.t, "pos":self.pos, "angle":self.angle}
        s = json.dumps(js)
        return s

    def fromjson(self,s):
        jsst = json.loads(s)
        self.t = jsst["time"]
        self.pos = jsst["pos"]
        self.angle = jsst["angle"]

    def __str__(self):
        objstr = super().__str__()
        objstr = objstr + "\ntime:  " + str(self.t)
        objstr = objstr + "\npos:   " + str(self.pos)
        objstr = objstr + "\nangle: " + str(self.angle) + "\n"
        return objstr

if __TEST__:
    t = 100.

    state1 = InvpStateMsg(t,100.,200.)
    print(state1)
    jstr = state1.tojson()
    print(jstr)

    state2 = InvpStateMsg()
    print(state2)
    state2.fromjson(jstr)
    print(state2)





