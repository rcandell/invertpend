import socketserver
import matplotlib.pyplot as plt
import numpy as np
import math
from InvpStateMsg import *

# globals
fig = None
li = None
server = None

def initPlot():
    global fig
    global li
    global server

    # create the plot canvas
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal',adjustable='box')
    ax.set_xlim([-10.0,10.0])
    ax.set_ylim([-5.0,5.0])
    x = 0
    y = 1
    li, = ax.plot([0.0,0.0], [0.0,1.0], 'go-')
    fig.canvas.draw()
    fig.canvas.flush_events()
    plt.show(block=False)


class PlotCommandUDPHandler(socketserver.BaseRequestHandler):
    """
    This class works similar to the TCP handler class, except that
    self.request consists of a pair of data and client socket, and since
    there is no connection the client address must be given explicitly
    when sending data back via sendto().
    """

    def handle(self):
        global fig
        global li
        global server

        print("Message received")

        data = self.request[0].strip()
        socket = self.request[1]
        print("{} wrote:".format(self.client_address[0]))
        print(data)
        msg = InvpStateMsg()
        msg.fromjson(data.decode('utf-8'))
        print(msg)

        # plot the message
        x1 = msg.pos
        y1 = 0.0
        x2 = x1 + math.sin(msg.angle)
        angle = msg.angle
        y2 = math.cos(angle)
        x = np.array([x1,x2])
        y = np.array([y1,y2])
        li.set_xdata(x)
        li.set_ydata(y)

        # redraw the canvas
        fig.canvas.draw()
        fig.canvas.flush_events()
        plt.show(block=False)


def serverMain():
    # launch the socket connection handler
    host = '127.0.0.1'
    port = 24777
    buf = 1024
    addr = (host, port)
    server = socketserver.UDPServer(addr, PlotCommandUDPHandler)
    server.serve_forever()


if __name__ == "__main__":
    initPlot()
    serverMain()
