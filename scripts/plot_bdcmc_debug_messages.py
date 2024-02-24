import struct

import can

from math import *

import can
import numpy as np
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore

if __name__ == "__main__":
    app = pg.mkQApp()

    win = pg.GraphicsLayoutWidget(show=True, title="Basic plotting examples")

    win.setWindowTitle("Live Graph")

    pg.setConfigOptions(antialias=True)

    plot = win.addPlot()
    curve1 = plot.plot(pen="b")
    # curve2 = plot.plot(pen='g')
    # curve3 = plot.plot(pen='r')

    plot.enableAutoRange("xy", True)
    plot.setYRange(0, 1 << 14)

    xs = np.zeros(40)
    ys = np.zeros((40, 4))

    with can.interface.Bus(channel="vcan0", interface="socketcan", fd=True) as bus:

        def update():
            message = bus.recv()
            # unpack message id as signed short
            message_id = message.data[16]

            if message_id == 1:
                global xs
                global ys

                xs = np.roll(xs, -1)
                ys = np.roll(ys, -1, axis=0)

                xs[-1] = message.timestamp
                ys[-1] = np.array(struct.unpack("ffff", message.data[0:16]))

                curve1.setData(xs, ys[:, 0])
                # curve2.setData(xs, ys[:, 1])
                # curve3.setData(xs, ys[:, 2])

        timer = QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(0)

        pg.exec()
