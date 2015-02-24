#!/usr/bin/env python
#
# Copright (c) 2015 Michele Segata <segata@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import sys
from PyQt4 import QtCore, QtGui
import socket
import threading

run = True

class SockServer():

    def __init__(self, *args):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_address = ('0.0.0.0', 33333)
        self.sock.bind(server_address)

        self.sock.listen(1)

    def setObjects(self, rpmMeter, speedMeter, gear, accMeter):
        self.rpmMeter = rpmMeter
        self.speedMeter = speedMeter
        self.gear = gear
        self.accMeter = accMeter

    def startServing(self):
        global run
        while run:
            try:
                client, client_address = self.sock.accept()
                runClient = True
                while runClient:
                    try:
                        data = client.recv(2048).rstrip()
                        (rpm, speed, gear, acc) = data.split()
                        rpm = float(rpm)
                        speed = float(speed)
                        gear = int(gear)
                        acc = float(acc)
                        self.rpmMeter.setValue(rpm)
                        self.speedMeter.setValue(speed)
                        self.gear.display(gear)
                        self.accMeter.setValue(acc)
                    except:
                        client.close()
                        runClient = False
            except:
                run = False

class Tachometer():

    def __init__(self, *args):
        self.x = 0
        self.y = 0
        self.w = 100
        self.h = 100
        self.minv = 0.0
        self.maxv = 9000.0
        self.mind = -45.0
        self.maxd = 225.0
        self.v = self.minv
        self.dv = self.minv
        self.upperLimited = False
        self.alpha = 0.01 / (0.1 + 0.01)
        self.tacho = QtGui.QPixmap("tachometer.png")
        self.needle = QtGui.QPixmap("needle.png")
        self.circle = QtGui.QPixmap("circle.png")
    def setPixmaps(self, tacho, needle, circle):
        self.tacho = QtGui.QPixmap(tacho)
        self.needle = QtGui.QPixmap(needle)
        self.circle = QtGui.QPixmap(circle)
    def setPosition(self, x, y, w, h):
        self.x = x
        self.y = y
        self.w = w
        self.h = h
    def setMinMaxValues(self, minv, maxv):
        self.minv = minv
        self.maxv = maxv
    def setMinMaxDegrees(self, mind, maxd):
        self.mind = mind
        self.maxd = maxd
    def setUpperLimited(self, upperLimited):
        self.upperLimited = upperLimited
    def setValue(self, v):
        if (v < self.minv): v = self.minv
        if (v > self.maxv and self.upperLimited): v = self.maxv
        self.v = v
    def valueToDegrees(self, v):
        degrees = (v - self.minv) / (self.maxv - self.minv) * (self.maxd - self.mind) + self.mind
        return degrees
    def paint(self, painter):
        #draw the tachometer
        painter.drawPixmap(self.x, self.y, self.w, self.h, self.tacho)

        #draw the needle, rotated depending on the value
        painter.save()
        painter.translate(self.x + self.w/2, self.y + self.h/2)
        self.dv = self.alpha * self.v + (1-self.alpha)*self.dv
        degrees = self.valueToDegrees(self.dv)
        painter.rotate(degrees)
        painter.translate(-self.x-self.w/2, -self.y-self.h/2)
        painter.drawPixmap(self.x, self.y, self.w, self.h, self.needle)
        painter.restore()

        #draw the top circle
        x = self.w/2 - self.circle.width()/2
        y = self.h/2 - self.circle.height()/2
        w = self.w
        h = self.h
        painter.drawPixmap(self.x, self.y, self.w, self.h, self.circle)

def server(rpmMeter, speedMeter, gear, accMeter):
    ss = SockServer()
    ss.setObjects(rpmMeter, speedMeter, gear, accMeter)
    ss.startServing()

class Dashboard(QtGui.QWidget):

    def __init__(self, parent=None):

        super(Dashboard, self).__init__(parent)

        timer = QtCore.QTimer(self)
        timer.timeout.connect(self.update)
        timer.start(10)

        self.setWindowTitle(QtCore.QObject.tr(self, "Plexe Dashboard"))
        self.setStyleSheet("background-color:#222;");
        self.resize(700, 300)

        self.rpm = 0
        self.speed = 0
        self.increasing = +1
        self.sincreasing = +1

        self.rpmMeter = Tachometer()
        self.rpmMeter.setPixmaps("tachometer.png", "needle.png", "circle.png")
        self.rpmMeter.setPosition(0, 0, 300, 300)
        self.rpmMeter.setMinMaxValues(0.0, 9000.0)
        self.rpmMeter.setMinMaxDegrees(-45.0, 225.0)
        self.rpmMeter.setValue(0.0)

        self.speedMeter = Tachometer()
        self.speedMeter.setPixmaps("speedometer.png", "needle.png", "circle.png")
        self.speedMeter.setPosition(400, 0, 300, 300)
        self.speedMeter.setMinMaxValues(0.0, 400.0)
        self.speedMeter.setMinMaxDegrees(-45.0, 225.0)
        self.speedMeter.setValue(0.0)

        self.accMeter = Tachometer()
        self.accMeter.setPixmaps("accelerometer.png", "acc-needle.png", "acc-circle.png")
        self.accMeter.setPosition(275, 217, 150, 150)
        self.accMeter.setMinMaxValues(-12.0, 12.0)
        self.accMeter.setMinMaxDegrees(0.0, 180.0)
        self.accMeter.setValue(0.0)

        self.gear = QtGui.QLCDNumber(1, self)
        self.gear.setGeometry(325, 25, 50, 100)
        palette = self.gear.palette()
        palette.setColor(palette.WindowText, QtGui.QColor(180, 0, 0))
        self.gear.setPalette(palette)
        self.gear.setSegmentStyle(QtGui.QLCDNumber.Flat)

        self.child = threading.Thread(target=server, args=(self.rpmMeter,self.speedMeter,self.gear,self.accMeter,))
        self.child.setDaemon(True)
        self.child.start()

    def paintEvent(self, event):

        painter = QtGui.QPainter()
        painter.begin(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)

        self.rpmMeter.paint(painter)
        self.speedMeter.paint(painter)
        self.accMeter.paint(painter)

        painter.end()

if __name__ == "__main__":

    app = QtGui.QApplication(sys.argv)
    dashboard = Dashboard()
    dashboard.show()
    sys.exit(app.exec_())

