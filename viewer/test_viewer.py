import sys
import numpy as np
from PyQt5 import QtWidgets, QtCore
import pyqtgraph.opengl as gl

app = QtWidgets.QApplication(sys.argv)

view = gl.GLViewWidget()
view.setWindowTitle("3D test - synthetic points")
view.resize(900, 650)
view.setCameraPosition(distance=5, elevation=25, azimuth=45)

grid = gl.GLGridItem()
view.addItem(grid)

x, y = np.meshgrid(
    np.linspace(-1, 1, 80),
    np.linspace(-1, 1, 40)
)
points = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))

cloud = gl.GLScatterPlotItem(
    pos=points, size=3,
    color=(0.2, 0.8, 1.0, 1.0),
    pxMode=True
)
cloud.setGLOptions("opaque")
view.addItem(cloud)

phase = 0.0

def update():
    global phase
    phase += 0.1
    points[:, 2] = 0.3 * np.sin(points[:, 0] * 4 + phase)
    cloud.setData(pos=points)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(100)

view.show()
sys.exit(app.exec_())
