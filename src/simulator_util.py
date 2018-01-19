import matplotlib.pyplot as plt
from matplotlib.patches import Circle, Ellipse

import Util


class DraggablePoint(object):

    # http://stackoverflow.com/questions/21654008/matplotlib-drag-overlapping-points-interactively

    lock = None

    def __init__(self, parent, x=0.1, y=0.1, size=0.2, color=None):

        if color is None:
            color = [20, 20, 20]
        self.parent = parent
        self.point = Circle((x, y), size, color=Util.rgb_array_to_hex_str(color), zorder=99)
        self.x = x
        self.y = y
        self.parent.axes[0].add_patch(self.point)
        self.press = None
        self.background = None
        self.connect()

    def connect(self):
        self.cidpress = self.point.figure.canvas.mpl_connect('button_press_event', self.on_press)
        self.cidrelease = self.point.figure.canvas.mpl_connect('button_release_event', self.on_release)
        self.cidmotion = self.point.figure.canvas.mpl_connect('motion_notify_event', self.on_motion)

    def on_press(self, event):
        if event.inaxes != self.point.axes: return
        if DraggablePoint.lock is not None: return
        contains, attrd = self.point.contains(event)
        if not contains: return
        self.press = (self.point.center), event.xdata, event.ydata
        DraggablePoint.lock = self
        canvas = self.point.figure.canvas
        axes = self.point.axes
        self.point.set_animated(True)
        canvas.draw()
        self.background = canvas.copy_from_bbox(self.point.axes.bbox)
        axes.draw_artist(self.point)
        canvas.blit(axes.bbox)

    def on_motion(self, event):
        if DraggablePoint.lock is not self:
            return
        if event.inaxes != self.point.axes: return
        self.point.center, xpress, ypress = self.press
        dx = event.xdata - xpress
        dy = event.ydata - ypress
        x = self.point.center[0]+dx
        y = self.point.center[1]+dy
        self.set_point_pose(x, y)

    def set_point_pose(self, x, y):
        self.point.center = (x, y)
        self.refresh()

        self.pose.position.x = self.x
        self.pose.position.y = self.y

    def on_release(self, event):
        if DraggablePoint.lock is not self:
            return

        self.press = None
        DraggablePoint.lock = None
        self.point.set_animated(False)

        self.background = None
        self.point.figure.canvas.draw()

        self.x = self.point.center[0]
        self.y = self.point.center[1]

    def refresh(self):
        self.point.set_animated(True)
        canvas = self.point.figure.canvas
        canvas.draw()
        self.background = canvas.copy_from_bbox(self.point.axes.bbox)
        axes = self.point.axes
        canvas.restore_region(self.background)
        axes.draw_artist(self.point)
        canvas.blit(axes.bbox)
        self.x = self.point.center[0]
        self.y = self.point.center[1]
        self.point.set_animated(False)
        self.background = None

    def disconnect(self):
        self.point.figure.canvas.mpl_disconnect(self.cidpress)
        self.point.figure.canvas.mpl_disconnect(self.cidrelease)
        self.point.figure.canvas.mpl_disconnect(self.cidmotion)

    def remove(self):
        self.point.remove()
        self.disconnect()
