#!/usr/bin/python
#   @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University
import interaction_primitive_widget as ipw
import PyQt5.QtWidgets
import rospy
import sys

class DashboardMainWindow(PyQt5.QtWidgets.QMainWindow):

    def __init__(self, parent = None):
        super(DashboardMainWindow, self).__init__(parent)

        self.alive = True

        self.title = 'Interaction Primitives Dashboard'
        self.left = 10
        self.top = 10
        self.width = 1200
        self.height = 600

        rospy.init_node("dashboard_widget", anonymous = False)

        self.init_ui()

    def closeEvent(self, event):
        self.alive = False

    def init_ui(self):
        self.setWindowTitle(self.title)
        self.setGeometry(self.left, self.top, self.width, self.height)

        tab_widget = PyQt5.QtWidgets.QTabWidget()
        tab_widget.addTab(ipw.InteractionPrimitiveWidget(self), "Interaction Primitive")

        self.setCentralWidget(tab_widget)

if __name__ == '__main__':
    app = PyQt5.QtWidgets.QApplication(sys.argv)
    dashboard = DashboardMainWindow()
    dashboard.show()
    sys.exit(app.exec_())
