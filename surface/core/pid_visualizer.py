import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import QTimer
import pickle

class PIDVisualizer(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('PID Controller Visualization')
        self.resize(800, 400)
        
        self.plot_widget = pg.PlotWidget()
        self.setCentralWidget(self.plot_widget)
        
        self.plot_widget.setLabel('left', 'Value')
        self.plot_widget.setLabel('bottom', 'Time')
        self.plot_widget.showGrid(x=True, y=True)
        self.plot_widget.setYRange(0, 100)
        
        self.data_points = 500
        self.time_data = np.zeros(self.data_points)
        self.yaw_data = np.zeros(self.data_points)
        self.pitch_data = np.zeros(self.data_points)
        self.roll_data = np.zeros(self.data_points)
        self.setpoint_data = np.zeros(self.data_points)
        
        self.plot_widget.addLegend()
        self.yaw_curve = self.plot_widget.plot(
            self.time_data, 
            self.yaw_data, 
            pen=pg.mkPen('b', width=2), 
            name='Yaw Value'
        )
        self.pitch_curve = self.plot_widget.plot(
            self.time_data, 
            self.pitch_data, 
            pen=pg.mkPen('g', width=2), 
            name='Pitch Value'
        )
        self.roll_curve = self.plot_widget.plot(
            self.time_data, 
            self.roll_data, 
            pen=pg.mkPen('y', width=2), 
            name='Roll Value'
        )
        self.setpoint_curve = self.plot_widget.plot(
            self.time_data, 
            self.setpoint_data, 
            pen=pg.mkPen('r', width=2), 
            name='Setpoint'
        )

        self.timer = QTimer()
        self.timer.timeout.connect(self.update_plot)
        self.timer.start(20)

        self.current_time = 0

    def update_plot(self):
        self.time_data[:-1] = self.time_data[1:]
        self.yaw_data[:-1] = self.yaw_data[1:]
        self.pitch_data[:-1] = self.pitch_data[1:]
        self.roll_data[:-1] = self.roll_data[1:]
        self.setpoint_data[:-1] = self.setpoint_data[1:]
        
        self.current_time += 0.02
        self.time_data[-1] = self.current_time

        try:
            pitch, roll, yaw = pickle.load(open("rot.pkl", 'rb'))
            setpoint = pickle.load(open("setpoint.pkl", 'rb'))
        except:
            current_position = 0
            setpoint = 0

        self.yaw_data[-1] = yaw
        self.pitch_data[-1] = pitch
        self.roll_data[-1] = roll
        self.setpoint_data[-1] = setpoint
        
        self.yaw_curve.setData(self.time_data, self.yaw_data)
        self.pitch_curve.setData(self.time_data, self.pitch_data)
        self.roll_curve.setData(self.time_data, self.roll_data)
        self.setpoint_curve.setData(self.time_data, self.setpoint_data)

if __name__ == "__main__":
    import sys
    from PyQt6.QtWidgets import QApplication

    app = QApplication(sys.argv)
    viz = PIDVisualizer()
    viz.show()
    app.exec()