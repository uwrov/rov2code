import numpy as np
import pyqtgraph as pg
from PyQt6.QtWidgets import QMainWindow
from PyQt6.QtCore import QTimer

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
        self.process_data = np.zeros(self.data_points)
        self.setpoint_data = np.zeros(self.data_points)
        
        self.process_curve = self.plot_widget.plot(
            self.time_data, 
            self.process_data, 
            pen=pg.mkPen('b', width=2), 
            name='Process Value'
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

    def update_plot(self, setpoint, current_position):
        self.time_data[:-1] = self.time_data[1:]
        self.process_data[:-1] = self.process_data[1:]
        self.setpoint_data[:-1] = self.setpoint_data[1:]
        
        self.current_time += 0.02
        self.time_data[-1] = self.current_time
        self.process_data[-1] = current_position
        self.setpoint_data[-1] = setpoint
        
        self.process_curve.setData(self.time_data, self.process_data)
        self.setpoint_curve.setData(self.time_data, self.setpoint_data)

if __name__ == "__main__":
