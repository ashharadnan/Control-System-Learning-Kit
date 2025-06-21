import sys
import random

from pathlib import Path
PROJECT_DIR = Path(__file__).parent


from PySide6.QtUiTools import QUiLoader
from PySide6.QtWidgets import QApplication, QWidget, QDialog, QTableWidgetItem, QVBoxLayout
from PySide6.QtGui import QIcon
from PySide6.QtCore import QFile, QIODevice

import easygui

from serial.tools.list_ports import comports
from serial import Serial
import _thread
import struct

import numpy as np
import pandas as pd
import time

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure


class PlotCanvas(FigureCanvas):
    def __init__(self, parent=None, dpi=100):
        super(PlotCanvas, self).__init__(Figure())

        self.setParent(parent)

        # Create the figure and figure canvas
        fig = Figure(dpi=dpi)
        self.figure = fig
        self.canvas = FigureCanvas(self.figure)
        self.ax1 = fig.add_subplot()

        # Graph title text
        self.ax1.set_title('Time Response')

        # Axes labels text
        self.ax1.set_xlabel('time [s]')

        self.ax1.set_ylabel('Height [cm]')
        self.ax1.set_ylim([-2, 20])

        self.ax2 = self.ax1.twinx()

        self.ax2.set_ylabel('PWM [%]', color='green')
        self.ax2.set_ylim([0, 100])
        self.ax2.tick_params(axis='y', labelcolor='green')

        self.ax1.grid(True)

    def clean(self):
        self.figure.clf()
        self.canvas = FigureCanvas(self.figure)
        self.ax1 = self.figure.add_subplot()

        # Graph title text
        self.ax1.set_title('Time Response')

        # Axes labels text
        self.ax1.set_xlabel('time [s]')

        self.ax1.set_ylabel('Height [cm]')
        self.ax1.set_ylim([-2, 20])

        self.ax2 = self.ax1.twinx()

        self.ax2.set_ylabel('PWM [%]', color='green')
        self.ax2.set_ylim([0, 100])
        self.ax2.tick_params(axis='y', labelcolor='green')

        self.ax1.grid(True)

def populate_coms():
    global coms
    coms = comports()
    myApp.comboBoxCOM.clear()
    for comp in coms:
        myApp.comboBoxCOM.addItem(comp.description)


def update_combobox(idx):
    myApp.comboBoxCOM.setCurrentIndex(idx)


def connect_serial():
    global coms
    global ser
    global pollFlag
    pollFlag = False
    ser.close()
    try:
        ser = Serial(coms[myApp.comboBoxCOM.currentIndex()].device, 115200)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        debug_on()
        set_kp()
        set_ki()
        set_kd()
        set_pwm()
        tare()
        set_sp()
        myApp.comboBoxCOM.setEnabled(False)
        myApp.BtnRefresh.setEnabled(False)
        myApp.groupBox_8.setEnabled(True)
        myApp.BtnSave.setEnabled(True)
        pollFlag = True
        _thread.start_new_thread(data_aqcuire, ())
    except Exception as e:
        print(e)
        ui_file_name = "dialog.ui"
        ui_file = QFile(ui_file_name)
        loader = QUiLoader()
        dlg = loader.load(ui_file)
        ui_file.close()
        dlg.setWindowIcon(QIcon("assets/icon.jpg"))
        dlg.exec()
        myApp.comboBoxCOM.setEnabled(True)
        myApp.BtnRefresh.setEnabled(True)
        myApp.groupBox_8.setEnabled(False)
        populate_coms()


def debug_on():
    global ser
    ser.write(b"DEBUG 1\r\n")
    myApp.textDebugState.setText("Debug Mode: on")
    myApp.BtnDebugOn.setEnabled(False)
    myApp.BtnDebugOff.setEnabled(True)
    myApp.groupBox_2.setEnabled(True)
    myApp.groupBox_3.setEnabled(True)


def debug_off():
    global ser
    ser.write(b"DEBUG 0\r\n")
    myApp.textDebugState.setText("Debug Mode: off")
    myApp.BtnDebugOn.setEnabled(True)
    myApp.BtnDebugOff.setEnabled(False)
    myApp.groupBox_2.setEnabled(False)
    myApp.groupBox_3.setEnabled(False)


def set_kp():
    global ser
    val = myApp.boxKp.value()
    ser.reset_input_buffer()
    ser.write(bytes("KP {0}\r\n".format(val), "utf-8"))


def set_ki():
    global ser
    val = myApp.boxKi.value()
    ser.reset_input_buffer()
    ser.write(bytes("KI {0}\r\n".format(val), "utf-8"))


def set_kd():
    global ser
    val = myApp.boxKd.value()
    ser.reset_input_buffer()
    ser.write(bytes("KD {0}\r\n".format(val), "utf-8"))


def tare():
    global ser
    ser.reset_input_buffer()
    ser.write(bytes("TARE\r\n", "utf-8"))


def set_pwm():
    global ser
    val = myApp.boxPWM.value()
    ser.reset_input_buffer()
    ser.write(bytes("PWM% {0}\r\n".format(val), "utf-8"))


def set_sp():
    global ser
    val = myApp.boxSP.value()
    ser.reset_input_buffer()
    ser.write(bytes("SP {0}\r\n".format(val), "utf-8"))


def sp_box2slider(val):
    myApp.SliderSP.setValue(round(val))


def sp_slider2box():
    val = myApp.SliderSP.value()
    if round(myApp.boxSP.value()) != val:
        myApp.boxSP.setValue(val)


def start_acquire():
    global acquireFlag, Tstart, series, dframe
    acquireFlag = True
    Tstart = None
    series = None
    dframe = None
    myApp.tableData.clearContents()
    myApp.tableData.setRowCount(0)
    myApp.BtnStart.setEnabled(False)
    myApp.actionStart.setEnabled(False)
    myApp.BtnStop.setEnabled(True)
    myApp.actionStop.setEnabled(True)


def stop_acquire():
    global acquireFlag, series, dframe
    acquireFlag = False
    dframe = pd.DataFrame(series, columns=["time", "height", "error", "setpoint", "pwm"])
    myApp.BtnStart.setEnabled(True)
    myApp.actionStart.setEnabled(True)
    myApp.BtnStop.setEnabled(False)
    myApp.actionStop.setEnabled(False)


def data_aqcuire():
    global ser
    global pollFlag, acquireFlag
    global current, Tstart, series
    global canvas

    while (pollFlag):
        msg = []
        ser.reset_input_buffer()
        while (len(msg) != 24):
            msg = ser.readline()
        st = struct.unpack('<xxxcxxxffffc', msg)

        current["height"] = st[1]
        current["error"] = st[2]
        current["setpoint"] = st[3]
        current["pwm"] = st[4]

        myApp.lcdHeight.display(current["height"])
        myApp.lcdError.display(current["error"])
        myApp.lcdPWM.display(current["pwm"])

        if (acquireFlag):
            if Tstart is None:
                Tstart = time.time()
            t = round(time.time() - Tstart, 1)
            row = myApp.tableData.rowCount()
            myApp.tableData.setRowCount(row + 1)
            myApp.tableData.setCurrentCell(row, 2)
            myApp.tableData.setItem(row, 0, QTableWidgetItem(str(t)))
            myApp.tableData.setItem(row, 1, QTableWidgetItem(str(round(current["height"], 2))))
            myApp.tableData.setItem(row, 2, QTableWidgetItem(str(round(current["error"], 2))))
            myApp.tableData.setItem(row, 3, QTableWidgetItem(str(round(current["setpoint"], 2))))
            myApp.tableData.setItem(row, 4, QTableWidgetItem(str(round(current["pwm"], 2))))

            if series is None:
                series = np.array([t, current["height"], current["error"], current["setpoint"], current["pwm"]])
                canvas.clean()
                canvas.Hplot, = canvas.ax1.plot(series[0], series[1])
                canvas.SPplot, = canvas.ax1.plot(series[0], series[3])
                canvas.PWMplot, = canvas.ax2.plot(series[0], series[4], color='green')
                canvas.ax1.legend(["Height", "SetPoint"], loc='upper left')
                canvas.ax2.legend(["PWM"], loc='upper right')
                canvas.draw()
                canvas.flush_events()
            else:
                new = np.array([t, current["height"], current["error"], current["setpoint"], current["pwm"]])
                series = np.vstack((series, new))

                canvas.Hplot.set_data(series[:, 0], series[:, 1])
                canvas.SPplot.set_data(series[:, 0], series[:, 3])
                canvas.PWMplot.set_data(series[:, 0], series[:, 4])
                canvas.figure.gca().relim()
                canvas.figure.gca().autoscale_view()
                canvas.draw()
                canvas.flush_events()



def save_csv():
    global dframe
    if dframe is not None:
        path = easygui.filesavebox("Select a location to save csv file", "Save as CSV", "data.csv", ["*.csv"])
        dframe.to_csv(path, index=False)
    else:
        ui_file_name = "dialog_csv.ui"
        ui_file = QFile(ui_file_name)
        loader = QUiLoader()
        dlg = loader.load(ui_file)
        ui_file.close()
        dlg.setWindowIcon(QIcon("assets/icon.jpg"))
        dlg.exec()


def quit():
    sys.exit(app.exec())


if __name__ == "__main__":
    app = QApplication(sys.argv)
    app.setApplicationDisplayName('Control System Learning Kit')

    ui_file_name = "layout.ui"
    ui_file = QFile(PROJECT_DIR/ui_file_name)
    if not ui_file.open(QIODevice.ReadOnly):
        print(f"Cannot open {ui_file_name}: {ui_file.errorString()}")
        sys.exit(-1)
    loader = QUiLoader()
    myApp = loader.load(ui_file)
    ui_file.close()

    coms = []
    ser = Serial()

    populate_coms()

    current = {'height': 0.0,
               'error': 0.0,
               'setpoint': 0.0,
               'pwm': 0.0}

    pollFlag = False
    acquireFlag = False

    Tstart = None

    dframe = None
    series = None

    myApp.groupBox_8.setEnabled(False)

    canvas = PlotCanvas(myApp.framePlot)
    layout = QVBoxLayout()
    layout.addWidget(canvas)
    myApp.framePlot.setLayout(layout)

    myApp.comboBoxCOM.activated.connect(update_combobox)
    myApp.BtnRefresh.clicked.connect(populate_coms)
    myApp.BtnConnect.clicked.connect(connect_serial)
    myApp.BtnDebugOn.clicked.connect(debug_on)
    myApp.BtnDebugOff.clicked.connect(debug_off)
    myApp.BtnKp.clicked.connect(set_kp)
    myApp.BtnKi.clicked.connect(set_ki)
    myApp.BtnKd.clicked.connect(set_kd)
    myApp.BtnTare.clicked.connect(tare)
    myApp.BtnPWM.clicked.connect(set_pwm)
    myApp.BtnSP.clicked.connect(set_sp)
    myApp.boxSP.valueChanged.connect(sp_box2slider)
    myApp.SliderSP.valueChanged.connect(sp_slider2box)

    myApp.BtnStart.clicked.connect(start_acquire)
    myApp.actionStart.triggered.connect(start_acquire)
    myApp.BtnStop.clicked.connect(stop_acquire)
    myApp.actionStop.triggered.connect(stop_acquire)
    myApp.BtnSave.clicked.connect(save_csv)
    myApp.actionSave.triggered.connect(save_csv)

    myApp.actionExit.triggered.connect(quit)

    myApp.setWindowIcon(QIcon("assets/icon.jpg"))

    myApp.show()

    quit()