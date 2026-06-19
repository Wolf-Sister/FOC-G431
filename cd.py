import sys
import serial
import re
from PyQt5 import QtCore, QtWidgets
import pyqtgraph as pg
from collections import deque

# ==================== 参数配置 ====================
SERIAL_PORT = 'COM6'      # 串口号
BAUD_RATE = 115200        # 波特率
MAX_POINTS = 500          # 屏幕上最多显示的实时点数（可设得很大，完全不卡）
# ==================================================

# 正则表达式：精准提取数据
data_pattern = re.compile(
    r"Elec:([-\d\.]+)\s*\|\s*Ia:([-\d\.\+]+)\s*Ib:([-\d\.\+]+)\s*Ic:([-\d\.\+]+)\s*\|\s*Sum:([-\d\.]+)"
)

class FocOscilloscope(QtWidgets.QMainWindow):
    def __init__(self):
        super().__init__()
        
        # 初始化数据缓冲区
        self.count = 0
        self.x_data = deque(maxlen=MAX_POINTS)
        self.elec_data = deque(maxlen=MAX_POINTS)
        self.ia_data = deque(maxlen=MAX_POINTS)
        self.ib_data = deque(maxlen=MAX_POINTS)
        self.ic_data = deque(maxlen=MAX_POINTS)
        self.sum_data = deque(maxlen=MAX_POINTS)
        
        # 打开串口
        try:
            self.ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.01)
            self.ser.reset_input_buffer()
            print(f"成功打开串口 {SERIAL_PORT}")
        except Exception as e:
            print(f"串口打开失败: {e}")
            sys.exit(1)
            
        self.init_ui()
        
        # 设置高频定时器，每 10ms 检查一次串口并刷新界面
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(10) # 10毫秒刷新一次

    def init_ui(self):
        self.setWindowTitle('FOC 实时数据示波器 (PyQtGraph)')
        self.resize(1000, 700)
        
        # 创建中央部件和布局
        central_widget = QtWidgets.QWidget()
        self.setCentralWidget(central_widget)
        layout = QtWidgets.QVBoxLayout(central_widget)
        
        # 创建 pyqtgraph 绘图窗口部件
        self.win = pg.GraphicsLayoutWidget()
        layout.addWidget(self.win)
        
        # 设置全局样式背景为深色（护眼、看起来更像示波器）
        pg.setConfigOption('background', 'k')
        pg.setConfigOption('foreground', 'w')
        
        # 子图1：专门画 Elec
        self.p1 = self.win.addPlot(title="Elec 角度/变量监控")
        self.p1.showGrid(x=True, y=True, alpha=0.3)
        self.p1.setLabel('left', 'Elec')
        self.curve_elec = self.p1.plot(pen=pg.mkPen('d', width=1.5), name="Elec") # 橙黄色
        
        self.win.nextRow() # 换行，创建第二个子图
        
        # 子图2：画三相电流和 Sum
        self.p2 = self.win.addPlot(title="Ia / Ib / Ic / Sum 电流监控")
        self.p2.showGrid(x=True, y=True, alpha=0.3)
        self.p2.setLabel('left', 'Current (A)')
        self.p2.setLabel('bottom', 'Samples')
        
        # 添加图例
        self.p2.addLegend()
        self.curve_ia = self.p2.plot(pen=pg.mkPen('r', width=1.5), name="Ia")     # 红色
        self.curve_ib = self.p2.plot(pen=pg.mkPen('g', width=1.5), name="Ib")     # 绿色
        self.curve_ic = self.p2.plot(pen=pg.mkPen('b', width=1.5), name="Ic")     # 蓝色
        # Sum 电流用白色虚线表示
        self.curve_sum = self.p2.plot(pen=pg.mkPen('w', width=1.5, style=QtCore.Qt.DashLine), name="Sum")
        
        # 让两个图的 X 轴联动滚动
        self.p1.setXLink(self.p2)

    def update_data(self):
        # 只要串口缓冲区有数据，就全部读完，防止丢帧和产生延迟
        data_added = False
        while self.ser.in_waiting > 0:
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                match = data_pattern.search(line)
                if match:
                    self.count += 1
                    self.x_data.append(self.count)
                    self.elec_data.append(float(match.group(1)))
                    self.ia_data.append(float(match.group(2)))
                    self.ib_data.append(float(match.group(3)))
                    self.ic_data.append(float(match.group(4)))
                    self.sum_data.append(float(match.group(5)))
                    data_added = True
            except Exception:
                pass
        
        # 如果有新数据进入，刷新波形线条
        if data_added:
            x = list(self.x_data)
            self.curve_elec.setData(x, list(self.elec_data))
            self.curve_ia.setData(x, list(self.ia_data))
            self.curve_ib.setData(x, list(self.ib_data))
            self.curve_ic.setData(x, list(self.ic_data))
            self.curve_sum.setData(x, list(self.sum_data))

    def closeEvent(self, event):
        # 窗口关闭时安全释放串口
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            print("串口已安全关闭。")
        event.accept()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = FocOscilloscope()
    window.show()
    sys.exit(app.exec_())