# -*- coding: utf-8 -*-
import sys
import os
import time
import threading
import subprocess
import traceback
from PyQt5 import QtCore
from PyQt5.QtWidgets import QMainWindow, QApplication
from ultraarm_window import Ui_MainWindow as ultraArm_window


class Ultraarm_Window(ultraArm_window, QMainWindow):
    def __init__(self):
        super(ultraArm_window, self).__init__()
        self.setupUi(self)
        self.setWindowTitle('ultraArm Test Tool')
        # 使其窗口最前
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        
        self.compile_program.clicked.connect(self.build_code)
        self.run_button.clicked.connect(self.run_program)
        self.close_button.clicked.connect(self.close_rviz)
        
        
    def get_current_time(self):
        current_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(time.time()))
        return current_time
        
    def build_code(self):
        current_time = self.get_current_time()
        build_mes = 'start build...'
        self.textBrowser.append('[' + str(current_time) + ']' + ' ' + build_mes)
        t1 = threading.Thread(target=self.start_bulid)
        t1.setDaemon(True)
        t1.start()
        
    def start_bulid(self):
        res = subprocess.getoutput("cd $HOME/colcon_ws; colcon build;")
        self.textBrowser.append(res)
        
    def run_program(self):
        current_time = self.get_current_time()
        try:
            if self.comboBox.currentText() == 'rviz2测试':
                self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'start test.....')
                t1 = threading.Thread(target=self.rviz_test)
                t1.setDaemon(True)
                t1.start()
                
            elif self.comboBox.currentText() == '滑块控制':
                self.textBrowser.append('[' + str(current_time) + ']' + ' '+ 'start slider control......')
                t1 = threading.Thread(target=self.rviz_test)
                t1.setDaemon(True)
                t1.start()
                
        except Exception as e:
            e = traceback.format_exc()
            with open("./error.log", "a") as f:
                f.write(e)
            self.textBrowser.append('[' + str(current_time) + ']' + ' '+ str(e))
            
    
    def rviz_test(self):
        if self.comboBox.currentText() == 'rviz2测试':
            os.system("gnome-terminal -e 'bash -c \"cd $HOME/colcon_ws/; source install/setup.bash; ros2 launch ultraarm test.launch.py; exec bash\"'")
        elif self.comboBox.currentText() == '滑块控制':
            os.system("gnome-terminal -e 'bash -c \"cd $HOME/colcon_ws/; source install/setup.bash; ros2 launch ultraarm slider_control.launch.py; exec bash\"'")
        
    
    def close_rviz(self):
        current_time = self.get_current_time()
        try:
            if self.comboBox.currentText() == 'rviz2测试':
                self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'rviz2 test closed ! ! !')
                t1 = threading.Thread(target=self.close_test)
                t1.setDaemon(True)
                t1.start()
            elif self.comboBox.currentText() == '滑块控制':
                self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'slider control closed ! ! !')
                t1 = threading.Thread(target=self.close_test)
                t1.setDaemon(True)
                t1.start()
        
        except Exception as e:
            e = traceback.format_exc()
            with open("./error.log", "a") as f:
                f.write(e)
            self.textBrowser.append('[' + str(current_time) + ']' + ' '+ str(e))
            
    def close_test(self):
        current_time = self.get_current_time()
        if self.comboBox.currentText() == 'rviz2测试':
            
            os.system("ps -ef | grep -E " + "test.launch.py" + " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
        
        elif self.comboBox.currentText() == '滑块控制':
            
            os.system("ps -ef | grep -E " + "slider_control.launch.py" + " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
        
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = Ultraarm_Window()
    main_window.show()
    sys.exit(app.exec_())