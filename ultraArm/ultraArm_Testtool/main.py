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
        # self.setWindowTitle('ultraArm Test Tool')
        # 使其窗口最前
        self.setWindowFlags(QtCore.Qt.WindowStaysOnTopHint)
        
        self.compile_program.clicked.connect(self.build_code)
        self.run_button.clicked.connect(self.run_program)
        self.close_button.clicked.connect(self.close_rviz)
        self.language_button.clicked.connect(self.language)
        self.flage = True
        self.retranslateUi()
        
    
    def language(self):
        if self.flage:
            self.flage = False
            self.retranslateUi()
            
        else:
            self.flage = True
            self.retranslateUi()
    
    def retranslateUi(self):
        if self.flage:
            _translate = QtCore.QCoreApplication.translate
            self.setWindowTitle("ultraArm ROS2 测试工具")
            self.compile_program.setText(_translate("MainWindow", "编译程序"))
            self.label.setText(_translate("MainWindow", "程序："))
            self.comboBox.setItemText(0, _translate("MainWindow", "滑块控制"))
            self.comboBox.setItemText(1, _translate("MainWindow", "rviz2"))
            self.run_button.setText(_translate("MainWindow", "运行"))
            self.close_button.setText(_translate("MainWindow", "  关闭"))
            self.language_button.setText(_translate("MainWindow", "简体中文"))
        else:
            _translate = QtCore.QCoreApplication.translate
            self.setWindowTitle("ultraArm ROS2 Test Tool")
            self.compile_program.setText(_translate("MainWindow", "compile program"))
            self.label.setText(_translate("MainWindow", "program："))
            self.comboBox.setItemText(0, _translate("MainWindow", "slider control"))
            self.comboBox.setItemText(1, _translate("MainWindow", "rviz2"))
            self.run_button.setText(_translate("MainWindow", "run"))
            self.close_button.setText(_translate("MainWindow", "  close"))
            self.language_button.setText(_translate("MainWindow", "Simplified English"))
    
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
            if self.flage:
                if self.comboBox.currentText() == 'rviz2':
                    self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'start test.....')
                    t1 = threading.Thread(target=self.rviz_test)
                    t1.setDaemon(True)
                    t1.start()
                    
                elif self.comboBox.currentText() == '滑块控制':
                    self.textBrowser.append('[' + str(current_time) + ']' + ' '+ 'start slider control......')
                    t1 = threading.Thread(target=self.rviz_test)
                    t1.setDaemon(True)
                    t1.start()
            else:
                if self.comboBox.currentText() == 'rviz2':
                    self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'start test.....')
                    t1 = threading.Thread(target=self.rviz_test)
                    t1.setDaemon(True)
                    t1.start()
                    
                elif self.comboBox.currentText() == 'slider control':
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
        if self.flage:
            if self.comboBox.currentText() == 'rviz2':
                os.system("gnome-terminal -e 'bash -c \"cd $HOME/colcon_ws/; source install/setup.bash; ros2 launch ultraarm test.launch.py; exec bash\"'")
            elif self.comboBox.currentText() == '滑块控制':
                os.system("gnome-terminal -e 'bash -c \"cd $HOME/colcon_ws/; source install/setup.bash; ros2 launch ultraarm slider_control.launch.py; exec bash\"'")
        else:
            if self.comboBox.currentText() == 'rviz2':
                os.system("gnome-terminal -e 'bash -c \"cd $HOME/colcon_ws/; source install/setup.bash; ros2 launch ultraarm test.launch.py; exec bash\"'")
            elif self.comboBox.currentText() == 'slider control':
                os.system("gnome-terminal -e 'bash -c \"cd $HOME/colcon_ws/; source install/setup.bash; ros2 launch ultraarm slider_control.launch.py; exec bash\"'")
    
    def close_rviz(self):
        current_time = self.get_current_time()
        try:
            if self.flage:
                if self.comboBox.currentText() == 'rviz2':
                    self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'rviz2 test closed ! ! !')
                    t1 = threading.Thread(target=self.close_test)
                    t1.setDaemon(True)
                    t1.start()
                elif self.comboBox.currentText() == '滑块控制':
                    self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'slider control closed ! ! !')
                    t1 = threading.Thread(target=self.close_test)
                    t1.setDaemon(True)
                    t1.start()
            else:
                if self.comboBox.currentText() == 'rviz2':
                    self.textBrowser.append('[' + str(current_time) + ']' + ' ' + 'rviz2 test closed ! ! !')
                    t1 = threading.Thread(target=self.close_test)
                    t1.setDaemon(True)
                    t1.start()
                elif self.comboBox.currentText() == 'slider control':
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
        if self.flage:
            if self.comboBox.currentText() == 'rviz2':
                
                os.system("ps -ef | grep -E " + "test.launch.py" + " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
            
            elif self.comboBox.currentText() == '滑块控制':
                
                os.system("ps -ef | grep -E " + "slider_control.launch.py" + " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
        else:
            if self.comboBox.currentText() == 'rviz2':
            
                os.system("ps -ef | grep -E " + "test.launch.py" + " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")
            
            elif self.comboBox.currentText() == 'slider control':
                
                os.system("ps -ef | grep -E " + "slider_control.launch.py" + " | grep -v 'grep' | awk '{print $2}' | xargs kill -9")    
        
if __name__ == "__main__":
    app = QApplication(sys.argv)
    main_window = Ultraarm_Window()
    main_window.show()
    sys.exit(app.exec_())
