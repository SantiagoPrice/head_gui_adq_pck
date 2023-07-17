#! /usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Bool , Empty
from sensor_msgs.msg import  Imu
import signal
import numpy as np
import quaternion
from sound_play.libsoundplay import SoundClient
import os
import time
from datetime import datetime

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QLineEdit, QComboBox, QPushButton , QRadioButton
from PyQt5.QtGui import QFont
from PyQt5.QtCore import QTimer

N_PART=6
AU_PATH = "/home/santiago/catkin_ws/src/IMU_ADQ_pck/src/voice commands"



def yawPitchRoll(q, ls = False):
    """Function that converts quaternion to the yaw pitch roll representation
    This representation corresponds to a tait-bryan rotation of xyz-order.
    Input: 
        .q: quaternion
    Output:
        yaw , pitch roll in radians"""

    yaw = np.arctan2(2.0*(q.y*q.z + q.w*q.x), q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z);
    pitch = np.arcsin(-2.0*(q.x*q.z - q.w*q.y));
    roll = np.arctan2(2.0*(q.x*q.y + q.w*q.z), q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z);
    if ls:
        return [yaw , pitch , roll]
    else:
        return ((np.array((yaw , pitch , roll))+np.pi)%(2*np.pi)-np.pi) * 180/np.pi
    

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('GUI commands')
        self.initUI()
        self.record = { "cond": False , "filename": ""}
        self.rec_fold = "/home/santiago/catkin_ws/src/IMU_ADQ_pck/src"
        self.soundhandle = SoundClient()
        self.offset_imu_pose = np.quaternion(1.,0.,0.,0.)
        self.switch_offset=False 
        rospy.init_node('gui_node')  # Initialize the ROS node

    def initUI(self):
        layout = QVBoxLayout()

        #self.save_checkbox  = QCheckBox('Save File')
        #layout.addWidget(self.save_checkbox )

        # QLineEdit widget for containing folder
        self.folder_entry = QLineEdit()
        self.folder_entry.setPlaceholderText('Folder Name')
        layout.addWidget(self.folder_entry)
        

        name_label = QLabel('Name:')
        #self.name_entry = QLineEdit()
        self.name_entry = QComboBox()
        for i in range(N_PART):
            self.name_entry.addItem('Part {}'.format(i+1))
        self.name_entry.addItem('Test'.format(i+1))
        layout.addWidget(name_label)
        layout.addWidget(self.name_entry)


        self.label = QLabel('Joint State Value:')
        self.label.setFont(QFont('Arial', 28, QFont.Bold))  # Set font size and bold
        layout.addWidget(self.label)

        checkbox_list = QVBoxLayout()
        checkboxes = []
        checkbox_names = [f'{int((i+1)*45/5):2d}' for i in range(5)]
        for name in checkbox_names:
            checkbox = QRadioButton(name)
            checkboxes.append(checkbox)
            checkbox_list.addWidget(checkbox)
        layout.addLayout(checkbox_list)

        buttons_layout = QVBoxLayout()
        #demonstration_button = QPushButton('Demonstration')
        abort_button = QPushButton('Abort')
        start_button = QPushButton('Start Experiment')
        reset_button = QPushButton('Reset Q')
        #demonstration_button.clicked.connect(self.send_demo_request)
        abort_button.clicked.connect(self.send_abort_request)
        start_button.clicked.connect(self.send_start_request)
        reset_button.clicked.connect(self.send_reset_request)
        #buttons_layout.addWidget(demonstration_button)
        buttons_layout.addWidget(abort_button)
        buttons_layout.addWidget(start_button)
        buttons_layout.addWidget(reset_button)
        layout.addLayout(buttons_layout)

        self.setLayout(layout)

        self.joint_state_sub = rospy.Subscriber('/imu_rel/imu2_realtive_imu1/data', Imu, self.IMU_callback)
        self.demo_pub = rospy.Publisher('/adq/reference/trial', Bool, queue_size=10)
        self.abort_pub = rospy.Publisher('/adq/reference/abort', Bool, queue_size=10)
        self.start_pub = rospy.Publisher('/adq/reference/start', Bool, queue_size=10)
        self.reset_pub = rospy.Publisher('/adq/imu_rel/offset', Bool, queue_size=10)
        self.close_pub = rospy.Publisher('/rosout/stop_logging', Empty, queue_size=10)
    
    def closeEvent(self, event):
        #rospy.loginfo("Publishing stop_logging message")  
        rospy.signal_shutdown("Closing all the nodes")
        #self.close_pub.publish()
        super().closeEvent(event)
    
    def IMU_callback(self, q):
        angles= np.quaternion(q.orientation.w, q.orientation.x, q.orientation.y, q.orientation.z)

        if self.switch_offset:
            self.offset_imu_pose=1/angles
            self.switch_offset=False
        

        rpy= yawPitchRoll(self.offset_imu_pose * angles, ls = False)
        self.label.setText(f'Joint State Value: y{rpy[0]:.1f} p{rpy[1]:.1f} r{rpy[2]:.1f}')
        if self.record["cond"]:# and self.save_checkbox.isChecked():

                filename = "{}.txt".format(self.record["filename"])

                # NEW: Get the containing folder from the folder_entry widget
                folder = os.path.join(self.rec_fold ,self.folder_entry.text())

                # Create the folder if it doesn't exist
                if folder and not os.path.exists(folder):
                    os.makedirs(folder)

                # Use the folder as part of the filename
                if folder:
                    filename = os.path.join(folder, filename)
                    


                with open(filename, 'a') as file:
                    if file.tell() == 0:
                        file.write(datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3] + '\n')
                    file.write(','.join(str(ang) for ang in rpy) + '\n')


    def send_demo_request(self):
        self.demo_pub.publish(True)
        self.showMinimized()

    def send_abort_request(self):
        self.record["cond"] = False
        self.abort_pub.publish(True)

    def send_start_request(self):
        name = self.name_entry.currentText()
        selected_checkbox = next(filter(lambda checkbox: checkbox.isChecked(), self.findChildren(QRadioButton)))
        self.record.update({"cond":True ,"filename": selected_checkbox.text()})
        self.soundhandle.playWave(AU_PATH+r"/trial start.wav",1)
        QTimer.singleShot(11000, self.start_timer_callback)
        self.folder_entry.setEnabled(False)
        

    def send_reset_request(self):
        self.switch_offset=True
        
    
    def start_timer_callback(self):
        self.record["cond"] = False
        self.soundhandle.playWave(AU_PATH+r"/the sequence was terminated.wav",1)
        print("Timer end")
        self.setVisible(True)



signal.signal(signal.SIGINT, exit)
signal.signal(signal.SIGTERM, exit)

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()

    try:
        sys.exit(app.exec())
    except KeyboardInterrupt:
        window.close()