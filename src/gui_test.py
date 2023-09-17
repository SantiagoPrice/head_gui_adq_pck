#! /usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Bool , Empty
from sensor_msgs.msg import JointState
import signal

from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QCheckBox, QLineEdit, QComboBox, QPushButton , QRadioButton , QAction
from PyQt5.QtCore import QTimer
N_PART=11

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('GUI commands')
        self.initUI()
        self.record = { "cond": False , "filename": ""}
        self.rec_fold = "/home/santiago/catkin_ws/src/IMU_ADQ_pck/src/participant_record/"
        
        rospy.init_node('gui_node')  # Initialize the ROS node

    def initUI(self):
        layout = QVBoxLayout()

        #self.save_checkbox  = QCheckBox('Save File')
        #layout.addWidget(self.save_checkbox )

        

        name_label = QLabel('Name:')
        #self.name_entry = QLineEdit()
        self.name_entry = QComboBox()
        for i in range(6,N_PART):
            self.name_entry.addItem('Part {}'.format(i+1))
        self.name_entry.addItem('Test'.format(i+1))
        layout.addWidget(name_label)
        layout.addWidget(self.name_entry)

        language_label = QLabel('Language:')
        language_combo = QComboBox()
        language_combo.addItem('English')
        language_combo.addItem('Japanese')
        layout.addWidget(language_label)
        layout.addWidget(language_combo)

        checkbox_list = QVBoxLayout()
        checkboxes = []
        checkbox_names = ['Practice','Free Part1', 'Free Part2','Low Stiff Part1', 'Low Stiff Part2', 'Medium Stiff Part1', 'Medium Stiff Part2', 'High Stiff Part1', 'High Stiff Part2']
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

        close_debug = QTimer(self)
        close_debug.timeout.connect(self.ROSclosed)
        close_debug.start(2000)

        quit = QAction("Quit", self)
        quit.triggered.connect(self.closeEvent)

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

        self.joint_state_sub = rospy.Subscriber('/joint_states', JointState, self.joint_state_callback)
        self.demo_pub = rospy.Publisher('/adq/reference/trial', Bool, queue_size=10)
        self.abort_pub = rospy.Publisher('/adq/reference/abort', Bool, queue_size=10)
        self.start_pub = rospy.Publisher('/adq/reference/start', Bool, queue_size=10)
        self.reset_pub = rospy.Publisher('/adq/imu_rel/offset', Bool, queue_size=10)
        self.close_pub = rospy.Publisher('/rosout/stop_logging', Empty, queue_size=10)
    
    def ROSclosed(self):
         if rospy.is_shutdown():
              self.close()
    
    def closeEvent(self ,event):
        #rospy.loginfo("Publishing stop_logging message")
        print("GUI was closed")  
        rospy.signal_shutdown("Closing all the nodes")
        #self.close_pub.publish()
        
    
    def joint_state_callback(self, msg):
        
        if self.record["cond"]:# and self.save_checkbox.isChecked():
                fn = self.record["filename"]
                filename = self.rec_fold + f"{self.name_entry.currentText()}_{fn}.txt"
                with open(filename, 'a') as file:
                    file.write(','.join(str(value) for value in msg.position) + '\n')


    def send_demo_request(self):
        self.demo_pub.publish(True)
        self.showMinimized()

    def send_abort_request(self):
        self.record["cond"] = False
        self.abort_pub.publish(True)

    def send_start_request(self):
        name = self.name_entry.currentText()
        selected_checkbox = next(filter(lambda checkbox: checkbox.isChecked(), self.findChildren(QRadioButton)))
        if not selected_checkbox.text() == 'Practice':
                self.record.update({"cond":True ,"filename": selected_checkbox.text()})
                QTimer.singleShot(240000, self.start_timer_callback)
                self.start_pub.publish(True)
        else:
                self.demo_pub.publish(True)        
        self.showMinimized()
        

    def send_reset_request(self):
        print("orientation was reset")
        self.reset_pub.publish(True)
    
    def start_timer_callback(self):
        self.record["cond"] = False
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