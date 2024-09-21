#! /usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Bool , Empty
from sensor_msgs.msg import JointState
import signal
import numpy as np
import quaternion
from termcolor import colored
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QCheckBox, QLineEdit, QPushButton, QRadioButton, QFormLayout , QComboBox , QAction
from visualization_msgs.msg import Marker
from PyQt5.QtCore import QTimer, Qt
from PyQt5.QtGui import QDoubleValidator  # Import QDoubleValidator
from datetime import datetime
import os

N_PART=6
TRAN_SB_TIME=2
TRAN_BS_TIME=2
STR_TIME=1
BNT_TIME=5

q_head_curr=np.zeros(4)
q_ref_curr=np.zeros(4)

trial_names= ['Practice','Free Part1', 'Free Part2','Low Stiff Part1', 'Low Stiff Part2', 'Medium Stiff Part1', 'Medium Stiff Part2', 'High Stiff Part1', 'High Stiff Part2']
trial_names= ['Practice','Free Part1', 'Low Stiff Part1',  'Medium Stiff Part1', 'High Stiff Part1']
abreviations= [tn[0]+tn[-1] for tn in trial_names]
trial_abrev={}
[trial_abrev.update({n:sn}) for n , sn in zip(trial_names,abreviations) ]


class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('GUI commands')
        self.record = { "cond": False , "filename": ""}
        self.rec_fold = "./EMBC/"
        self.rec_fold = "./I_beam_brace/"
        self.initUI()
        rospy.init_node('gui_node')  # Initialize the ROS node

    def initUI(self):


        #Main layout
        layout = QVBoxLayout()

        #self.save_checkbox  = QCheckBox('Save File')
        #layout.addWidget(self.save_checkbox )

        name_label = QLabel('Name:')
        #self.Pnumber_entry = QLineEdit()
        self.Pnumber_entry = QComboBox()
        for i in range(N_PART):
            self.Pnumber_entry.addItem('Part {}'.format(i+1))
        self.Pnumber_entry.addItem('Test'.format(i+1))
        layout.addWidget(name_label)
        layout.addWidget(self.Pnumber_entry)

        language_label = QLabel('Language:')
        self.language_combo = QComboBox()
        self.language_combo.addItem('English')
        self.language_combo.addItem('Japanese')
        layout.addWidget(language_label)
        layout.addWidget(self.language_combo)

        checkbox_list = QVBoxLayout()
        checkboxes = []
        checkbox_names = trial_names
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



        # Create a new layout for settings (initially hidden)
        self.settings_layout = QVBoxLayout()
        # Create a form layout for settings
        self.form_layout = QFormLayout()

        # Add a QLineEdit for Transition Time with QDoubleValidator
        
        self.time_validator = QDoubleValidator()
        self.time_validator.setNotation(QDoubleValidator.StandardNotation)

        time_RosParams = ["/ExpSeq/Time/" + phase for phase in ["straight","transitionsb","bent","transitionbs"]]
        time_Labels = ["Time "+ phase for phase in ["straight","transitionsb","bent","transitionbs"]]
        time_STD_durs=[STR_TIME,TRAN_SB_TIME,BNT_TIME,TRAN_BS_TIME]

        for tPar , tLab , stdDur in zip(time_RosParams,time_Labels,time_STD_durs):
            time_edit  = QLineEdit(tPar.split("/")[-1])
            time_edit .setValidator(self.time_validator)
            time_edit .setText(str(stdDur))
            self.apply_settings(tPar,time_edit)
            # Connect the textChanged signal to apply_settings automatically
            
            f=lambda: self.apply_settings(tPar,time_edit)
            print(f is (lambda: self.apply_settings(tPar,time_edit)))
            time_edit.returnPressed.connect(lambda tPar=tPar, time_edit=time_edit: self.apply_settings(tPar,time_edit))
            self.form_layout.addRow(QLabel(tLab), time_edit )
        
        self.set_language()
        self.language_combo.currentTextChanged.connect(self.set_language)

        # Add the form layout to the settings layout
        self.settings_layout.addLayout(self.form_layout)

        self.hidding_settings_layout()
        # Create a button to show/hide the settings layout
        settings_button = QPushButton('Settings')
        settings_button.clicked.connect(self.toggle_settings_layout)
        layout.addWidget(settings_button)

        # Add the settings layout to the main layout
        layout.addLayout(self.settings_layout)

        self.setLayout(layout)

        today_date = datetime.today().strftime('%Y_%m_%d')
        self.folder_path = os.path.join(self.rec_fold, today_date)
                    
        if not os.path.exists(self.folder_path):
            os.makedirs(self.folder_path)


        self.usr_mrk_pub = rospy.Subscriber('/adq/adq/Marker/Head', Marker, self.usr_mrk_callback)
        self.ref_mrk_pub= rospy.Subscriber('/adq/adq/Marker/Ref', Marker, self.ref_mrk_callback)
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
        
    
    def usr_mrk_callback(self, msg):
        global q_head_curr
        q_head_curr=np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])

    def ref_mrk_callback(self, msg):
        global q_ref_curr,q_head_curr
        q_ref_curr=np.array([msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        

        
        qhr_arr= np.append(q_head_curr,q_ref_curr,axis=0)

        if self.record["cond"]:# and self.save_checkbox.isChecked():
                fn = self.record["filename"]
                Pname=self.Pnumber_entry.currentText()
                P_name_short=Pname[0]+Pname[-2:]
                #print("algo")
                filename = os.path.join(self.folder_path , f"{P_name_short}_{fn}_GUI_logger.txt")
                with open(filename, 'a') as file:
                    file.write(','.join(str(comp) for comp in qhr_arr) + '\n')

    def send_demo_request(self):
        self.demo_pub.publish(True)
        self.showMinimized()

    def send_abort_request(self):
        self.record["cond"] = False
        self.abort_pub.publish(True)

    def send_start_request(self):
        name = self.Pnumber_entry.currentText()
        try:
            current_trial = next(filter(lambda checkbox: checkbox.isChecked(), self.findChildren(QRadioButton)))
        except(StopIteration):
            # The iterator was exausted because there is no selected option
            print(colored(":"*80, "red"))
            print(colored("Select an option! Onegai :)", "red"))
            print(colored(":"*80, "red"))
        else:

            if not current_trial.text() == 'Practice':
                              
                    time_now=datetime.today().strftime('%H_%M_%S')
                    self.record.update({"cond":True ,"filename": trial_abrev[current_trial.text()]+time_now})
                    times=[self.form_layout.itemAt(i,1).widget() for i in range (4)]
                    cicle_time=np.array([float(t.text()) for t in times]).sum()
                    n_cicles = 8                                                         # The number of cicles is asumed from the reference script
                    trial_time=cicle_time*n_cicles
                    print(f"Trial time: {trial_time}")
                    QTimer.singleShot(trial_time*1000, self.elapsed_timer_callback)
                    #QTimer.singleShot(240000, self.elapsed_timer_callback)
                    self.start_pub.publish(True)
            else:   
                    print(":"*80)
                    print(current_trial.text())
                    self.demo_pub.publish(True)        
            self.showMinimized()
        

    def send_reset_request(self):
        print("orientation was reset")
        self.reset_pub.publish(True)
    
    def elapsed_timer_callback(self):
        #self.record["cond"] = False

        fn = self.record["filename"]
        Pname=self.Pnumber_entry.currentText()
        P_name_short=Pname[0]+Pname[-2:]

        filename = os.path.join(self.folder_path , f"{P_name_short}_{fn}.txt")
        with open(filename, 'w') as file:
            file.write('Timing_seq: "straight","transitionsb","bent","transitionbs"'+ '\n')
            time_RosPars = ["/ExpSeq/Time/" + phase for phase in ["straight","transitionsb","bent","transitionbs"]]           
            file.write(','.join(str(rospy.get_param(pN)) for pN in time_RosPars) + '\n')
            sequ=rospy.get_param('/adq/reference/sequence')
            sequ=sequ[1:-1].split("\n ")
          
            for target in sequ:
                print(target)
                file.write(target[1:-1]  + '\n')
        self.record["cond"]=False

        print("Timer end")
        self.setVisible(True)

    def hidding_settings_layout(self,switch=False):
        # Toggle the visibility of the settings layout
        #self.settings_visible = not self.settings_visible
        for i in range(self.form_layout.count()):
            widget = self.form_layout.itemAt(i).widget()
            if widget:
                widget.setVisible(not widget.isVisible() and switch)

    def toggle_settings_layout(self):self.hidding_settings_layout(switch=True)

    def apply_settings(self,paramName,time_edit):
        # Implement your settings functionality here
        new_time = time_edit.text()

        # Validate the value using the validator
        if self.time_validator.validate(new_time, 0)[0] == QDoubleValidator.Acceptable:
            # Convert the value to float
            new_time = float(new_time)

            # Set the ROS parameter
            rospy.set_param(paramName, new_time)

            print(f"Transition Time: {new_time} (ROS parameter [{paramName} updated)")
        else:
            print("Invalid transition time entered.")

    def set_language(self):
        new_lang=self.language_combo.currentText()
        rospy.set_param("/ExpSeq/Lang", new_lang)
        print(f"New Language: {new_lang} (ROS parameter /ExpSeq/Lang updated)")






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