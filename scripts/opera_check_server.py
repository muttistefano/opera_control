#!/usr/bin/env python3
import subprocess
import ntplib
import sys
import time
import systemd.daemon
import threading
from flask import Flask, render_template,request 
from flask_classful import FlaskView, route
import rosgraph
import signal
import ros
import rospy
import os
from std_srvs.srv import Trigger, TriggerRequest
from pkg_rp_msgs.srv import  MovePlatSrv, MovePlatSrvRequest
import RPi.GPIO as GPIO
from std_msgs.msg import Bool
import subprocess

app = Flask(__name__,static_url_path='/templates')


class opera_check(FlaskView):

    def __init__(self):
        self._HOST_UP_rp1 = False
        self._HOST_UP_rp2 = False
        self._HOST_UP_rp3 = False
        self._HOST_UP_rp4 = False
        self._LAT_rp1  = 0.0
        self._LAT_rp2  = 0.0
        self._LAT_rp3  = 0.0
        self._LAT_rp4  = 0.0
        self._stop_requested = False
        self.__ros_is_up     = False
        


        # signal.signal(signal.SIGTERM, self.__signal_handler)
        # signal.signal(signal.SIGINT, self.__signal_handler)
        systemd.daemon.notify('READY=0')
        self.__hosts_check_single()
        self.__start_opera = rospy.ServiceProxy('/opera_control_node/Start_service', Trigger)
        self.__stop_opera = rospy.ServiceProxy('/opera_control_node/Stop_service', Trigger)
        
        self.__move1 = rospy.ServiceProxy('/rp_control_rp1/MovePlat_rp1', MovePlatSrv)
        self.__move2 = rospy.ServiceProxy('/rp_control_rp2/MovePlat_rp2', MovePlatSrv)
        self.__move3 = rospy.ServiceProxy('/rp_control_rp3/MovePlat_rp3', MovePlatSrv)
        self.__move4 = rospy.ServiceProxy('/rp_control_rp4/MovePlat_rp4', MovePlatSrv)


        self.th_monitor_hosts = threading.Thread(target=self.__hosts_check)
        self.th_monitor_hosts.start()

        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(27, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        # self.th_off = threading.Thread(target=self.__off_butt)
        # self.th_off.start()

        sys.stdout.write("\n construction finished \n")
        sys.stdout.flush()
    

  
    # def __off_butt(self):
    #     while True:
    #         GPIO.wait_for_edge(27, GPIO.RISING)
    #         rospy.init_node("offing",disable_signals=True)
    #         in_msg = TriggerRequest()
    #         try:
    #             self.__stop_opera(in_msg)
    #         except:
    #             print("no state machine up")
    #         self._stop_requested = True
    #         print("OFF button pressed!")
    #         offer = rospy.wait_for_message("/opera_control_node/offer", Bool)
    #         print("OFF state received")
    #         rospy.signal_shutdown("off")
    #         os.system("rosnode kill --all")
    #         rospy.sleep(10)
    #         self.__turn_off_all()




    def __check_host(self,host):
        ret = subprocess.call(['timeout','1.5','ping', '-c', '1', host], stdout=open('/dev/null', 'w'), stderr=open('/dev/null', 'w'))
        return ret == 0

    def __check_time_host(self,host):
        try:
            c = ntplib.NTPClient()
            response = c.request(host, version=3)
            chk = True if (abs(response.offset) < 0.5 and response.offset != 0.0 ) else False
            if chk:
                return float(response.offset * 1000)
        except:
            return False

    def __hosts_check_single(self):

        while not (self._HOST_UP_rp1 and self._HOST_UP_rp2 and self._HOST_UP_rp3 and self._HOST_UP_rp4):
            
            self._HOST_UP_rp1  = self.__check_host("192.168.50.101")
            self._HOST_UP_rp2  = self.__check_host("192.168.50.102")
            self._HOST_UP_rp3  = self.__check_host("192.168.50.103")
            self._HOST_UP_rp4  = self.__check_host("192.168.50.104")

            self._LAT_rp1  = self.__check_time_host("192.168.50.101")
            self._LAT_rp2  = self.__check_time_host("192.168.50.102")
            self._LAT_rp3  = self.__check_time_host("192.168.50.103")
            self._LAT_rp4  = self.__check_time_host("192.168.50.104")
            
            sys.stdout.write(str(self._LAT_rp1) + " " + str(self._LAT_rp2) + " " + str(self._LAT_rp3) + " " + str(self._LAT_rp4) + "\n")
            sys.stdout.write(str(self._HOST_UP_rp1) + " " + str(self._HOST_UP_rp2) + " " + str(self._HOST_UP_rp3) + " " + str(self._HOST_UP_rp4) + "\n")
            
            sys.stdout.flush()

            if(self._HOST_UP_rp1 and self._HOST_UP_rp2 and self._HOST_UP_rp3 and self._HOST_UP_rp4):
                sys.stdout.write("setting daemon ready \n")
                systemd.daemon.notify('READY=1')
                # subprocess.Popen("/opt/openoffice.org3/program/scalc")
                os.system("sudo systemctl start opera_launch.service")
            else:
                sys.stdout.write("HOSTS not up \n")
            
            sys.stdout.flush()
            time.sleep(5)

        while not self.__ros_is_up:
            try:
                rosgraph.Master('/rostopic').getPid()
                self.__ros_is_up = True
                # sys.stdout.write("Roscore UP! \n")
            except Exception as e:
                sys.stdout.write("Unable to communicate with master! \n")
                time.sleep(2)
            sys.stdout.flush()

        time.sleep(10)

    def __hosts_check(self):

        while True:
            
            self._HOST_UP_rp1  = self.__check_host("192.168.50.101")
            self._HOST_UP_rp2  = self.__check_host("192.168.50.102")
            self._HOST_UP_rp3  = self.__check_host("192.168.50.103")
            self._HOST_UP_rp4  = self.__check_host("192.168.50.104")

            self._LAT_rp1  = self.__check_time_host("192.168.50.101")
            self._LAT_rp2  = self.__check_time_host("192.168.50.102")
            self._LAT_rp3  = self.__check_time_host("192.168.50.103")
            self._LAT_rp4  = self.__check_time_host("192.168.50.104")
            
            # sys.stdout.write("PERIODIC CHECK \n")
            # sys.stdout.write(str(self._LAT_rp1) + " " + str(self._LAT_rp2) + " " + str(self._LAT_rp3) + " " + str(self._LAT_rp4) + "\n")
            # sys.stdout.write(str(self._HOST_UP_rp1) + " " + str(self._HOST_UP_rp2) + " " + str(self._HOST_UP_rp3) + " " + str(self._HOST_UP_rp4) + "\n")

            try:
                rosgraph.Master('/rostopic').getPid()
                self.__ros_is_up = True
                # sys.stdout.write("Roscore UP! \n")
            except Exception as e:
                systemd.daemon.notify('READY=0')
                sys.stdout.write("Unable to communicate with master! \n")

            sys.stdout.flush()
            time.sleep(10)


    def __turn_off_all(self):
        os.system("ssh ubuntu@rp4 'sudo poweroff' ")
        os.system("ssh ubuntu@rp3 'sudo poweroff' ")
        os.system("ssh ubuntu@rp2 'sudo poweroff' ")
        time.sleep(2)
        os.system("sudo poweroff")


    @route('/', methods=['GET', 'POST'])
    def index(self):
        sys.stdout.write(request.method)
        if request.method == 'POST':
            # print(request.form)
            if 'btn1down' in request.form :
                sys.stdout.write("btn1down")
                req_ms = MovePlatSrvRequest()
                req_ms.position_command = 100
                self.__move1(req_ms)
                return render_template('index.html')
            elif 'btn2down' in request.form :
                req_ms = MovePlatSrvRequest()
                req_ms.position_command = 100
                self.__move2(req_ms)
                return render_template('index.html')
            elif 'btn3down' in request.form :
                req_ms = MovePlatSrvRequest()
                req_ms.position_command = 100
                self.__move3(req_ms)
                return render_template('index.html')
            elif 'btn4left' in request.form :
                req_ms = MovePlatSrvRequest()
                req_ms.position_command = 100
                self.__move4(req_ms)
                return render_template('index.html')
            elif 'stop' in request.form :
                rospy.logerr("Its the web")
                sys.stdout.write("stop")
                in_msg = TriggerRequest()
                self.__stop_opera(in_msg)
                self._stop_requested = True
                return render_template('index.html')
            elif 'start' in request.form :
                in_msg = TriggerRequest()
                self.__start_opera(in_msg)
                sys.stdout.write("start")
                return render_template('index.html')
            elif 'kill' in request.form :
                rospy.signal_shutdown("off")
                os.system("rosnode kill --all")
                rospy.sleep(10)
                self.__turn_off_all()
            elif 'off' in request.form :
                in_msg = TriggerRequest()
                try:
                    self.__stop_opera(in_msg)
                except:
                    print("no state machine up")
                self._stop_requested = True
                print("OFF WEB button pressed!")
                rospy.init_node("offing",disable_signals=True)
                offer = rospy.wait_for_message("/opera_control_node/offer", Bool)
                print("OFF state received")
                rospy.signal_shutdown("off")
                os.system("rosnode kill --all")
                rospy.sleep(10)
                self.__turn_off_all()
                return render_template('index.html')
            else:
                sys.stdout.write("bho")
                return render_template('index.html')
        if request.method == 'GET':
            sys.stdout.write("get")
            return render_template('index.html')






if __name__ == '__main__':

    opera_check.register(app,route_base = '/')
    app.run(host="0.0.0.0",debug=False,threaded=True) 
    
    



