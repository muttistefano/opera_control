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
from std_srvs.srv import Trigger

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
        self._daemon_ready   = True
        self._stop_requested = False
        self.__ros_is_up     = False
        
        # signal.signal(signal.SIGTERM, self.__signal_handler)
        # signal.signal(signal.SIGINT, self.__signal_handler)

        self.th_monitor_hosts = threading.Thread(target=self.__hosts_check)
        self.th_monitor_hosts.start()

        print("\n construction finished \n")
        
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
            
            sys.stdout.write(str(self._LAT_rp1) + " " + str(self._LAT_rp2) + " " + str(self._LAT_rp3) + " " + str(self._LAT_rp4) + "\n")
            sys.stdout.write(str(self._HOST_UP_rp1) + " " + str(self._HOST_UP_rp2) + " " + str(self._HOST_UP_rp3) + " " + str(self._HOST_UP_rp4) + "\n")
            sys.stdout.flush()

            if(self._HOST_UP_rp1 and self._HOST_UP_rp2 and self._HOST_UP_rp3 and self._HOST_UP_rp4 and self._daemon_ready):
                self._daemon_ready = False
                print("setting daemon ready")
                systemd.daemon.notify('READY=1')

            try:
                rosgraph.Master('/rostopic').getPid()
                self.__ros_is_up = True
                print("Roscore UP!")
            except Exception as e:
                self.__ros_is_up = False
                print("Unable to communicate with master!")

            time.sleep(10)

    # def __signal_handler(self,sig, frame):
    #     self._stop_requested = True
    #     print('Closing the opera control server')

    @route('/', methods=['GET', 'POST'])
    def index(self):
        print(request.method)
        if request.method == 'POST':
            print(request.form)
            if 'btn1up' in request.form :
                print("btn1up")
                return render_template('index.html')
            elif 'btn1down' in request.form :
                print("btn1down")
                return render_template('index.html')
            elif 'btn2up' in request.form :
                print("btn2up")
                return render_template('index.html')
            elif 'btn2down' in request.form :
                print("btn2down")
                return render_template('index.html')
            elif 'btn3up' in request.form :
                print("btn3up")
                return render_template('index.html')
            elif 'btn3down' in request.form :
                print("btn3down")
                return render_template('index.html')
            elif 'btn4left' in request.form :
                print("btn4left")
                return render_template('index.html')
            elif 'btn4right' in request.form :
                print("btn4right")
                return render_template('index.html')
            elif 'stop' in request.form :
                print("stop")
                return render_template('index.html')
            elif 'start' in request.form :
                print("start")
                return render_template('index.html')
            else:
                print("bho")
                return render_template('index.html')
        if request.method == 'GET':
            print("get")
            return render_template('index.html')




if __name__ == '__main__':




    opera_check.register(app,route_base = '/')
    app.run(host="0.0.0.0",debug=False,threaded=True) 



