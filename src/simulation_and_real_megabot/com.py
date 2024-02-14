from threading import Thread, Lock

import os
import os.path
import time
import traceback
from serial import Serial, SerialException
import sys
import re
import com_data

from subprocess import Popen,PIPE
import select



class ControlerHandler(Thread):
    def __init__(self,logfile=None):
        Thread.__init__(self)
        self.serials=[]
        if logfile!=None:
            self.logfile=open(logfile,'w')
        else:
            self.logfile=None
        for i in range(10):
            pname="/dev/ttyACM%d"%i
            print("try ",pname)
            if os.path.exists(pname)==False:
                continue
            try:
                port=Serial(port=pname, baudrate=115200, timeout=0.2)
                print('port ',pname,' opened')
            except Exception as e:
                traceback.print_exc()
                continue
            c=0
            while port.isOpen()==False and c<10:
                time.sleep(0.5)
                c+=1
            if c==10:
                port.close()
                continue
            self.serials.append(port)
        print(self.serials)
        self.print_str_msg=False
        self.shouldStop=False
        self.start_time=time.time()
        self.legs=[]
        for a in [1,2,3,4]:
            self.legs.append([{'enabled':False,'position':0,'target':0,'last_order':0,'time':0},
                              {'enabled':False,'position':0,'target':0,'last_order':0,'time':0},
                              {'enabled':False,'position':0,'target':0,'last_order':0,'time':0}])
        self.start()
    def stop(self):
        self.shouldStop=True

    def send_move(self,leg,actuator,position,pwm=1.0):
        """ leg should be 1,2,3 or 4
        actuator 1,2 or 3
        position is in m
        pwm is used to limit maximal pwm value is between 0 (no move) and 1.0 (max)
        """
        o=com_data.Order()
        o.move(leg,actuator,position,pwm)
        data=o.pack()
        for s in self.serials:
            s.write(data)

    def send_stop(self):
        o=com_data.Order()
        o.stop()
        data=o.pack()
        for s in self.serials:
            s.write(data)

    def send_info(self,freq):
        o=com_data.Order()
        o.print(freq)
        data=o.pack()
        for s in self.serials:
            s.write(data)

    def print_string_msg(self,b):
        self.print_str_msg=b
    
    def run(self):
        while self.shouldStop==False:
            action=False
            for s in self.serials:
                if s.in_waiting:
                    action=True
                    d=s.read(500)
                    p=0
                    while p<len(d):
                        if d[p]==0xA5 and (len(d)-p)>=com_data.Infos.size:
                            i=com_data.Infos(d[p:])
                            if i.check():
                                print(".",end='')
                                for a in range(3):
                                    self.legs[i.leg-1][a]['enabled']=True
                                    self.legs[i.leg-1][a]['position']=i.actuators[a].position/10000.0
                                    self.legs[i.leg-1][a]['target']=i.actuators[a].target/10000.0
                                    self.legs[i.leg-1][a]['pwm']=i.actuators[a].pwm/10000.0
                                    self.legs[i.leg-1][a]['pwmtarget']=i.actuators[a].pwmtarget/10000.0
                                    self.legs[i.leg-1][a]['status']=i.actuators[a].status
                                    self.legs[i.leg-1][a]['time']=time.time()-self.start_time
                                    if self.logfile!=None:
                                        self.logfile.write("%d;%d;%d;%.3f;%.3f;%.3f;%.3f;%d;%.3f\n"%(i.leg,a+1,self.legs[i.leg-1][a]['status'],
                                                                        self.legs[i.leg-1][a]['position'],
                                                                        self.legs[i.leg-1][a]['target'],
                                                                        self.legs[i.leg-1][a]['pwm'],
                                                                        self.legs[i.leg-1][a]['pwmtarget'],
                                                                        self.legs[i.leg-1][a]['status'],
                                                                        self.legs[i.leg-1][a]['time']))
                                p+=com_data.Infos.size                                
                            else:
                                #print("infos check wrong:",i)
                                p+=1
                        elif d[p]==0x53 and (len(d)-p)>=com_data.GlobalInfos.size:
                            i=com_data.GlobalInfos(d[p:])
                            if i.check():
                                #print("stats: main:",end='')
                                #print(i.main_ticking,end='')
                                #for l in range(4):
                                #    for a in range(3):
                                #        print('L',l+1,a+1,'[',i.actuator_ticking[l*3+a],',',i.read_ticking[l*3+a],']',end='')
                                #print()
                                p+=com_data.GlobalInfos.size
                            else:
                                #print("main stats check wrong:",i)
                                p+=1
                        else:
                            if self.print_str_msg:
                                if chr(d[p]).isprintable() or chr(d[p])=='\n':
                                    print(chr(d[p]),end='')
                            p+=1
            if action==False:
                time.sleep(0.001)
        if self.logfile!=None:
            self.logfile.close()
        print("controler thread leaving")
        


if __name__ == "__main__":
    try:
        import sys
        c=ControlerHandler(sys.argv[1])
        c.print_string_msg(True)
        time.sleep(0.2)
        c.send_info(0.0001)
        input("waiting...")
    except Exception as e:
        print("Exception:",e)
        pass
    c.stop()
    c.join()
#print("run controler handlers")
#controler=ControlerHandler()
#controler.start()
#print("done")
