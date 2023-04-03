import serial
import serial.tools.list_ports
from dearpygui import core, simple
from multiprocessing import Process, Queue, Value
import numpy as np
import os
import time
from struct import unpack
# def imudecode(self):
#         while(True):
#             if(self.qdata.empty()):
#                 break
#             tempdatastr=self.qdata.get()
#             tempdatalist=tempdatastr.split(' ')
#             if len(tempdatalist)<8:
#                 continue
#             core.log_info(tempdatalist)

#             self.imudata['dx'].append(int(tempdatalist[1])/self.BMA2x2_ACCELEROMETER_SENSITIVITY_AT_2G*self.g)
#             self.imudata['dy'].append(int(tempdatalist[2])/self.BMA2x2_ACCELEROMETER_SENSITIVITY_AT_2G*self.g)
#             self.imudata['dz'].append(int(tempdatalist[3])/self.BMA2x2_ACCELEROMETER_SENSITIVITY_AT_2G*self.g)
#             self.imudata['dr'].append(int(tempdatalist[4])/self.BMG160_GYRO_SENSITIVITY_AT_250_DEG_SEC)
#             self.imudata['dp'].append(int(tempdatalist[5])/self.BMG160_GYRO_SENSITIVITY_AT_250_DEG_SEC)
#             self.imudata['dw'].append(int(tempdatalist[6])/self.BMG160_GYRO_SENSITIVITY_AT_250_DEG_SEC)
#             self.imudata['roll_acc'].append(np.arctan2(self.imudata['dy'][-1],self.imudata['dz'][-1])*self.rad2ang)
#             self.imudata['pitch_acc'].append(np.arctan2(-self.imudata['dx'][-1],self.imudata['dz'][-1])*self.rad2ang)
#             self.imudata['yaw_acc'].append(np.arctan2(self.imudata['dx'][-1],np.sqrt(self.imudata['dz'][-1]**2+self.imudata['dy'][-1]**2))*self.rad2ang)
#             if(len(self.imudata['roll_gyro'])==0):
#                 self.imudata['roll_gyro'].append(self.imudata['roll_acc'][-1]+self.imudata['dr'][-1]*self.dt)
#                 self.imudata['pitch_gyro'].append(self.imudata['pitch_acc'][-1]+self.imudata['dp'][-1]*self.dt)
#                 self.imudata['yaw_gyro'].append(self.imudata['yaw_acc'][-1]+self.imudata['dw'][-1]*self.dt)
#                 self.imudata['roll_filter'].append((self.imudata['roll_acc'][-1]+self.imudata['dr'][-1]*self.dt)*self.filter_a \
#                                             +self.imudata['roll_acc'][-1]*self.filter_b)
#                 self.imudata['pitch_filter'].append((self.imudata['pitch_acc'][-1]+self.imudata['dp'][-1]*self.dt)*self.filter_a \
#                                             +self.imudata['pitch_acc'][-1]*self.filter_b)
#                 self.imudata['yaw_filter'].append((self.imudata['yaw_acc'][-1]+self.imudata['dw'][-1]*self.dt)*self.filter_a \
#                                             +self.imudata['yaw_acc'][-1]*self.filter_b)
#             else:
#                 self.imudata['roll_gyro'].append(self.imudata['roll_gyro'][-1]+self.imudata['dr'][-1]*self.dt)
#                 self.imudata['pitch_gyro'].append(self.imudata['pitch_gyro'][-1]+self.imudata['dp'][-1]*self.dt)
#                 self.imudata['yaw_gyro'].append(self.imudata['yaw_gyro'][-1]+self.imudata['dw'][-1]*self.dt)
#                 self.imudata['roll_filter'].append((self.imudata['roll_filter'][-1]+self.imudata['dr'][-1]*self.dt)*self.filter_a \
#                                             +self.imudata['roll_acc'][-1]*self.filter_b)
#                 self.imudata['pitch_filter'].append((self.imudata['pitch_filter'][-1]+self.imudata['dp'][-1]*self.dt)*self.filter_a \
#                                             +self.imudata['pitch_acc'][-1]*self.filter_b)
#                 self.imudata['yaw_filter'].append((self.imudata['yaw_filter'][-1]+self.imudata['dw'][-1]*self.dt)*self.filter_a \
#                                             +self.imudata['yaw_acc'][-1]*self.filter_b)
#             self.imudata['tem_temperature'].append(int(tempdatalist[7])* 0.001953125 + 23.0)
#             self.saveque.put([self.imudata['dx'][-1],self.imudata['dy'][-1],self.imudata['dz'][-1],self.imudata['dr'][-1],self.imudata['dp'][-1], \
#                 self.imudata['dw'][-1],self.imudata['tem_temperature'][-1]])


class App():
    def __init__(self):
        ports = serial.tools.list_ports.comports()
        # 'com_list' contains list of all com ports
        self.qdata = Queue(100)
        self.saveque = Queue(100)
        self.imudata = {}
        self.rad2ang = 180/np.pi
        self.BMA2x2_ACCELEROMETER_SENSITIVITY_AT_2G = 1024
        self.BMG160_GYRO_SENSITIVITY_AT_250_DEG_SEC = 131.2
        self.g = 9.8
        self.dt = 1/100
        self.filter_a = 0.98
        self.filter_b = 1-self.filter_a
        self.com_list = []
        self.stop_flag = Value('i', 1)
        for p in ports:
            self.com_list.append(p.device)

        core.log_info(self.com_list)

    def multpro_serial_getdata(self, com, que):
        core.log_info(f'sub pid is {os.getpid()}')
        ser = serial.Serial(com, 115200, timeout=100)
        ser.flushInput()
        ser.flushOutput()
        while(~self.stop_flag.value):
            line = ser.readline().decode()
            que.put(line[:-1])
        ser.close()

    def multpro_serial_getdata_hex(self, com, que):
        core.log_info(f'sub pid is {os.getpid()}')
        ser = serial.Serial(com, 115200, timeout=100)
        ser.flushInput()
        ser.flushOutput()
        tempreciv = []
        while(~self.stop_flag.value):
            tempreciv.append(ser.read(1))
            if len(tempreciv) >= 2 and tempreciv[-1].hex() == '5a' and tempreciv[-2].hex() == '5b':
                tempdata = ser.read(16)
                putquedata =unpack('<q2f', tempdata)
                putquedata=[str(i) for i in putquedata]
                putquedata=' '.join(putquedata)
                que.put(putquedata)
                tempreciv = []
        ser.close()

    def save_imudata(self, que):
        name = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())+'.csv'
        with open(name, 'w+') as fid:
            while(True):
                if(que.empty()):
                    continue
                tempdata = que.get()
                for i in tempdata:
                    fid.write('%f' % i)
                    fid.write(' ')
                fid.write('\n')

    def start_get_data(self, sender, data):
        self.stop_flag.value = 0
        self.column = core.get_value('data num is')
        for i in range(self.column):
            self.imudata[str(i)] = []
        for i in range(1, self.column):
            core.add_plot(f"{i}", height=200, parent="main")
        core.log_info(f'main pid is {os.getpid()}')
        argss = (core.get_value("com"), self.qdata)
        p1 = Process(target=self.multpro_serial_getdata_hex, args=argss)
        p2 = Process(target=self.save_imudata, args=(self.saveque,))
        # p3=Process(target=imudecode,args=(self,))
        core.set_render_callback(self.plotdata)
        p1.daemon = True
        p2.daemon = True
        # p3.daemon=True

        p2.start()
        p1.start()
        # p3.start()

    def update_com(self, sender, data):
        ports = serial.tools.list_ports.comports()
        self.com_list = []
        for p in ports:
            self.com_list.append(p.device)
        itemvalue=core.get_value("com")
        core.delete_item("com")
        core.add_combo("com", items=self.com_list, before='start',default_value=itemvalue)

    def plotdata(self, sender, data):
        # core.log_info(f'come in plotdata q is {self.qdata.empty()}')
        while(True):
            if(self.qdata.empty()):
                break
            tempdatastr = self.qdata.get()
            core.log_info(f'tempdatastr is {tempdatastr}')
            tempdatalist = tempdatastr.split(' ')
            if len(tempdatalist) < self.column:
                continue

            tempdata = []
            for i in range(self.column):
                try:
                    tempdata.append(float(tempdatalist[i]))
                except:
                    core.log_error(f'data error={tempdatalist[i]}')
                    tempdata = []
                    break
            if len(tempdata) == self.column:
                for i in range(self.column):
                    self.imudata[str(i)].append(tempdata[i])
                self.saveque.put(tempdata)
        for i in range(self.column):
            core.clear_plot(str(i))

            plotdata = self.imudata[str(i)][max(
                0, len(self.imudata[str(i)])-2000):]
            core.add_line_series(f'{i}', f'{i}_1', x=list(range(
                max(0, len(self.imudata[str(i)])-2000), len(self.imudata[str(i)]))), y=plotdata)

    def stop(self, sender, data):
        self.stop_flag.value = 1
        core.set_render_callback(self.idel)

    def idel(self, sender, data):
        pass

    def gui(self):
        with simple.window("main"):

            core.show_logger()

            core.add_combo("com", items=self.com_list,)
            core.add_button('start', callback=self.start_get_data)
            core.add_same_line()
            core.add_button('stop', callback=self.stop)
            core.add_same_line()
            core.add_button('restart', callback=self.start_get_data)
            core.add_same_line()
            core.add_input_int('data num is', default_value=3)
            core.add_plot("0", height=200)

        core.set_render_callback(self.update_com)
        core.start_dearpygui(primary_window="main")


if __name__ == '__main__':
    app = App()
    app.gui()
