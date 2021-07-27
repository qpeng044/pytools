from io import open_code
import serial
import serial.tools.list_ports
from dearpygui import core, simple
from multiprocessing import Process, Queue, Value
import numpy as np
import os
import time
from struct import unpack
import sys
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
        self.plotque = Queue(100)
        self.saveque = Queue(100)
        self.rad2ang = 180/np.pi
        self.com_list = []
        self.yaw = [0]
        self.plotpoint = 1000
        self.data_format = []
        self.frame_head = ['5a5b']
        self.stop_flag = Value('i', 1)
        self.posetion = []
        for p in ports:
            self.com_list.append(p.device)

        core.log_info(self.com_list)

    def MultProSerialGetDataHex(self, com, que, data_format):
        print(f'sub pid is {os.getpid()}')
        ser = serial.Serial(com, 115200, timeout=100)
        ser.flushInput()
        ser.flushOutput()
        tempreciv = []
        factory_mode = 0
        while(~self.stop_flag.value):
            tempreciv.append(ser.read(1))
            if len(tempreciv) >= 2 and tempreciv[-1].hex() == '5a' and tempreciv[-2].hex() == '5b':
                try:
                    if factory_mode == 1:
                        ser.read(1)
                        typenume = unpack('B', ser.read(1))[0]
                        ser.read(1)
                        numfloat = int((typenume)/4)
                        tempdata = []
                        for i in range(numfloat):
                            tempdata.append(unpack('<f', ser.read(4))[0])
                            ser.read(1)
                        putquedata = tempdata
                    else:
                        typenume = unpack('B', ser.read(1))[0]
                        numfloat = int((typenume)/4)
                        tempdata = ser.read(typenume)
                        #tempdata = ser.read(16)
                        putquedata = unpack(f'<{numfloat}f', tempdata)
                        # putquedata = unpack(data_format[0], tempdata)
                    # print(putquedata)
                    putquedata = [str(i) for i in putquedata]
                    putquedata = ' '.join(putquedata)
                    que.put(['5a5b', putquedata])
                    # print('1 put data')
                    tempreciv = []
                except:
                    tempreciv = []
            if len(tempreciv) >= 2 and tempreciv[-1].hex() == '5c' and tempreciv[-2].hex() == '5d':
                typenume = int(ser.read(1).hex())
                tempdata = ser.read(20)
                putquedata = unpack('<q3f', tempdata)
                putquedata = [str(i) for i in putquedata]
                putquedata = ' '.join(putquedata)
                que.put(['5c5d', putquedata])
                # print('2 put data')
                tempreciv = []
        ser.close()

    def SaveImuData(self, que):
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

    def StartGetData(self, sender, data):
        self.stop_flag.value = 0
        self.column = core.get_value('data num is')
        core.log_info(f'main pid is {os.getpid()}')
        # if len(self.data_format) == 0:
        #     core.log_error("There is no data format")
        #     return
        argss = (core.get_value("com"), self.qdata, self.data_format)
        if '5c5d' in self.frame_head:
            with simple.window("rotbot posetion window", on_close=self.CloseWindows):
                core.add_plot('robot posetion plot')
        p1 = Process(target=self.MultProSerialGetDataHex, args=argss)
        p2 = Process(target=self.SaveImuData, args=(self.saveque,))
        p3 = Process(target=self.Preparedata, args=(self.qdata, self.plotque,))
        # p3=Process(target=imudecode,args=(self,))
        core.set_render_callback(self.PlotData)
        p1.daemon = True
        p2.daemon = True
        p3.daemon = True
        # p3.daemon=True

        p2.start()
        p1.start()
        p3.start()
        # p3.start()

    def UpdateCom(self, sender, data):
        ports = serial.tools.list_ports.comports()

        templist = []
        for p in ports:
            templist.append(p.device)
        if templist == self.com_list:
            return
        # self.com_list = []
        # for p in ports:
        #     self.com_list.append(p.device)
        core.log_info('com is changed.')
        itemvalue = core.get_value("com")
        self.com_list = templist
        core.delete_item("com")
        core.add_combo("com", items=self.com_list,
                       before='start', default_value=itemvalue)

    def Preparedata(self, getque, putque):
        while(~self.stop_flag.value):
            if(getque.empty()):
                continue
            # print(f'getque is {getque.empty()}')
            getquedata = getque.get()
            tempdatastr = getquedata[1]
            # print(f'tempdatastr is {tempdatastr}')
            tempdatalist = tempdatastr.split(' ')
            tempdata = []
            column = len(tempdatalist)
            for i in range(column):
                try:
                    tempdata.append(float(tempdatalist[i]))
                except:
                    print(f'data error={tempdatalist[i]}')
                    tempdata = []
                    break
            putque.put([getquedata[0], column, tempdata])

            # print('put data to queen')
            # if len(tempdata) == column:
            #     for i in range(column):
            #         self.imudata[str(i)].append(tempdata[i])

    def PlotData(self, sender, data):
        core.log_info(f'in {sys._getframe().f_code.co_name}')
        while(~self.stop_flag.value):
            if(self.plotque.empty()):
                break
            # core.log_info('get plot data')
            getquedata = self.plotque.get()
            core.log_info(f'get plot data {getquedata}')
            datatype = getquedata[0]  # type of data
            column = getquedata[1]  # data nums
            tempdata = getquedata[2]  # data
            if datatype == '5a5b':
                if not hasattr(self, 'imudata'):
                    self.imudata = {}
                    for i in range(column):
                        self.imudata[str(i)] = []
                    if column < 5:
                        for i in range(column):
                            core.add_plot(f"{i}", height=200, parent="main")
                    else:
                        for i in range(column):
                            core.add_plot(f"{i}", height=200,
                                          width=800, parent="main")
                            if i % 2 == 0:
                                core.add_same_line(parent='main')

                for i in range(column):
                    self.imudata[str(i)].append(tempdata[i])
                for i in range(column):
                    core.clear_plot(str(i))
                    plotdata = self.imudata[str(i)][max(
                        0, len(self.imudata[str(i)])-self.plotpoint):]
                    core.add_line_series(f'{i}', f'{i}_1', x=list(range(
                        max(0, len(self.imudata[str(i)])-self.plotpoint), len(self.imudata[str(i)]))), y=plotdata)
            else:
                if not hasattr(self, 'rawyaw'):
                    self.rawyaw = []
                    self.wheelx = []
                    self.wheely = []
                self.rawyaw.append(tempdata[1])
                self.wheelx.append(tempdata[2])
                self.wheely.append(tempdata[3])
                core.clear_plot('robot posetion plot')
                core.add_line_series(
                    'robot posetion plot', 'robot posetion lines', x=self.wheelx, y=self.wheely)

            if(0):  # plot yaw calculate by dt and dyaw
                if datatype == '5a5b':
                    # tempyaw = self.yaw[-1]+self.imudata["1"][-1] * \
                    #     self.imudata["2"][-1]*self.rad2ang
                    # if tempyaw != 0:
                    #     tempyaw = (np.abs(tempyaw) % 180) * \
                    #         tempyaw/np.abs(tempyaw)
                    # else:
                    #     tempyaw = 0
                    tempyaw = tempdata[-1]
                    self.yaw.append(tempyaw)
                    tempdata.append(float(tempyaw))

                    plotdata = self.yaw[max(
                        0, len(self.yaw)-self.plotpoint):]
                    core.clear_plot('yaw')
                    core.add_line_series('yaw', 'yawline', x=list(range(
                        max(0, len(self.yaw)-self.plotpoint), len(self.yaw))), y=plotdata)
                # if datatype == '5c5d':
                    plotdata = self.rawyaw[max(
                        0, len(self.rawyaw)-self.plotpoint):]
                    core.add_line_series('yaw', 'rawyawline', x=list(range(
                        max(0, len(self.rawyaw)-self.plotpoint), len(self.rawyaw))), y=plotdata)
            if datatype == '5a5b':
                self.saveque.put(tempdata)

    def Stop(self, sender, data):
        self.stop_flag.value = 1
        core.set_render_callback(self.IdelRun)

    def IdelRun(self, sender, data):
        pass

    def AddDataFormat(self, sender, data):

        if sender == 'add data format':
            items = self.data_format
        elif sender == 'add frame head':
            items = self.frame_head
        with simple.window(f"{sender} window", on_close=self.CloseWindows):
            core.add_listbox(f'exist value##{sender}', items=items)
            core.add_input_text(f'input value##{sender}')
            core.add_button(f'ok##{sender}',
                            callback=self.AddDataFormatOk, callback_data=[sender, items])

    def CloseWindows(self, sender, data):
        core.delete_item(sender)

    def AddDataFormatOk(self, sender, data):
        input_value = core.get_value(f'input value##{data[0]}')
        for i in range(len(data[1])):
            if data[1][i] == input_value:
                data[1].pop(i)
                core.delete_item(f'exist value##{data[0]}')
                core.add_listbox(f'exist value##{data[0]}',
                                 items=data[1], before=f'input value##{data[0]}')
                return
        data[1].append(input_value)
        core.delete_item(f'exist value##{data[0]}')
        core.add_listbox(f'exist value##{data[0]}',
                         items=data[1], before=f'input value##{data[0]}')

    def ShowLog(self, sender, data):
        core.show_logger()

    def Gui(self):
        with simple.window("main"):

            core.show_logger()

            core.add_combo("com", items=self.com_list,)
            core.add_button('start', callback=self.StartGetData)
            core.add_same_line()
            core.add_button('Stop', callback=self.Stop)
            core.add_same_line()
            core.add_button('restart', callback=self.StartGetData)
            core.add_same_line()
            core.add_button('add data format', callback=self.AddDataFormat)
            core.add_same_line()
            core.add_button('add frame head', callback=self.AddDataFormat)
            core.add_same_line()
            core.add_button("show log", callback=self.ShowLog)
            # core.add_input_int('data num is', default_value=3)
            # core.add_plot("0", height=200)
            # core.add_plot("yaw", height=200)

        core.set_render_callback(self.UpdateCom)
        core.start_dearpygui(primary_window="main")


if __name__ == '__main__':
    app = App()
    app.Gui()
