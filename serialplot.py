from turtle import hideturtle
import dearpygui.dearpygui as dpg
import dearpygui.demo as demo
import serial.tools.list_ports
import serial
from multiprocessing import Process, Lock, Manager, Queue, Value
from multiprocessing.pool import Pool
import time
import os
import struct
import copy

label_name_en = {
    "title": "serial plot",
    "open serial": "open serial",
    "open file": "open file",
    "close serial": "close serial",
    "input send text": "input send text",
    "start ota": "Start OTA",
    "log out": "log out",
    "send cmd": "send cmd",
    "Plot": "plot",
    "line num": "line num",
    "set num": "set num"
}
dpg.create_context()
print(label_name_en["close serial"])


class SerialPlotData:
    portlist = []
    ser = None
    label_name = label_name_en
    update_serial_port_flag = 1
    recv_que = Queue(100)
    send_que = Queue(5)
    serial_handle = None
    serial_send_flag = Value('I', 0)
    serial_recv_flag = Value('I', 1)
    serial_enalbe = Value('I', 1)
    plot_num = Value('I', 1)
    output_log = ''
    count = 0
    bin_file_data = Manager().dict()

    def __init__(self) -> None:
        # self.mp=threading.Thread(target=self.UpdateSerialPortlist)
        # self.mp.start()
        pass

    def UpdateSerialPortlist(self):
        self.portlist = []
        ports = serial.tools.list_ports.comports()
        for port in sorted(ports):
            self.portlist.append(port.device)
        dpg.configure_item("serial_port_list", items=self.portlist)
        # time.sleep(0.5)

    def OpenFileCallback(self, sender, app_data):
        print("Sender: ", sender)
        print("App Data: ", app_data)
        filename = app_data['file_path_name']
        self.bin_file_data["bin_size"] = os.path.getsize(filename)
        with open(filename, 'rb') as fid:
            self.bin_file_data["bin_data"] = [
                int(i, 16) for i in fid.read(
                    self.bin_file_data["bin_size"]).hex(' ').split(' ')
            ]
        file_info = f"file path:\r\n{filename}\r\nfile size:{self.bin_file_data['bin_size']}"
        dpg.set_value("bin_file_path", file_info)
        with dpg.window(tag="process_bar_win", width=400, pos=[100, 200]):
            dpg.add_text(
                default_value="calculate crc of bin file,please wait...")
            dpg.add_progress_bar(tag="process_bar", default_value=0.0)
        time.sleep(0.5)
        self.bin_file_data['bin_crc'] = self.CRC16_CCITT(
            self.bin_file_data['bin_data'], self.bin_file_data['bin_size'],
            self.CRC16_CCITT_FFFF)
        time.sleep(0.5)
        dpg.delete_item("process_bar_win")
        self.output_log += (
            f"bin file length is: {self.bin_file_data['bin_size']} crc is :" +
            str(self.bin_file_data["bin_crc"]) + "\r\n")
        dpg.configure_item("text_tag_log_out", default_value=self.output_log)

    def DecodeDataHex(self, com, que, data_format):
        print(f'sub pid is {os.getpid()}')
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

    def SerialReadCallback(self, ser):
        ser.reset_input_buffer()
        while self.serial_enalbe:
            if self.serial_recv_flag.value:
                if ser.in_waiting:
                    str = ser.read(ser.in_waiting)
                    # str=ser.readline()
                    if not self.recv_que.full():
                        self.recv_que.put([0, str])
                    else:
                        print("que full")

    def SerialSendCallback(self, ser):
        while 1:
            if self.send_que.empty():
                self.serial_recv_flag.value = 1
                continue
            send_data = self.send_que.get()
            if self.ota_mode.value == 0:
                ser.write(send_data[-1].encode())
                self.recv_que.put([1, send_data[-1]])
            else:
                ser.write(send_data[-1])
                self.recv_que.put([2, send_data])

    def OpenSerialPort(self, sender, app_data):
        self.ser = serial.Serial(port=dpg.get_value("serial_port_list"),
                                 baudrate=115200)
        dpg.set_item_label("serial_button", self.label_name["close serial"])
        dpg.set_item_callback("serial_button", self.CloseSerialPort)

        self.serial_recv_flag.value = 1
        self.serial_enalbe.value = 1
        self.serial_recv_handle = Process(target=self.SerialReadCallback,
                                          args=(self.ser, ))
        self.serial_recv_handle.daemon
        self.serial_recv_handle.start()

        self.serial_send_handle = Process(target=self.SerialSendCallback,
                                          args=(self.ser, ))
        self.serial_send_handle.daemon
        self.serial_send_handle.start()
        print(dpg.get_value("serial_port_list"))

    def UpdateSerialOutPutLog(self):
        if not self.recv_que.empty():
            log_info = self.recv_que.get()
            self.output_log += ("\n<<- " + log_info[1])
            dpg.configure_item("text_tag_log_out",
                               default_value=self.output_log)

    def CloseSerialPort(self, sender, app_data):
        if self.ser != None:
            # self.serial_handle.terminate()
            dpg.set_item_callback("serial_button", self.OpenSerialPort)
            dpg.set_item_label("serial_button", self.label_name["open serial"])
            self.serial_recv_flag.value = 0
            self.serial_enalbe.value = 0
            self.serial_handle.join()
            self.serial_handle.close()
            self.ser.close()
            self.ser = None
            self.output_log = ''

    def CRC16_CCITT(self, data, len, crc_init):
        crc = copy.deepcopy(crc_init)
        for i in range(len):
            if i % 100 == 0:
                dpg.configure_item("process_bar", default_value=i / len)
                time.sleep(0.001)
            x = (crc >> 8) ^ data[i]
            x ^= (x >> 4)
            crc = ((crc << 8) & 0xFFFF) ^ ((x << 12) & 0xFFFF) ^ (
                (x << 5) & 0xFFFF) ^ x
        crc &= 0xFFFF
        return crc

    def SendCmd(self, sender, app_data):
        cmdstr = dpg.get_value("send_text")
        # cmdhex=struct.pack()
        # print(cmd)
        self.send_que.put(cmdstr)
        self.serial_recv_flag.value = 0

    def CloseGui(self, sender, app_data):
        self.CloseSerialPort(sender, app_data)

    def ClearOutput(self, sender, app_data):
        self.output_log = ""

    def Gui(self):
        with dpg.file_dialog(directory_selector=False,
                             show=False,
                             width=600,
                             height=600,
                             callback=self.OpenFileCallback,
                             id="file_dialog_id"):
            dpg.add_file_extension("bin file (*.bin *.hex){.bin,.hex}")
            dpg.add_file_extension(".bin",
                                   color=(255, 0, 255, 255),
                                   custom_text="[binary]")
            dpg.add_file_extension("", color=(150, 255, 150, 255))
        with dpg.window(tag="Primary Window", on_close=self.CloseGui):
            with dpg.menu_bar():
                with dpg.menu(label="Tools"):
                    dpg.add_menu_item(
                        label="Font Manager",
                        callback=lambda: dpg.show_tool(dpg.mvTool_Font))
            with dpg.child_window(width=600, height=100):
                with dpg.group(horizontal=True):
                    dpg.add_combo(items=self.portlist,
                                  tag="serial_port_list",
                                  width=200,
                                  drop_callback=self.UpdateSerialPortlist)
                    dpg.add_button(label=self.label_name["open serial"],
                                   callback=self.OpenSerialPort,
                                   tag="serial_button")
                with dpg.group(horizontal=True):
                    # dpg.add_button(label=self.label_name["open file"],width=100,height=20,callback=lambda:dpg.show_item("file_dialog_id"))
                    dpg.add_input_int(tag="set_line_num", width=100)
                    dpg.add_button(label=self.label_name["set num"])

                dpg.add_text(tag="bin_file_path",
                             default_value="",
                             color=(0, 255, 0, 255),
                             wrap=500)

            with dpg.child_window(width=1600, height=400):
                dpg.add_plot(label="plot", width=1600, tag="data_plot")
            demo.show_demo()

        dpg.create_viewport(title=self.label_name["title"],
                            width=1200,
                            height=800)
        dpg.setup_dearpygui()
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        # dpg.start_dearpygui()
        while dpg.is_dearpygui_running():
            if self.update_serial_port_flag == 1:
                self.UpdateSerialPortlist()
            self.UpdateSerialOutPutLog()
            dpg.render_dearpygui_frame()
        dpg.destroy_context()


if __name__ == "__main__":
    vdot = SerialPlotData()
    vdot.Gui()