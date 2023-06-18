import dearpygui.dearpygui as dpg
from multiprocessing import Process, Queue, Lock, Value, Manager
import time
import sched
import numpy as np
import pandas as ps
import threading
import copy
gui_label_ch = ["开始", "添加路径", "清除路径", "设置速度", "结束", "关闭多图模式", "日志"]
gui_label_en = ["Start", "AddPath", "DeletePath",
                "SetSpeed", "Stop", "CloseMultiPlot", "Logger"]

gui_label_tag = ["start", "add_path", "delete_path",
                 "set_speed", "stop", "close_multplot", "logger"]


motion_start_time = time.time()

sensor_timer_event = None
current_pose_index = 0
robot_max_acc = 0.5
robot_max_speed = 0.5
robot_max_yaw_rate = 0.5
robot_max_yaw_acc = 0.5
lock = threading.RLock()


distance_two_wheel = 0.1
is_start_simulation = Value('i', 0)
generation_queue = Queue(10)
app_queue = Queue(1)


def GenerationData(is_start_simulation, generation_queue, app_queue):
    counter = 0
    key_press_status = {"a": 0, "w": 0, "s": 0, "d": 0}
    control_robot_position_data = {"px": 0, "py": 0, "vr":
                                   0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    control_robot_position_data_last = {"px": 0, "py": 0, "vr":
                                        0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    print(is_start_simulation.value)
    while(is_start_simulation.value):
        if(not generation_queue.empty()):
            key_press_status = generation_queue.get()
        # time.sleep(0.001)
        control_robot_position_data["t"] = (time.time())
        dt = control_robot_position_data["t"] - \
            control_robot_position_data_last["t"]
        if(key_press_status["w"] == 1):
            dv = robot_max_acc*dt
            if(control_robot_position_data_last["v"] >= robot_max_speed):
                control_robot_position_data["v"] = (
                    control_robot_position_data_last["v"])
            else:
                control_robot_position_data["v"] = (
                    control_robot_position_data_last["v"]+dv)
        else:
            dv = -robot_max_acc*dt
            if(control_robot_position_data_last["v"] <= 0):
                control_robot_position_data["v"] = (0)
            else:
                control_robot_position_data["v"] = (
                    control_robot_position_data_last["v"]+dv)
                ###############theta#######################
        if(key_press_status["d"] == 1 and key_press_status["a"] == 1):
            dtheta = 0
        elif(key_press_status["a"] == 1):
            dtheta = robot_max_yaw_acc*dt
        elif key_press_status["d"] == 1:
            dtheta = -robot_max_yaw_acc*dt
        else:
            dtheta = 0
        control_robot_position_data["theta"] = (
            control_robot_position_data_last["theta"]+dtheta)

    # #######################px,py###################################
        vx = control_robot_position_data["v"] * \
            np.cos(control_robot_position_data["theta"])
        vy = control_robot_position_data["v"] * \
            np.sin(control_robot_position_data["theta"])
        control_robot_position_data["px"] = (
            control_robot_position_data_last["px"]+vx*dt)
        control_robot_position_data["py"] = (
            control_robot_position_data_last["py"]+vy*dt)
        # print(f"end time:{time.time()}")
        # if(counter % 100 == 0):
        #     counter = 0
        #     # print(dv, control_robot_position_data["v"][-1],
        #     #       dtheta, control_robot_position_data["theta"][-1])
        #     print(control_robot_position_data["px"][-1],
        #           control_robot_position_data["py"][-1])
        # print(f'a:{key_press_status["a"]}')
        if not app_queue.full():
            app_queue.put(control_robot_position_data)
        control_robot_position_data_last = copy.deepcopy(
            control_robot_position_data)


raw_robot_data = {"px": [0], "py": [0], "vr": [
    0], "vl": [0], "v": [0], "theta": [0], "t": [0]}


class App:
    is_init_motion_handle = 0
    frame_counter = 0
    key_press_status = {"a": 0, "w": 0, "s": 0, "d": 0}

    def __init__(self) -> None:
        word_simulink = WorldCoordinate()
        self.robot = Robot()
        self.Gui()

    def Gui(self):
        global raw_robot_data
        dpg.create_context()
        dpg.create_viewport(title='数据融合', width=600, height=400)
        dpg.setup_dearpygui()
        with dpg.font_registry():
            with dpg.font(r"ZiTiGuanJiaFangSongTi-2.ttf", 15) as default_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Chinese_Full)
        dpg.bind_font(default_font)  # Binds the font globally
        with dpg.window(tag="Primary Window", on_close=self.CloseCallback):
            global head_direction_data, head_speed_data, control_time, control_robot_position_data_key
            with dpg.group(horizontal=True):
                dpg.add_button(label=gui_label_ch[0],
                               tag=gui_label_tag[0], callback=self.StartSimulation)
                dpg.add_button(label=gui_label_ch[4],
                               tag=gui_label_tag[4], callback=self.StopSimulation)
            with dpg.plot(tag="simulink_plot",
                          width=dpg.get_viewport_width()-10, height=dpg.get_viewport_height()-dpg.get_item_pos(gui_label_tag[0])[1]-50):
                dpg.add_plot_legend()

                # REQUIRED: create x and y axes
                dpg.add_plot_axis(dpg.mvXAxis, label="x",
                                  tag=f"simulink_plot_x_axis")
                dpg.add_plot_axis(dpg.mvYAxis, label="y",
                                  tag=f"simulink_plot_y_axis")
                dpg.add_line_series(
                    x=raw_robot_data["px"], y=raw_robot_data["py"], tag="raw_path", parent=f"simulink_plot_y_axis")
        dpg.set_viewport_resize_callback(self.viewport_resize_callback)
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        # dpg.start_dearpygui()
        while dpg.is_dearpygui_running():
            # insert here any code you would like to run in the render loop
            # you can manually stop by using stop_dearpygui()
            self.FrameCallback()
            dpg.render_dearpygui_frame()
        dpg.destroy_context()

    def FrameCallback(self):
        global is_start_simulation, raw_robot_data
        self.frame_counter += 1
        if(is_start_simulation.value == 0):
            return
        if dpg.is_key_down(key=dpg.mvKey_W):
            self.key_press_status["w"] = 1
        else:
            self.key_press_status["w"] = 0
        if dpg.is_key_down(key=dpg.mvKey_A):
            self.key_press_status["a"] = 1
        else:
            self.key_press_status["a"] = 0
        if dpg.is_key_down(key=dpg.mvKey_S):
            self.key_press_status["s"] = 1
        else:
            self.key_press_status["s"] = 0
        if dpg.is_key_down(key=dpg.mvKey_D):
            self.key_press_status["d"] = 1
        else:
            self.key_press_status["d"] = 0
        generation_queue.put(self.key_press_status)
        # print(
        #     f'a:{self.key_press_status["a"]},w:{self.key_press_status["w"]},d:{self.key_press_status["d"]},s:{self.key_press_status["s"]}')
        if(self.frame_counter % 10 == 0):
            self.frame_counter = 0
            pad = 2
            if(not app_queue.empty()):
                temp_data = app_queue.get()
                # print(f"get pose data{temp_data}")
                lock.acquire()
                for key in raw_robot_data.keys():
                    raw_robot_data[key].append(
                        temp_data[key])
                lock.release()
            dpg.set_value("raw_path",
                          [raw_robot_data["px"], raw_robot_data["py"]])
            dpg.set_axis_limits("simulink_plot_x_axis", min(
                raw_robot_data["px"])-pad, max(raw_robot_data["px"])+pad)
            dpg.set_axis_limits("simulink_plot_y_axis", min(
                raw_robot_data["py"])-pad, max(raw_robot_data["py"])+pad)

    def StartSimulation(self, sender, appdata):
        global is_start_simulation, generation_queue, app_queue
        print("start sched")
        is_start_simulation.value = 1
        self.process = Process(target=GenerationData,
                               args=(is_start_simulation, generation_queue, app_queue,))
        self.process.start()
        self.robot.StartRun()

    def StopSimulation(self, sender, appdata):
        global is_start_simulation
        print("stop sched")
        is_start_simulation.value = 0
        self.process.join()
        self.process.close()
        self.robot.StopRun()
        dpg.set_axis_limits_auto("simulink_plot_x_axis")
        dpg.set_axis_limits_auto("simulink_plot_y_axis")

    def viewport_resize_callback(self, sender, appdata):
        dpg.set_item_width("simulink_plot", dpg.get_viewport_width()-10)
        dpg.set_item_height("simulink_plot", dpg.get_viewport_height(
        )-dpg.get_item_pos(gui_label_tag[0])[1]-50)

    def CloseCallback(self, sender, appdata):
        pass

    def RobotMotionResizeCallback(self, send, appdata):
        dpg.set_item_width("motion_map", dpg.get_item_width("robot_motion")-10)


class WorldCoordinate:
    def __init__(self) -> None:
        pass

    def InitRobotMotion(self):
        pass

    def run(self):
        pass


algo_res_queue = Queue(10)
sensor_timer = sched.scheduler(time.time, time.sleep)
stop_sensor_timer = Value('i', 0)


class Robot:
    robot_run_start = Value("i", 0)
    robot_sensor_queue = Queue(10)

    def __init__(self) -> None:
        self.imu = ImuSensor(self.robot_sensor_queue)
        self.wheel = WheelEncoder(self.robot_sensor_queue)
        self.optical = OpticalFlow(self.robot_sensor_queue)

        print("init robot")

    def StartRun(self):
        global stop_sensor_timer
        self.robot_run_start.value = 1
        self.process = Process(target=self.RobotAlgo,
                               args=(self.robot_run_start, self.robot_sensor_queue, algo_res_queue,))
        self.process.start()
        sensor_timer.run()

    def StopRun(self):
        print("stop robot")
        global stop_sensor_timer
        self.robot_run_start.value = 0
        stop_sensor_timer.value = 1
        sensor_timer.cancel(imu_timer)
        sensor_timer.cancel(encoder_timer)
        sensor_timer.cancel(optical_timer)
        self.process.join()
        self.process.close()

    def RobotAlgo(self, status, recv_queue, send_queue):

        while(status.value):
            if(recv_queue.empty()):
                time.sleep(0.001)
                continue
            sensor_data = recv_queue.get()
            if(sensor_data[0] == "imu"):
                print("imu")
            elif(sensor_data[0] == "encoder"):
                print("encoder")
            elif(sensor_data[0] == "optical"):
                print("optical")


class ImuSensor:
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    imu_nosie = {"ax_n": [0, 0.1], "ay_n": [0, 0.1], "az_n": [0, 0.1], "wx_n": [
        0, 0.01], "wy_n": [0, 0.01], "wz_n": [0, 0.01]}  # [mean,var]
    noise_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                      "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]

    def __init__(self, q_msg) -> None:
        global sensor_timer, imu_timer
        imu_timer = sensor_timer.enter(0.010, 2, self.generate_data)
        self.robot_queue = q_msg

    def generate_data(self):
        global raw_robot_data, stop_sensor_timer, imu_timer
        if(stop_sensor_timer.value):
            return
        imu_timer = sensor_timer.enter(0.010, 2, self.generate_data)
        lock.acquire()
        for key in raw_robot_data.keys():
            self.current_raw_robot_data[key] = raw_robot_data[key][-1]
        lock.release()
        if(self.last_raw_robot_data["t"] == 0):
            self.last_raw_robot_data = copy.deepcopy(
                self.current_raw_robot_data)
            return
        if(self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"] == 0):
            return
        #####
        print(self.current_raw_robot_data)
        print(self.last_raw_robot_data)
        self.real_imu_data["ax"] = (self.current_raw_robot_data["v"]-self.last_raw_robot_data["v"]) / (
            self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"])
        self.real_imu_data["wx"] = (self.current_raw_robot_data["theta"]-self.last_raw_robot_data["theta"]) / (
            self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"])
        self.noise_imu_data["ax"] = self.real_imu_data["ax"] + \
            np.random.normal(
                loc=self.imu_nosie["ax_n"][0], scale=self.imu_nosie["ax_n"][1], size=None)
        self.noise_imu_data["wx"] = self.real_imu_data["wx"] + \
            np.random.normal(
                loc=self.imu_nosie["wx_n"][0], scale=self.imu_nosie["wx_n"][1], size=None)

        ######
        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(["imu", self.real_imu_data, self.noise_imu_data])


class WheelEncoder:
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_wheel_data = {"vr": 0, "vl": 0}
    noise_wheel_data = {"vr": 0, "vl": 0}
    distance_to_center = 0.1  # 10cm
    encoder_nosie = {"vr": [0, 0.01], "vl": [0, 0.01]}  # [mean,var]

    def __init__(self, q_msg) -> None:
        global sensor_timer, encoder_timer
        encoder_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        self.robot_queue = q_msg

    def generate_data(self):
        global raw_robot_data, stop_sensor_timer, encoder_timer
        if(stop_sensor_timer.value):
            return
        encoder_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        lock.acquire()
        for key in raw_robot_data.keys():
            self.current_raw_robot_data[key] = raw_robot_data[key][-1]
        lock.release()
        if(self.last_raw_robot_data["t"] == 0):
            self.last_raw_robot_data = copy.deepcopy(
                self.current_raw_robot_data)
            return
        if(self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"] == 0):
            return
        dtheta = self.current_raw_robot_data["theta"] - \
            self.last_raw_robot_data["theta"]
        ddistance = np.sqrt(np.power(self.current_raw_robot_data["px"]-self.last_raw_robot_data["px"], 2)+np.power(
            self.current_raw_robot_data["py"]-self.last_raw_robot_data["py"], 2))
        R = np.sin(dtheta/2)*ddistance/2
        dt = self.current_raw_robot_data["t"] - \
            self.last_raw_robot_data["t"]
        self.real_wheel_data["vr"] = (R-self.distance_to_center)/dt
        self.real_wheel_data["vl"] = (R+self.distance_to_center)/dt

        self.noise_wheel_data["vr"] = self.real_wheel_data["vr"] + \
            np.random.normal(
                self.encoder_nosie["vr"][0], self.encoder_nosie["vr"][1])
        self.noise_wheel_data["vl"] = self.real_wheel_data["vl"] + \
            np.random.normal(
                self.encoder_nosie["vl"][0], self.encoder_nosie["vl"][1])

        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(
            ["encoder", self.real_wheel_data, self.noise_wheel_data])


class OpticalFlow:
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_optical_data = {"dx": 0, "dy": 0}
    noise_optical_data = {"dx": 0, "dy": 0}
    optcal_nosie = {"dx": [0, 0.5], "dy": [0, 0.5]}  # [mean,var]

    def __init__(self, q_msg) -> None:
        global sensor_timer, optical_timer
        optical_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        self.robot_queue = q_msg

    def generate_data(self):
        global raw_robot_data, stop_sensor_timer, optical_timer
        if(stop_sensor_timer.value):
            return
        optical_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        lock.acquire()
        for key in raw_robot_data.keys():
            self.current_raw_robot_data[key] = raw_robot_data[key][-1]
        lock.release()
        if(self.last_raw_robot_data["t"] == 0):
            self.last_raw_robot_data = copy.deepcopy(
                self.current_raw_robot_data)
            return
        if(self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"] == 0):
            return
        ####
        dtheta = self.current_raw_robot_data["theta"] - \
            self.last_raw_robot_data["theta"]
        ddistance = np.sqrt(np.power(self.current_raw_robot_data["px"]-self.last_raw_robot_data["px"], 2)+np.power(
            self.current_raw_robot_data["py"]-self.last_raw_robot_data["py"], 2))
        self.real_optical_data['dx'] = np.sin(dtheta)*ddistance
        self.real_optical_data['dy'] = np.cos(dtheta)*ddistance

        self.noise_optical_data['dx'] = self.real_optical_data['dx'] + \
            np.random.normal(
                self.optcal_nosie["dx"][0], self.optcal_nosie["dx"][1])
        self.noise_optical_data['dy'] = self.real_optical_data['dy'] + \
            np.random.normal(
                self.optcal_nosie["dy"][0], self.optcal_nosie["dy"][1])
        ####
        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(
            ["optical", self.real_optical_data, self.noise_optical_data])


class Datafusion:
    def __init__(self) -> None:
        pass


if __name__ == "__main__":
    app = App()
