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
robot_max_yaw_w = 0.5
lock = threading.RLock()

# robot coeffw
distance_center_to_wheel = 0.1
wheel_radius = 0.05


is_start_simulation = Value('i', 0)
generation_queue = Queue(10)
app_queue = Queue(2)


def GenerationData(is_start_simulation, generation_queue, app_queue, data_dict):
    counter = 0
    key_press_status = {"a": 0, "w": 0, "s": 0, "d": 0}
    control_robot_position_data = {"px": 0, "py": 0, "vr":
                                   0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    control_robot_position_data_last = {"px": 0, "py": 0, "vr":
                                        0, "vl": 0, "v": 0, "theta": 0, "ax": 0, "ay": 0, "t": 0}
    print(is_start_simulation.value)
    while(is_start_simulation.value):
        if(not generation_queue.empty()):
            key_press_status = generation_queue.get()
        time.sleep(0.01)
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
            dtheta = robot_max_yaw_w*dt
        elif key_press_status["d"] == 1:
            dtheta = -robot_max_yaw_w*dt
        else:
            dtheta = 0
        control_robot_position_data["theta"] = (
            control_robot_position_data_last["theta"]+dtheta)
        vx = control_robot_position_data["v"] * \
            np.cos(control_robot_position_data["theta"])
        vy = control_robot_position_data["v"] * \
            np.sin(control_robot_position_data["theta"])
        if(dtheta != 0):
            r = control_robot_position_data["v"]/robot_max_yaw_w
            d = 2*r*np.sin(abs(dtheta)/2)
            dx = d*np.cos(control_robot_position_data_last["theta"]+dtheta/2)
            dy = d*np.sin(control_robot_position_data_last["theta"]+dtheta/2)
            control_robot_position_data["px"] = (
                control_robot_position_data_last["px"]+dx)
            control_robot_position_data["py"] = (
                control_robot_position_data_last["py"]+dy)
        else:
            # #######################px,py###################################
            control_robot_position_data["px"] = (
                control_robot_position_data_last["px"]+vx*dt)
            control_robot_position_data["py"] = (
                control_robot_position_data_last["py"]+vy*dt)

        axo = (
            vx-control_robot_position_data_last["v"]*np.cos(control_robot_position_data_last["theta"]))/(dt)
        ayo = (
            vy-control_robot_position_data_last["v"]*np.sin(control_robot_position_data_last["theta"]))/(dt)

        control_robot_position_data["ax"] = np.cos(
            control_robot_position_data["theta"])*axo+np.sin(control_robot_position_data["theta"])*ayo

        control_robot_position_data["ay"] = np.cos(
            control_robot_position_data["theta"])*ayo-np.sin(control_robot_position_data["theta"])*axo
        # print("vx:%f   vy:%f    axo:%f   ayo:%f    theta:%f  aximu:%f   ayimu:%f"%(vx,vy,axo,ayo,control_robot_position_data["theta"],control_robot_position_data["ax"],control_robot_position_data["ay"]))
        if not app_queue.full():
            app_queue.put(control_robot_position_data)
        else:
            app_queue.get()
        for key in control_robot_position_data.keys():
            data_dict[key] = control_robot_position_data[key]
        control_robot_position_data_last = copy.deepcopy(
            control_robot_position_data)


class App:
    is_init_motion_handle = 0
    frame_counter = 0
    key_press_status = {"a": 0, "w": 0, "s": 0, "d": 0}
    ekf_path = {"x": [0], "y": [0], "speed": [0], "direction": [0]}
    plot_raw_robot_data = {"px": [0], "py": [0], "vr": [0],
                           "vl": [0], "v": [0], "theta": [0], "ax": [0], "ay": [0], "t": [0]}

    def __init__(self) -> None:
        self.robot = Robot()
        self.Gui()

    def Gui(self):
        dpg.create_context()
        dpg.create_viewport(title='数据融合', width=600, height=600)
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
                    x=self.plot_raw_robot_data["px"], y=self.plot_raw_robot_data["py"], tag="raw_path", parent=f"simulink_plot_y_axis")
                dpg.add_line_series(
                    x=self.ekf_path["x"], y=self.ekf_path["y"], tag="EKF_path", parent=f"simulink_plot_y_axis")
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
        global is_start_simulation, algo_res_queue, app_queue
        self.frame_counter += 1
        # print(is_start_simulation.value)
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
        if(not app_queue.empty()):
            temp_data = app_queue.get()
            for key in self.plot_raw_robot_data.keys():
                self.plot_raw_robot_data[key].append(
                    temp_data[key])
        if(not algo_res_queue.empty()):
            temp_data = algo_res_queue.get()
            # print(f"get algo data{temp_data}")
            self.ekf_path["x"].append(temp_data[0])
            self.ekf_path["y"].append(temp_data[1])
        # print(
        #     f'a:{self.key_press_status["a"]},w:{self.key_press_status["w"]},d:{self.key_press_status["d"]},s:{self.key_press_status["s"]}')
        if(self.frame_counter % 2 == 0):
            pad = 2
            # print(f"get pose data{temp_data}")
            dpg.set_value("raw_path",
                          [self.plot_raw_robot_data["px"], self.plot_raw_robot_data["py"]])
            if(self.frame_counter % 10 == 0):
                self.frame_counter = 0
                min_value = min([min(
                    self.plot_raw_robot_data["px"])-pad, min(
                    self.plot_raw_robot_data["py"])-pad, min(self.ekf_path["x"])-pad, min(self.ekf_path["y"])-pad])
                max_value = max([max(
                    self.plot_raw_robot_data["px"])+pad, max(
                    self.plot_raw_robot_data["py"])+pad, max(self.ekf_path['y'])+pad, max(self.ekf_path['x'])+pad])
                dpg.set_axis_limits("simulink_plot_x_axis",
                                    min_value, max_value)
                dpg.set_axis_limits("simulink_plot_y_axis",
                                    min_value, max_value)
                dpg.set_value(
                    "EKF_path", [self.ekf_path["x"], self.ekf_path["y"]])

    def StartSimulation(self, sender, appdata):
        global is_start_simulation, generation_queue, app_queue, control_robot_position_data_dict
        print("start sched")
        is_start_simulation.value = 1
        self.process = Process(target=GenerationData,
                               args=(is_start_simulation, generation_queue, app_queue, control_robot_position_data_dict,))
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
        self.data_fusion = Datafusion()
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
        state_variable = [0, 0, 0, 0, 0]
        print("algo")
        while(status.value):
            if(recv_queue.empty()):
                time.sleep(0.001)
                continue
            sensor_data = recv_queue.get()
            if(sensor_data[0] == "imu"):
                state_variable_predict = self.data_fusion.Predict(
                    sensor_data[2])
            # elif(sensor_data[0] == "encoder"):
            #     state_variable = self.data_fusion.Update(
            #         sensor_data[1], "encoder")
            # elif(sensor_data[0] == "optical"):
            #     state_variable = self.data_fusion.Update(
            #         sensor_data[1], "optical")
            send_queue.put(state_variable_predict)


class ImuSensor:
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "ax": 0, "ay": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    imu_nosie = {"ax_n": [0, 0.01], "ay_n": [0, 0.01], "az_n": [0, 0.1], "wx_n": [
        0, 0.01], "wy_n": [0, 0.01], "wz_n": [0, 0.001]}  # [mean,var]
    noise_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                      "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    last_imu_state = {"vxhat": 0, "vyhat": 0, "wt": 0}

    def __init__(self, q_msg) -> None:
        global sensor_timer, imu_timer
        imu_timer = sensor_timer.enter(0.010, 2, self.generate_data)
        self.robot_queue = q_msg

    def generate_data(self):
        global control_robot_position_data_dict, stop_sensor_timer, imu_timer
        if(stop_sensor_timer.value):
            return
        imu_timer = sensor_timer.enter(0.010, 2, self.generate_data)
        lock.acquire()
        for key in control_robot_position_data_dict.keys():
            self.current_raw_robot_data[key] = control_robot_position_data_dict[key]
        lock.release()
        if(self.last_raw_robot_data["t"] == 0):
            self.last_raw_robot_data = copy.deepcopy(
                self.current_raw_robot_data)
            return
        dt = self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"]
        if(dt == 0):
            return
        #####
        dd = np.sqrt(np.power((self.current_raw_robot_data["px"]-self.last_raw_robot_data["px"]), 2)+np.power(
            (self.current_raw_robot_data["py"]-self.last_raw_robot_data["py"]), 2))
        dtheta = self.current_raw_robot_data["theta"] - \
            self.last_raw_robot_data["theta"]
        if(dtheta != 0):
            w = dtheta/dt
            R = dd/2/(dtheta/2)
            vxhat = R*w
            # print("vx:%f   dd:%f   axx:%f   ayy:%f   ax:%f   ay:%f" % (vxhat, dd, self.current_raw_robot_data["ax"],
            #                                                            self.current_raw_robot_data["ay"],  self.real_imu_data["ax"], self.real_imu_data["ay"]))
        else:
            vxhat = dd/dt
        self.real_imu_data["ay"] = self.current_raw_robot_data["ay"]
        self.real_imu_data["ax"] = self.current_raw_robot_data["ax"]
        # print("vx:%f   dd:%f   axx:%f   ayy:%f   ax:%f   ay:%f" % (vxhat, dd, self.current_raw_robot_data["ax"],
        #                                                            self.current_raw_robot_data["ay"],  self.real_imu_data["ax"], self.real_imu_data["ay"]))
        # print("vx:%f   dtheta:%f  ax:%f   ay:%f" % (vxhat, dtheta,
        #       self.real_imu_data["ax"], self.real_imu_data["ay"]))
        self.last_imu_state["vxhat"] = vxhat

        self.real_imu_data["wz"] = (
            self.current_raw_robot_data["theta"]-self.last_raw_robot_data["theta"]) / dt
        self.noise_imu_data["ax"] = self.real_imu_data["ax"] + \
            np.random.normal(
                loc=self.imu_nosie["ax_n"][0], scale=self.imu_nosie["ax_n"][1], size=None)
        self.noise_imu_data["ay"] = self.real_imu_data["ay"] + \
            np.random.normal(
                loc=self.imu_nosie["ay_n"][0], scale=self.imu_nosie["ay_n"][1], size=None)
        self.noise_imu_data["wz"] = self.real_imu_data["wz"] + \
            np.random.normal(
                loc=self.imu_nosie["wz_n"][0], scale=self.imu_nosie["wz_n"][1], size=None)
        self.real_imu_data['t'] = self.current_raw_robot_data['t']
        self.noise_imu_data['t'] = self.current_raw_robot_data['t']
        # print(self.real_imu_data, self.noise_imu_data)
        ######
        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(["imu", self.real_imu_data, self.noise_imu_data])


class WheelEncoder:
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_wheel_data = {"vr": 0, "vl": 0, "t": 0}
    noise_wheel_data = {"vr": 0, "vl": 0, "t": 0}
    distance_to_center = distance_center_to_wheel  # 10cm
    encoder_nosie = {"vr": [0, 0.01], "vl": [0, 0.01]}  # [mean,var]
    r = wheel_radius

    def __init__(self, q_msg) -> None:
        global sensor_timer, encoder_timer
        encoder_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        self.robot_queue = q_msg

    def generate_data(self):
        global control_robot_position_data_dict, stop_sensor_timer, encoder_timer
        if(stop_sensor_timer.value):
            return
        encoder_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        lock.acquire()
        for key in control_robot_position_data_dict.keys():
            self.current_raw_robot_data[key] = control_robot_position_data_dict[key]
        lock.release()
        if(self.last_raw_robot_data["t"] == 0):
            self.last_raw_robot_data = copy.deepcopy(
                self.current_raw_robot_data)
            return
        if(self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"] == 0):
            return
        dtheta = self.current_raw_robot_data["theta"] - \
            self.last_raw_robot_data["theta"]
        self.real_wheel_data["vr"] = (
            self.current_raw_robot_data["v"]+dtheta*self.distance_to_center)/self.r
        self.real_wheel_data["vl"] = (
            self.current_raw_robot_data["v"]-dtheta*self.distance_to_center)/self.r

        self.noise_wheel_data["vr"] = self.real_wheel_data["vr"] + \
            np.random.normal(
                self.encoder_nosie["vr"][0], self.encoder_nosie["vr"][1])
        self.noise_wheel_data["vl"] = self.real_wheel_data["vl"] + \
            np.random.normal(
                self.encoder_nosie["vl"][0], self.encoder_nosie["vl"][1])

        self.real_wheel_data['t'] = self.current_raw_robot_data['t']
        self.noise_wheel_data['t'] = self.current_raw_robot_data['t']
        # print(self.real_wheel_data['vr'], self.real_wheel_data['vl'],
        #       self.noise_wheel_data['vr'], self.noise_wheel_data['vl'])
        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(
            ["encoder", self.real_wheel_data, self.noise_wheel_data])


class OpticalFlow:
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_optical_data = {"dx": 0, "dy": 0, "t": 0}
    noise_optical_data = {"dx": 0, "dy": 0, "t": 0}
    optcal_nosie = {"dx": [0, 0.001], "dy": [0, 0.001]}  # [mean,var]

    def __init__(self, q_msg) -> None:
        global sensor_timer, optical_timer
        optical_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        self.robot_queue = q_msg

    def generate_data(self):
        global control_robot_position_data_dict, stop_sensor_timer, optical_timer
        if(stop_sensor_timer.value):
            return
        optical_timer = sensor_timer.enter(0.050, 2, self.generate_data)
        lock.acquire()
        for key in control_robot_position_data_dict.keys():
            self.current_raw_robot_data[key] = control_robot_position_data_dict[key]
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
        self.real_optical_data['dx'] = ddistance/np.cos(dtheta)
        self.real_optical_data['dy'] = np.tan(dtheta)*ddistance

        self.noise_optical_data['dx'] = self.real_optical_data['dx'] + \
            np.random.normal(
                self.optcal_nosie["dx"][0], self.optcal_nosie["dx"][1])
        self.noise_optical_data['dy'] = self.real_optical_data['dy'] + \
            np.random.normal(
                self.optcal_nosie["dy"][0], self.optcal_nosie["dy"][1])
        # print(self.real_optical_data['dx'], self.real_optical_data['dy'],
        #       self.noise_optical_data['dx'], self.noise_optical_data['dy'])
        self.real_optical_data['t'] = self.current_raw_robot_data['t']
        self.noise_optical_data['t'] = self.current_raw_robot_data['t']
        ####
        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(
            ["optical", self.real_optical_data, self.noise_optical_data])


class Datafusion:
    state_variable_current = np.array(
        [0, 0, 0, 0, 0]).reshape(-1, 1)  # [xt,yt,vt,thetat,wt]
    state_variable_last = np.array(
        [0, 0, 0, 0, 0]).reshape(-1, 1)  # [xt-1,vt,thetat-1,wt-1]
    last_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    last_optical_data = {"dx": 0, "dy": 0, "t": 0}
    last_wheel_data = {"vr": 0, "vl": 0, "t": 0}
    last_Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    b = distance_center_to_wheel
    r = wheel_radius
    l = 0.10

    def __init__(self) -> None:
        self.Q = np.diag([0.01, 0.01, 0.01, 0.01, 0.01])
        self.R_encoder = np.diag([0.01, 0.01])
        self.R_optcal = np.diag([0.01, 0.01])
        self.H_encoder = np.array(
            [[0, 0, 1/self.r, 0, self.b/self.r], [0, 0, 1/self.r, 0, -self.b/self.r]])
        self.H_optical = np.array([[0, 0, -1, 0, 0], [0, 0, 0, 0, 1/self.l]])
        self.H_encoder_jacobian = self.H_encoder
        self.H_optical_jacobian = self.H_optical

    def check_real_data(self, sensor_data):
        if(sensor_data[0] == "imu"):
            imu_data = sensor_data[1]
            dt = imu_data["t"]-self.last_imu_data["t"]
            dtheta = self.last_imu_data["wz"]*dt
            theta_t = self.state_variable_last[4]+dtheta
            wt = imu_data["wz"]
            vxt = imu_data["ax"]*np.cos(theta_t)*dt-imu_data["ay"] * \
                np.cos(np.pi/2-theta_t)*dt+self.state_variable_last[2]
            vyt = imu_data["ax"]*np.sin(theta_t)*dt+imu_data["ay"] * \
                np.sin(np.pi/2-theta_t)*dt+self.state_variable_last[3]
            xt = self.state_variable_last[0] + \
                (vxt+self.state_variable_last[2])*dt/2
            yt = self.state_variable_last[1] + \
                (vyt+self.state_variable_last[3])*dt/2

            print("ax:%f   ay:%f   theta:%f    vx:%f    vy:%f" %
                  (imu_data["ax"], imu_data["ay"], theta_t, vxt, vyt))
            self.last_imu_data = copy.deepcopy(imu_data)
            self.state_variable_last = copy.deepcopy(
                self.state_variable_current)
            return self.state_variable_current
        if(sensor_data[0] == "encoder"):
            encoder_data = sensor_data[1]

    def Predict(self, imu_data):
        if(self.last_imu_data["t"] == 0):
            self.last_imu_data = copy.deepcopy(imu_data)
            return self.state_variable_current
        dt = imu_data["t"]-self.last_imu_data["t"]
        dtheta = self.last_imu_data["wz"]*dt
        theta_t = self.state_variable_last[3]+dtheta
        wt = imu_data["wz"]
        vxt_1 = self.state_variable_last[2] * \
            np.cos(self.state_variable_last[3])
        vyt_1 = self.state_variable_last[2] * \
            np.sin(self.state_variable_last[3])
        vxt = imu_data["ax"]*np.cos(theta_t)*dt-imu_data["ay"] * \
            np.cos(np.pi/2-theta_t)*dt+vxt_1
        vyt = imu_data["ax"]*np.sin(theta_t)*dt+imu_data["ay"] * \
            np.sin(np.pi/2-theta_t)*dt+vyt_1
        vt = np.sqrt(vxt*vxt+vyt*vyt)
        xt = self.state_variable_last[0]+(vxt+vxt_1)*dt/2
        yt = self.state_variable_last[1]+(vyt+vyt_1)*dt/2

        # print("x:%f   y:%f   ax:%f   ay:%f   theta:%f    vx:%f    vy:%f   dt%f" %
        #       (xt, yt, imu_data["ax"], imu_data["ay"], theta_t, vxt, vyt, dt))
        self.state_variable_current = np.array(
            [xt, yt, vt, theta_t, wt], dtype=np.float32)
        # print(imu_data)
        # state_jacobian_matrix = np.zeros(
        #     (self.state_variable_current.shape[0], self.state_variable_current.shape[0]))
        # state_jacobian_matrix[0, 3] = dt*dt * \
        #     (-imu_data["ax"]*np.sin(theta_t)+imu_data["ay"]*np.cos(theta_t))/2
        # state_jacobian_matrix[1, 3] = dt*dt * \
        #     (imu_data["ax"]*np.cos(theta_t)-imu_data["ay"]*np.sin(theta_t))/2
        # state_jacobian_matrix[3, 4] = dt
        # state_jacobian_matrix[4, 4] = 1
        # self.Pt = state_jacobian_matrix@self.last_Pt@state_jacobian_matrix.T+self.Q
        self.last_imu_data = copy.deepcopy(imu_data)
        self.state_variable_last = copy.deepcopy(self.state_variable_current)
        return self.state_variable_current

    def Update(self, sensor_data, sensor_type):
        if(sensor_type == "encoder"):
            H = self.H_encoder
            measure_jacobian_matrix = H
            R = self.R_encoder
            sensor_data_array = np.array(
                [sensor_data["vr"], sensor_data['vl']])
        elif(sensor_type == "optical"):
            H = self.H_optical
            measure_jacobian_matrix = H
            R = self.R_optcal
            sensor_data_array = np.array(
                [sensor_data["dx"], sensor_data['dy']])

        temp_P = H@self.Pt@H.T+R  # (2x2)
        kalman_gain = self.Pt@H.T@np.linalg.inv(temp_P)  # (5x2)
        self.state_variable_current = self.state_variable_current + \
            kalman_gain@(sensor_data_array-H@self.state_variable_current)
        self.Pt = self.Pt-kalman_gain@H@self.Pt
        self.last_Pt = copy.deepcopy(self.Pt)
        if(sensor_type == "encoder"):
            self.last_wheel_data = copy.deepcopy(sensor_data)
        elif(sensor_type == "optical"):
            self.last_optical_data = copy.deepcopy(sensor_data)
        return self.state_variable_current


if __name__ == "__main__":
    control_robot_position_data_dict = Manager().dict({"px": 0, "py": 0, "vr":
                                                       0, "vl": 0, "v": 0, "theta": 0, "ax": 0, "ay": 0, "t": 0})
    # is_start_simulation.value = 1
    # process = Process(target=GenerationData,
    #                   args=(is_start_simulation, generation_queue, app_queue,))
    # process.start()
    app = App()
    # gui = Process(target=app.Gui)
    # gui.start()
