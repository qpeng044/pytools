import dearpygui.dearpygui as dpg
from multiprocessing import Process, Queue, Lock, Value, Manager
import time
import sched
import numpy as np
import pandas as ps
import threading
import copy
import os
gui_label_ch = ["开始", "轮子打滑", "光滑地板", "结束", "数据验证", "日志"]
gui_label_en = ["Start", "WheelSlipp", "SmoothFloor",
                "Stop", "DataCheck", "Logger"]

gui_label_tag = ["start", "wheel_slipp", "smooth_floor",
                 "stop", "data_check", "logger"]

gui_label = {}
for index in gui_label_tag:
    gui_label[index] = {"name": gui_label_ch[gui_label_tag.index(index)],
                        "tag": index}

# ######################


motion_start_time = time.time()

sensor_timer_event = None
current_pose_index = 0
robot_max_acc = 0.2
robot_max_speed = 0.5
robot_max_yaw_rate = 0.5
robot_max_yaw_w = 0.2
lock = threading.RLock()

# robot coeffw
distance_center_to_wheel = 0.244
wheel_radius = 0.0326
encoder_resolution = 515.458152


is_start_simulation = Value('i', 0)
generation_queue = Queue(10)
app_queue = Queue(2)


algo_res_queue = Queue(6)
# sensor_timer = sched.scheduler(time.time, time.sleep)
stop_sensor_timer = Value('i', 0)

imu_odr = 0.001  # 1ms
wheel_odr = 0.001
optical_odr = 0.01


class Sensor:
    start_time = time.time()
    odr = 0.010  # 10ms

    def __init__(self) -> None:
        pass

    def CheckTime(self):
        if(time.time()-self.start_time >= self.odr):
            self.start_time = time.time()
            self.generate_data()

    def generate_data():
        pass


class App:
    is_init_motion_handle = 0
    frame_counter = 0
    key_press_status = {"a": 0, "w": 0, "s": 0, "d": 0}
    ekf_path = {"x": [0], "y": [0], "speed": [0], "direction": [0]}
    plot_raw_robot_data = {"px": [0], "py": [0], "vr": [0],
                           "vl": [0], "v": [0], "theta": [0], "ax": [0], "ay": [0], "t": [0]}
    wheel_slipp_flag = 0
    smooth_slipp_flag = 0
    res_plot_data = {}
    robot_cmd_qm = Queue(5)

    def __init__(self) -> None:
        self.Gui()

    def AddRobot(self, cmd_qm):
        robot = Robot()
        is_robot_runing = 0
        while(True):
            if(cmd_qm.empty()):
                time.sleep(0.001)
                if(robot.robot_run_start.value):
                    for sensor in robot.sensor_list:
                        sensor.CheckTime()
                continue
            cmd = cmd_qm.get()
            if(cmd == "startrun"):
                robot.StartRun()
                is_robot_runing = 1
            elif(cmd == "stoprun"):
                if(is_robot_runing):
                    robot.StopRun()
                    is_robot_runing = 0
                break
        print("stop add robot")

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
                dpg.add_button(label=gui_label["start"]["name"],
                               tag=gui_label["start"]["tag"], callback=self.StartSimulation)
                dpg.add_button(label=gui_label["stop"]["name"],
                               tag=gui_label["stop"]["tag"], callback=self.StopSimulation)
                dpg.add_button(label=gui_label["data_check"]["name"],
                               tag=gui_label["data_check"]["tag"])
                dpg.add_button(label=gui_label["wheel_slipp"]["name"],
                               tag=gui_label["wheel_slipp"]["tag"]+"enable", show=False, callback=self.WheelSlippCallback)
                dpg.add_button(label=gui_label["wheel_slipp"]["name"],
                               tag=gui_label["wheel_slipp"]["tag"]+"disable", callback=self.WheelSlippCallback)
                dpg.add_button(label=gui_label["smooth_floor"]["name"],
                               tag=gui_label["smooth_floor"]["tag"]+"enable", show=False, callback=self.SmoothFloorCallback)
                dpg.add_button(label=gui_label["smooth_floor"]["name"],
                               tag=gui_label["smooth_floor"]["tag"]+"disable", callback=self.SmoothFloorCallback)
            with dpg.theme() as self.item_theme:
                with dpg.theme_component(dpg.mvAll):
                    dpg.add_theme_color(
                        dpg.mvThemeCol_Button, (0, 200, 0), category=dpg.mvThemeCat_Core)
                    dpg.add_theme_color(
                        dpg.mvThemeCol_ButtonHovered, (0, 200, 0), category=dpg.mvThemeCat_Core)
                    dpg.add_theme_color(
                        dpg.mvThemeCol_ButtonActive, (0, 200, 0), category=dpg.mvThemeCat_Core)
                    # dpg.add_theme_style(
                    #     dpg.mvStyleVar_FrameRounding, 0, category=dpg.mvThemeCat_Core)
            dpg.bind_item_theme(
                gui_label["wheel_slipp"]["tag"]+"enable", self.item_theme)
            dpg.bind_item_theme(
                gui_label["smooth_floor"]["tag"]+"enable", self.item_theme)
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
                dpg.set_item_label("raw_path", "raw_path")
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

    def WheelSlippCallback(self, sender, appdata):
        if(sender == gui_label["wheel_slipp"]["tag"]+"enable"):
            dpg.configure_item(
                gui_label["wheel_slipp"]["tag"]+"disable", show=True)
            dpg.configure_item(
                gui_label["wheel_slipp"]["tag"]+"enable", show=False)
            self.wheel_slipp_flag = False
        else:
            dpg.configure_item(
                gui_label["wheel_slipp"]["tag"]+"enable", show=True)
            dpg.configure_item(
                gui_label["wheel_slipp"]["tag"]+"disable", show=False)
            self.wheel_slipp_flag = True

    def DataCheckCallback(self, sender, appdata):
        pass

    def SmoothFloorCallback(self, sender, appdata):
        if(sender == gui_label["smooth_floor"]["tag"]+"enable"):
            dpg.configure_item(
                gui_label["smooth_floor"]["tag"]+"disable", show=True)
            dpg.configure_item(
                gui_label["smooth_floor"]["tag"]+"enable", show=False)
            self.smooth_slipp_flag = False
        else:
            dpg.configure_item(
                gui_label["smooth_floor"]["tag"]+"enable", show=True)
            dpg.configure_item(
                gui_label["smooth_floor"]["tag"]+"disable", show=False)
            self.smooth_slipp_flag = True

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
        while not algo_res_queue.empty():
            temp_data = algo_res_queue.get()
            # print(f"get algo data{temp_data}")
            if not temp_data[0] in self.res_plot_data.keys():
                self.res_plot_data[temp_data[0]] = {"x": [], "y": []}
                print(temp_data[0])
                line_tag = temp_data[0]
                dpg.add_line_series(
                    [0], [0], tag=line_tag, parent=f"simulink_plot_y_axis")
                dpg.set_item_label(line_tag, line_tag)
            if(len(temp_data) == 3):
                self.res_plot_data[temp_data[0]]["x"].append(temp_data[1])
                self.res_plot_data[temp_data[0]]["y"].append(temp_data[2])
            elif len(temp_data) == 2:
                self.res_plot_data[temp_data[0]]["y"].append(temp_data[1])

        # print(
        #     f'a:{self.key_press_status["a"]},w:{self.key_press_status["w"]},d:{self.key_press_status["d"]},s:{self.key_press_status["s"]}')
        if(self.frame_counter % 2 == 0):
            pad = 2
            # print(f"get pose data{temp_data}")
            dpg.set_value("raw_path",
                          [self.plot_raw_robot_data["px"], self.plot_raw_robot_data["py"]])
            if(self.frame_counter % 2 == 0):
                self.frame_counter = 0
                # [min_x,min_y,max_x,max_y]

                for key in self.res_plot_data.keys():
                    y = self.res_plot_data[key]["y"]
                    if(len(self.res_plot_data[key]["x"]) == 0):
                        x = list(range(len(self.res_plot_data[key]["y"])))
                    else:
                        x = self.res_plot_data[key]["x"]
                    line_tag = key
                    dpg.set_value(line_tag, [x, y])
                    min_max_value_list = [min(x), min(y), max(x), max(y)]

                    # print(min_max_value_list)
                    dpg.set_axis_limits("simulink_plot_x_axis",
                                        min_max_value_list[0]-pad, min_max_value_list[2]+pad)
                    dpg.set_axis_limits("simulink_plot_y_axis",
                                        min_max_value_list[1]-pad, min_max_value_list[3]+pad)

    def StartSimulation(self, sender, appdata):
        global is_start_simulation, generation_queue, app_queue, control_robot_position_data_dict
        print("start")
        self.robot = Process(target=self.AddRobot, args=(self.robot_cmd_qm,))
        self.robot.daemon

        is_start_simulation.value = 1
        self.process = Process(target=GenerationData,
                               args=(is_start_simulation, generation_queue, app_queue, control_robot_position_data_dict,))
        self.process.daemon

        self.robot.start()
        self.process.start()
        self.robot_cmd_qm.put("startrun")

    def StopSimulation(self, sender, appdata):
        global is_start_simulation
        print("stop sched")
        is_start_simulation.value = 0
        self.process.join()
        self.process.close()
        self.robot_cmd_qm.put("stoprun")
        self.robot.join()
        self.robot.close()
        dpg.set_axis_limits_auto("simulink_plot_x_axis")
        dpg.set_axis_limits_auto("simulink_plot_y_axis")

    def viewport_resize_callback(self, sender, appdata):
        dpg.set_item_width("simulink_plot", dpg.get_viewport_width()-10)
        dpg.set_item_height("simulink_plot", dpg.get_viewport_height(
        )-dpg.get_item_pos(gui_label_tag[0])[1]-50)

    def CloseCallback(self, sender, appdata):
        print("close window")
        self.robot_cmd_qm.put("stoprun")
        self.robot.join()
        self.robot.close()

    def RobotMotionResizeCallback(self, send, appdata):
        dpg.set_item_width("motion_map", dpg.get_item_width("robot_motion")-10)


class WorldCoordinate:
    def __init__(self) -> None:
        pass

    def InitRobotMotion(self):
        pass

    def run(self):
        pass


class Robot:
    robot_run_start = Value("i", 0)
    robot_sensor_queue = Queue(10)
    sensor_list = []

    def __init__(self) -> None:
        self.imu = ImuSensor(self.robot_sensor_queue)
        self.sensor_list.append(self.imu)
        self.wheel = WheelEncoder(self.robot_sensor_queue)
        self.sensor_list.append(self.wheel)
        self.optical = OpticalFlow(self.robot_sensor_queue)
        self.sensor_list.append(self.optical)
        # self.data_fusion = Datafusion()
        # self.data_fusion.SaveDataToFile()
        self.data_fusion2 = Datafusion2()
        self.data_fusion2.SaveDataToFile()
        self.direct_pose = DrectPose()
        print("init robot")

    def StartRun(self):
        global stop_sensor_timer
        self.robot_run_start.value = 1
        self.process = Process(target=self.RobotAlgo2,
                               args=(self.robot_run_start, self.robot_sensor_queue, algo_res_queue,))
        self.process.daemon
        self.process.start()

    def StopRun(self):
        print("stop robot")
        self.robot_run_start.value = 0
        self.process.join()
        self.process.close()

    def check_sensor_data(self, status, recv_queue, send_queue):
        state_variable = [0, 0, 0, 0, 0]
        print("algo")
        while(status.value):
            if(recv_queue.empty()):
                time.sleep(0.001)
                continue
            sensor_data = recv_queue.get()
            state_variable_predict = self.data_fusion.check_real_data(
                sensor_data)
            send_queue.put(
                ['check_data', state_variable_predict[0], state_variable_predict[1]])

    def RobotAlgo2(self, status, recv_queue, send_queue):
        print("algo")
        while(status.value):
            if(recv_queue.empty()):
                # time.sleep(0.001)
                continue
            sensor_data = recv_queue.get()
            if(sensor_data[0] == "imu" or sensor_data[0] == "encoder" or sensor_data[0] == "optical"):
                state_variable_predict = self.data_fusion2.Predict(
                    sensor_data[1]["t"])
                state_variable_update = self.data_fusion2.Update(
                    sensor_data[1], sensor_data[0])
                # no_filter_res = self.direct_pose.GetPose(
                #     [sensor_data[0], sensor_data[2]])
                if(not send_queue.full()):
                    send_queue.put(
                        ['predict_res', state_variable_predict[0], state_variable_predict[1]], block=False)
                if(not send_queue.full()):
                    send_queue.put(
                        ['update_res', state_variable_update[0], state_variable_update[1]], block=False)
                # if(not send_queue.full()):
                #     send_queue.put(
                #         ['no_filter_res', no_filter_res[0], no_filter_res[1]], block=False)
                # print(no_filter_res[0], no_filter_res[1])

    def RobotAlgo(self, status, recv_queue, send_queue):
        state_variable = [0, 0, 0, 0, 0]
        print("algo")
        while(status.value):
            if(recv_queue.empty()):
                # time.sleep(0.001)
                continue
            sensor_data = recv_queue.get()
            if(sensor_data[0] == "imu"):
                state_variable_predict = self.data_fusion.Predict(
                    sensor_data[2])

                # state_variable_predict = self.data_fusion.check_real_data(
                #     ["imu", sensor_data[1]])
                if(not send_queue.full()):
                    send_queue.put(
                        ['predict_res', state_variable_predict[0], state_variable_predict[1]], block=False)

            elif(sensor_data[0] == "encoder"):
                pass
                # state_variable = self.data_fusion.Update(
                #     sensor_data[1], "encoder")
            elif(sensor_data[0] == "optical"):
                state_variable = self.data_fusion.Update(
                    sensor_data[1], "optical")
                # state_variable = self.data_fusion.check_real_data(
                #     ["optical", sensor_data[1]])
                if(send_queue.full()):
                    send_queue.get()
                send_queue.put(
                    ['update_res', state_variable[0], state_variable[1]], block=False)

            # if(send_queue.full()):
            #     send_queue.get()
            #     send_queue.get()

            # if(send_queue.full()):
            #     send_queue.get()


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
            if(control_robot_position_data_last["v"]+dv < 0):
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
        vx_1 = control_robot_position_data_last["v"] * \
            np.cos(control_robot_position_data_last["theta"])
        vy_1 = control_robot_position_data_last["v"] * \
            np.sin(control_robot_position_data_last["theta"])
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
            dx = 0.5*(vx+vx_1)*dt
            dy = 0.5*(vy+vy_1)*dt
            control_robot_position_data["px"] = (
                control_robot_position_data_last["px"]+dx)
            control_robot_position_data["py"] = (
                control_robot_position_data_last["py"]+dy)

        axo = (vx-vx_1)/(dt)
        ayo = (vy-vy_1)/(dt)

        control_robot_position_data["ax"] = np.cos(
            control_robot_position_data["theta"])*axo+np.sin(control_robot_position_data["theta"])*ayo

        control_robot_position_data["ay"] = np.cos(
            control_robot_position_data["theta"])*ayo-np.sin(control_robot_position_data["theta"])*axo
        # print(
        #     f'raw v:{control_robot_position_data["v"]}  theta:{control_robot_position_data["theta"]}')
        # print("vx:%f   vy:%f    axo:%f   ayo:%f    theta:%f  aximu:%f   ayimu:%f"%(vx,vy,axo,ayo,control_robot_position_data["theta"],control_robot_position_data["ax"],control_robot_position_data["ay"]))
        lock.acquire()
        if not app_queue.full():
            app_queue.put(control_robot_position_data)
        else:
            app_queue.get()
        for key in control_robot_position_data.keys():
            data_dict[key] = control_robot_position_data[key]
        lock.release()
        control_robot_position_data_last = copy.deepcopy(
            control_robot_position_data)


class ImuSensor(Sensor):
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "ax": 0, "ay": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    imu_nosie = {"ax_n": [0, 0.4], "ay_n": [0, 0.4], "az_n": [0, 0.004], "wx_n": [
        0, 0.005], "wy_n": [0, 0.005], "wz_n": [0, 0.005]}  # [mean,var]
    noise_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                      "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    last_imu_state = {"vxhat": 0, "vyhat": 0, "wt": 0}

    def __init__(self, q_msg) -> None:
        super(ImuSensor, self).__init__()
        # global sensor_timer, imu_timer
        # imu_timer = sensor_timer.enter(imu_odr, 2, self.generate_data)
        self.robot_queue = q_msg
        self.odr = imu_odr

    def generate_data(self):
        global control_robot_position_data_dict, stop_sensor_timer
        if(stop_sensor_timer.value):
            return
        # imu_timer = sensor_timer.enter(imu_odr, 2, self.generate_data)
        # lock.acquire()
        # for key in control_robot_position_data_dict.keys():
        #     self.current_raw_robot_data[key] = control_robot_position_data_dict[key]
        # lock.release()
        self.current_raw_robot_data = copy.deepcopy(
            control_robot_position_data_dict)
        if(self.last_raw_robot_data["t"] == 0):
            self.last_raw_robot_data = copy.deepcopy(
                self.current_raw_robot_data)
            return
        dt = self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"]
        if(dt == 0):
            return
        #####
        self.real_imu_data["ay"] = self.current_raw_robot_data["ay"]
        self.real_imu_data["ax"] = self.current_raw_robot_data["ax"]
        # print("vx:%f   dd:%f   axx:%f   ayy:%f   ax:%f   ay:%f" % (vxhat, dd, self.current_raw_robot_data["ax"],
        #                                                            self.current_raw_robot_data["ay"],  self.real_imu_data["ax"], self.real_imu_data["ay"]))
        # print("vx:%f   dtheta:%f  ax:%f   ay:%f" % (vxhat, dtheta,
        #       self.real_imu_data["ax"], self.real_imu_data["ay"]))
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
        # self.robot_queue.put(["imu", self.real_imu_data, self.noise_imu_data])


class WheelEncoder(Sensor):
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_wheel_data = {"nr": 0, "nl": 0, "t": 0}
    noise_wheel_data = {"nr": 0, "nl": 0, "t": 0}
    distance_to_center = distance_center_to_wheel  # 10cm
    encoder_nosie = {"nr": [0, 0.01], "nl": [0, 0.01]}  # [mean,var]
    r = wheel_radius

    def __init__(self, q_msg) -> None:
        super(WheelEncoder, self).__init__()
        # global sensor_timer, encoder_timer
        # encoder_timer = sensor_timer.enter(wheel_odr, 2, self.generate_data)
        self.robot_queue = q_msg
        self.odr = wheel_odr

    def generate_data(self):
        global control_robot_position_data_dict, stop_sensor_timer, encoder_timer
        if(stop_sensor_timer.value):
            return
        # encoder_timer = sensor_timer.enter(wheel_odr, 2, self.generate_data)
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
        dd = np.sqrt((self.current_raw_robot_data["px"]-self.last_raw_robot_data["px"])**2+(
            self.current_raw_robot_data["py"]-self.last_raw_robot_data["py"])**2)
        dr = dd+dtheta*self.distance_to_center
        dl = dd-dtheta*self.distance_to_center
        self.real_wheel_data["nr"] = dr*encoder_resolution/(self.r*2*np.pi)
        self.real_wheel_data["nl"] = dl*encoder_resolution/(self.r*2*np.pi)
        self.real_wheel_data['t'] = self.current_raw_robot_data['t']

        ####old method####
        # self.real_wheel_data["nr"] = (
        #     self.current_raw_robot_data["v"]+dtheta*self.distance_to_center)/self.r
        # self.real_wheel_data["nl"] = (
        #     self.current_raw_robot_data["v"]-dtheta*self.distance_to_center)/self.r

        # self.noise_wheel_data["nr"] = self.real_wheel_data["nr"] + \
        #     np.random.normal(
        #         self.encoder_nosie["nr"][0], self.encoder_nosie["nr"][1])
        # self.noise_wheel_data["nl"] = self.real_wheel_data["nl"] + \
        #     np.random.normal(
        #         self.encoder_nosie["nl"][0], self.encoder_nosie["nl"][1])

        # self.real_wheel_data['t'] = self.current_raw_robot_data['t']
        # self.noise_wheel_data['t'] = self.current_raw_robot_data['t']
        ########################
        # print(self.real_wheel_data['vr'], self.real_wheel_data['vl'],
        #       self.noise_wheel_data['vr'], self.noise_wheel_data['vl'])
        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)
        self.robot_queue.put(
            ["encoder", self.real_wheel_data, self.real_wheel_data])


class OpticalFlow(Sensor):
    current_raw_robot_data = {"px": 0, "py": 0,
                              "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    last_raw_robot_data = {"px": 0, "py": 0,
                           "vr": 0, "vl": 0, "v": 0, "theta": 0, "t": 0}
    real_optical_data = {"dx": 0, "dy": 0, "theta": 0, "t": 0}
    noise_optical_data = {"dx": 0, "dy": 0, "theta": 0, "t": 0}
    optcal_nosie = {"dx": [0, 0.0001], "dy": [0, 0.0001]}  # [mean,var]
    odr_start_time = time.time()

    def __init__(self, q_msg) -> None:
        super(OpticalFlow, self).__init__()
        # global sensor_timer, optical_timer
        # optical_timer = sensor_timer.enter(optical_odr, 2, self.generate_data)
        self.robot_queue = q_msg
        self.odr = optical_odr

    def generate_data(self):
        self.odr_start_time = time.time()
        global control_robot_position_data_dict, stop_sensor_timer
        # if(stop_sensor_timer.value):
        #     return
        # optical_timer = sensor_timer.enter(optical_odr, 2, self.generate_data)
        # lock.acquire()
        # for key in control_robot_position_data_dict.keys():
        #     self.current_raw_robot_data[key] = control_robot_position_data_dict[key]
        # lock.release()
        self.current_raw_robot_data = copy.deepcopy(
            control_robot_position_data_dict)
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
        dt = self.current_raw_robot_data["t"]-self.last_raw_robot_data["t"]
        self.real_optical_data['dx'] = ddistance*np.cos(dtheta*0.5)
        self.real_optical_data['dy'] = np.sin(dtheta*0.5)*ddistance
        self.real_optical_data["theta"] = self.current_raw_robot_data["theta"]
        self.noise_optical_data['dx'] = self.real_optical_data['dx'] + \
            np.random.normal(
                self.optcal_nosie["dx"][0], self.optcal_nosie["dx"][1])
        self.noise_optical_data['dy'] = self.real_optical_data['dy'] + \
            np.random.normal(
                self.optcal_nosie["dy"][0], self.optcal_nosie["dy"][1])
        self.noise_optical_data['theta'] = self.real_optical_data["theta"]
        # print(self.real_optical_data['dx'], self.real_optical_data['dy'],
        #       self.noise_optical_data['dx'], self.noise_optical_data['dy'])
        self.real_optical_data['t'] = self.current_raw_robot_data['t']
        self.noise_optical_data['t'] = self.current_raw_robot_data['t']
        ####

        self.last_raw_robot_data = copy.deepcopy(
            self.current_raw_robot_data)

        self.robot_queue.put(
            ["optical", self.real_optical_data, self.noise_optical_data])


class Datafusion2:
    state_variable_current = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vxt,vyt,theta_t,wt]
    state_variable_last = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vt,theta_t,wt]
    ofs_state_variable_last = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vt,theta_t,wt]
    state_variable_last_t = 0
    state_variable_predic = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vt,theta_t,wt]
    last_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    last_optical_data = {"dx": 0, "dy": 0, "theta": 0, "t": 0}
    last_encoder_data = {"nr": 0, "nl": 0, "theta": 0, "t": 0}
    last_Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    b = distance_center_to_wheel
    r = wheel_radius
    l = 0.10
    save_data_to_file = 0
    data_file_name_base = "data_fusion_sensor"
    file_num = 0

    def __init__(self) -> None:
        self.Q = np.diag([0.1, 0.1, 0.1,
                         0.1, 0.1, 0.1])
        self.R_encoder = np.diag([0.001, 0.001])
        self.R_optcal = np.diag([0.0001, 0.0001])
        self.R_imu = np.diag([0.1, 0.1, 0.1])

    def SaveDataToFile(self):
        self.save_data_to_file = 1
        while(True):
            data_file_name = f"{self.data_file_name_base}{self.file_num}.dat"
            if os.path.exists(data_file_name):
                self.file_num += 1
            else:
                break
        self.fid = open(data_file_name, 'a+')

    def Predict(self, current_time):
        if(self.state_variable_last_t == 0):
            self.state_variable_last_t = current_time
            return self.state_variable_current
        dt = current_time-self.state_variable_last_t
        if(dt == 0):
            return self.state_variable_current
        theta_t = self.state_variable_current[2]
        A = np.array([[1, 0, dt, 0, 0, 0],
                      [0, 1, 0, dt, 0, 0],
                      [0, 0, 1, 0, 0, 0],
                      [0, 0, 0, 1, 0, 0],
                      [0, 0, 0, 0, 1, dt],
                      [0, 0, 0, 0, 0, 1]])
        self.state_variable_current = A@self.state_variable_current
        self.Pt = A @ self.Pt @ A.T + self.Q
        self.state_variable_last_t = current_time
        # print(self.state_variable_current)
        return self.state_variable_current

    def Update(self, sensor_data, sensor_type):
        if(sensor_type == "imu"):
            imu_data = sensor_data
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                self.fid.write(
                    f"imu_data:{imu_data['t']} {imu_data['ax']} {imu_data['ay']} {imu_data['az']} {imu_data['wx']} {imu_data['wy']} {imu_data['wz']}\n")
            if(self.last_imu_data["t"] == 0):
                self.last_imu_data = copy.deepcopy(imu_data)
                return self.state_variable_current
            dt = imu_data["t"]-self.last_imu_data["t"]
            if(dt == 0):
                return self.state_variable_current
            axt = (self.state_variable_current[2] -
                   self.state_variable_last[2])/dt
            ayt = (self.state_variable_current[3] -
                   self.state_variable_last[3])/dt
            theta_t = self.state_variable_current[4]
            H = np.array([[0, 0, np.cos(theta_t)/dt, np.sin(theta_t)/dt, -axt*np.sin(theta_t)+ayt*np.cos(theta_t), 0],
                          [0, 0, -np.sin(theta_t)/dt, np.cos(theta_t)/dt, -axt*np.cos(theta_t)-ayt*np.sin(theta_t), 0], [0, 0, 0, 0, 0, 1]])
            R = self.R_imu
            sensor_data_array = np.array(
                [sensor_data["ax"], sensor_data['ay'], sensor_data['wz']])
            Z = np.array([axt*np.cos(theta_t)+ayt*np.sin(theta_t), ayt *
                         np.cos(theta_t)-axt*np.sin(theta_t), self.state_variable_current[-1]])
            error = sensor_data_array-Z
        if(sensor_type == "encoder22"):
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                self.fid.write(
                    f"encoder_data:{sensor_data['t']} {sensor_data['nl']} {sensor_data['nr']}\n")
            if(self.last_encoder_data["t"] == 0):
                self.last_encoder_data = copy.deepcopy(sensor_data)
                return self.state_variable_current
            dt = sensor_data["t"]-self.last_encoder_data["t"]
            if(dt == 0):
                return self.state_variable_current
            vt = np.float64(np.sqrt(self.state_variable_current[2]*self.state_variable_current[2] +
                                    self.state_variable_current[3]*self.state_variable_current[3]))

            if(vt <= 0.00001):
                H = np.array([[0, 0, 1/self.r, 1/self.r, 0, self.b/self.r],
                              [0, 0, 1/self.r, 1/self.r, 0, -self.b/self.r]])
            else:
                H = np.array([[0, 0, self.state_variable_current[2]/vt/self.r, self.state_variable_current[3]/vt/self.r, 0, self.b/self.r],
                              [0, 0, self.state_variable_current[2]/vt/self.r, self.state_variable_current[3]/vt/self.r, 0, -self.b/self.r]])
            R = self.R_encoder
            sensor_data_array = np.array(
                [sensor_data["nr"]*2*np.pi/encoder_resolution/dt, sensor_data['nl']*2*np.pi/encoder_resolution/dt])
            Z = np.array([vt+self.b*self.state_variable_current[-1],
                         vt-self.b*self.state_variable_current[-1]])/self.r
            error = sensor_data_array-Z
            print(f"error:{error},vt{vt}")
        if(sensor_type == "encoder33"):
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                self.fid.write(
                    f"encoder_data:{sensor_data['t']} {sensor_data['nl']} {sensor_data['nr']}\n")
            if(self.last_encoder_data["t"] == 0):
                self.last_encoder_data = copy.deepcopy(sensor_data)
                return self.state_variable_current
            dt = sensor_data["t"]-self.last_encoder_data["t"]
            if(dt == 0):
                return self.state_variable_current
            theta_t = self.state_variable_current[4]
            A = np.array([[0, 0], [0, 0], [np.cos(theta_t)*self.r/2, np.cos(theta_t)*self.r/2], [np.sin(
                theta_t)*self.r/2, np.sin(theta_t)*self.r/2], [0, 0], [self.r/self.b, -self.r/self.b]])
            H = np.linalg.inv(A)
            R = self.R_encoder
            sensor_data_array = np.array(
                [sensor_data["nr"]*2*np.pi*self.r/encoder_resolution/dt, sensor_data['nl']*2*np.pi*self.r/encoder_resolution/dt])
            Z = np.array([ddt+self.b*dtheta,
                         ddt-self.b*dtheta])
            error = sensor_data_array-Z
            print(f"error:{error},ddt{ddt}")
        if(sensor_type == "encoder"):
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                self.fid.write(
                    f"encoder_data:{sensor_data['t']} {sensor_data['nl']} {sensor_data['nr']}\n")
            if(self.last_encoder_data["t"] == 0):
                self.last_encoder_data = copy.deepcopy(sensor_data)
                return self.state_variable_current
            dt = sensor_data["t"]-self.last_encoder_data["t"]
            if(dt == 0):
                return self.state_variable_current
            dxt = self.state_variable_current[0]-self.state_variable_last[0]
            dyt = self.state_variable_current[1]-self.state_variable_last[1]
            ddt = np.float64(np.sqrt((dxt)**2 + (dyt)**2))
            dtheta = self.state_variable_current[-1]*dt

            H = np.array([[np.cos(dtheta/2+self.state_variable_last[-2]), np.sin(dtheta/2+self.state_variable_last[-2]), 0, 0, 0, self.b*dt],
                          [np.cos(dtheta/2+self.state_variable_last[-2]), np.sin(dtheta/2+self.state_variable_last[-2]), 0, 0, 0, -self.b*dt]])
            R = self.R_encoder
            sensor_data_array = np.array(
                [sensor_data["nr"]*2*np.pi*self.r/encoder_resolution, sensor_data['nl']*2*np.pi*self.r/encoder_resolution])
            Z = np.array([ddt+self.b*dtheta,
                         ddt-self.b*dtheta])
            error = sensor_data_array-Z
            print(f"error:{error},ddt{ddt}")
        elif(sensor_type == "optical00"):
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                # print(sensor_data)
                self.fid.write(
                    f"optical_data:{sensor_data['t']} {sensor_data['dx']} {sensor_data['dy']}\n")
            if(self.last_optical_data["t"] == 0):
                self.last_optical_data = copy.deepcopy(sensor_data)
                return self.state_variable_current
            dt = sensor_data["t"]-self.last_optical_data["t"]
            R = self.R_optcal
            sensor_data_array = np.array(
                [sensor_data["dx"], sensor_data['dy']])
            dx = self.state_variable_current[0]-self.ofs_state_variable_last[0]
            dy = self.state_variable_current[1]-self.ofs_state_variable_last[1]
            dd = np.sqrt(dx**2+dy**2)
            dtheta = self.ofs_state_variable_last[4]*dt
            H = np.array([[np.cos(self.state_variable_last[-2]+dtheta/2)*np.cos(dtheta/2), np.sin(self.state_variable_last[-2]+dtheta/2)*np.cos(dtheta/2), 0, 0, dd*np.sin(dtheta/2)/2, 0], [
                np.cos(self.state_variable_last[-2]+dtheta/2)*np.sin(dtheta/2), np.sin(self.state_variable_last[-2]+dtheta/2)*np.sin(dtheta/2), 0, 0, dd*np.cos(dtheta/2)/2, 0]])
            Z = np.array([np.cos(dtheta/2)*dd, dd*np.sin(dtheta/2)])
            error = sensor_data_array-Z
        elif(sensor_type == "optical"):
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                # print(sensor_data)
                self.fid.write(
                    f"optical_data:{sensor_data['t']} {sensor_data['dx']} {sensor_data['dy']}\n")
            if(self.last_optical_data["t"] == 0):
                self.last_optical_data = copy.deepcopy(sensor_data)
                return self.state_variable_current
            dt = sensor_data["t"]-self.last_optical_data["t"]
            R = self.R_optcal
            sensor_data_array = np.array(
                [sensor_data["dx"], sensor_data['dy']])
            dx = self.state_variable_current[2]*dt
            dy = self.state_variable_current[3]*dt
            dd = np.sqrt(dx**2+dy**2)
            dtheta = self.ofs_state_variable_last[4]*dt
            H = np.array([[0, 0, np.cos(self.state_variable_last[-2]+dtheta/2)*np.cos(dtheta/2), np.sin(self.state_variable_last[-2]+dtheta/2)*np.cos(dtheta/2), 0,  -dd*np.sin(dtheta/2)/2], [
                0, 0, np.cos(self.state_variable_last[-2]+dtheta/2)*np.sin(dtheta/2), np.sin(self.state_variable_last[-2]+dtheta/2)*np.sin(dtheta/2), 0,  dd*np.cos(dtheta/2)/2]])*dt
            Z = np.array([np.cos(dtheta/2)*dd, dd*np.sin(dtheta/2)])
            error = sensor_data_array-Z
        temp_P = H@self.Pt@H.T+R  # (2x2)
        kalman_gain = self.Pt@H.T@np.linalg.inv(temp_P)  # (6x2)
        correct_value = kalman_gain@(error)
        # print(f"kalman:{kalman_gain}")
        # print(f"correct:{correct_value}")
        self.state_variable_current = self.state_variable_current + correct_value
        self.Pt = self.Pt-kalman_gain@H@self.Pt

        self.last_Pt = copy.deepcopy(self.Pt)
        self.state_variable_last = copy.deepcopy(self.state_variable_current)
        if(sensor_type == "imu"):
            self.last_imu_data = copy.deepcopy(sensor_data)
        if(sensor_type == "encoder"):
            self.last_encoder_data = copy.deepcopy(sensor_data)
        if(sensor_type == "optical"):
            self.last_optical_data = copy.deepcopy(sensor_data)
            self.ofs_state_variable_last = copy.deepcopy(
                self.state_variable_current)
        return self.state_variable_current


class DrectPose:
    state_variable_current = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vxt,vyt,theta_t,wt]
    state_variable_last = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vt,theta_t,wt]
    state_variable_last_t = 0
    state_variable_predic = np.array(
        [0, 0, 0, 0, 0, 0])  # [xt,yt,vt,theta_t,wt]
    last_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    last_optical_data = {"dx": 0, "dy": 0, "theta": 0, "t": 0}
    last_encoder_data = {"nr": 0, "nl": 0, "theta": 0, "t": 0}
    last_Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    b = distance_center_to_wheel
    r = wheel_radius
    l = 0.10
    save_data_to_file = 0
    data_file_name_base = "data_fusion_sensor"
    file_num = 0

    def SaveDataToFile(self):
        self.save_data_to_file = 1
        while(True):
            data_file_name = f"{self.data_file_name_base}{self.file_num}.dat"
            if os.path.exists(data_file_name):
                self.file_num += 1
            else:
                break

        self.fid = open(data_file_name, 'a+')

    def GetPose(self, sensor_data):
        if(sensor_data[0] == "imu"):
            imu_data = sensor_data[1]
            if(self.last_imu_data["t"] == 0):
                self.last_imu_data = copy.deepcopy(imu_data)
                return self.state_variable_current
            dt = imu_data["t"]-self.last_imu_data["t"]
            dtheta = (imu_data["wz"]+self.last_imu_data["wz"])*dt/2
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
            self.state_variable_current = np.array(
                [xt, yt, vxt, vyt, theta_t, wt], dtype=np.float32)
            # print(self.state_variable_last)
            # print("vxt:%f  vyt:%f   ax:%f   ay:%f   theta:%f    dt:%f" %
            #       (vxt, vyt, imu_data["ax"], imu_data["ay"], theta_t, dt))
            self.last_imu_data = copy.deepcopy(imu_data)
            self.state_variable_last = copy.deepcopy(
                self.state_variable_current)
            return self.state_variable_current


class Datafusion:
    state_variable_current = np.array(
        [0, 0, 0, 0, 0])  # [xt,yt,vt,thetat,wt]
    state_variable_last = np.array(
        [0, 0, 0, 0, 0])  # [xt-1,vt,thetat-1,wt-1]
    imu_state_variable_last = np.array(
        [0, 0, 0, 0, 0])  # [xt-1,vt,thetat-1,wt-1]
    ofs_state_variable_last = np.array(
        [0, 0, 0, 0, 0])  # [xt-1,vt,thetat-1,wt-1]
    encoder_state_variable_last = np.array(
        [0, 0, 0, 0, 0])  # [xt-1,vt,thetat-1,wt-1]
    last_imu_data = {"ax": 0, "ay": 0, "az": 0, "wx": 0,
                     "wy": 0, "wz": 0, "t": 0}  # [ax,ay,az,wx,wy,wz,t]
    last_optical_data = {"dx": 0, "dy": 0, "theta": 0, "t": 0}
    last_encoder_data = {"nr": 0, "nl": 0, "theta": 0, "t": 0}
    last_Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    Pt = np.zeros(
        (state_variable_current.shape[0], state_variable_current.shape[0]))
    b = distance_center_to_wheel
    r = wheel_radius
    l = 0.10
    save_data_to_file = 0
    data_file_name_base = "data_fusion_sensor"
    file_num = 0

    def __init__(self) -> None:
        self.Q = np.diag([0.001, 0.001, 0.001, 0.001, 0.001])
        self.R_encoder = np.diag([0.01, 0.01])
        self.R_optcal = np.diag([0.1, 0.1])
        self.H_encoder = np.array(
            [[0, 0, 1/self.r, 0, self.b/self.r], [0, 0, 1/self.r, 0, -self.b/self.r]])
        self.H_encoder_jacobian = self.H_encoder

    def SaveDataToFile(self):
        self.save_data_to_file = 1
        while(True):
            data_file_name = f"{self.data_file_name_base}{self.file_num}.dat"
            if os.path.exists(data_file_name):
                self.file_num += 1
            else:
                break

        self.fid = open(data_file_name, 'a+')

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
            vt = np.sqrt(vxt*vxt+vyt*vyt)
            self.state_variable_current = np.array(
                [xt, yt, vt, theta_t, wt], dtype=np.float32)
            print(self.state_variable_last)
            # print("xt:%f  yt:%f   ax:%f   ay:%f   theta:%f    vx:%f    vy:%f" %
            #       (xt, yt, imu_data["ax"], imu_data["ay"], theta_t, vxt, vyt))
            self.last_imu_data = copy.deepcopy(imu_data)
            self.state_variable_last = copy.deepcopy(
                self.state_variable_current)
            return self.state_variable_current
        if(sensor_data[0] == "encoder"):
            global encoder_resolution
            encoder_data = sensor_data[1]
            if(self.last_encoder_data["t"] == 0):
                self.last_encoder_data = copy.deepcopy(encoder_data)
                return self.state_variable_current
            dt = encoder_data["t"]-self.last_encoder_data["t"]
            dl = np.pi*2*self.r*encoder_data["nl"]/encoder_resolution
            dr = np.pi*2*self.r*encoder_data["nr"]/encoder_resolution
            dd = 0.5*(dl+dr)
            dtheta = (dr-dl)*0.5/distance_center_to_wheel

            dx = dd*np.cos(self.state_variable_last[3]+0.5*dtheta)
            dy = dd*np.sin(self.state_variable_last[3]+0.5*dtheta)
            # print(self.state_variable_last[0] +
            #       dx, self.state_variable_last[1] + dy)
            xt = self.state_variable_last[0] + dx
            yt = self.state_variable_last[1] + dy
            vt = dd/dt
            theta_t = self.state_variable_last[3]+dtheta
            wt = dtheta/dt
            self.state_variable_current = np.array(
                [xt, yt, vt, theta_t, wt], dtype=np.float32)
            self.last_encoder_data = copy.deepcopy(encoder_data)
            self.state_variable_last = copy.deepcopy(
                self.state_variable_current)
            # print(self.state_variable_current)
            return self.state_variable_current
        if sensor_data[0] == "optical":
            optical_data = sensor_data[1]
            if(self.last_optical_data["t"] == 0):
                self.last_optical_data = copy.deepcopy(optical_data)
                return self.state_variable_current
            dt = optical_data["t"]-self.last_optical_data["t"]
            if dt == 0:
                return self.state_variable_last
            dx_optical = optical_data["dx"]
            dy_optical = optical_data["dy"]
            theta_t = optical_data["theta"]
            dtheta = theta_t-self.state_variable_last[3]
            dd = np.sqrt(dx_optical*dx_optical+dy_optical*dy_optical)
            dx = dd*np.cos(self.state_variable_last[3]+0.5*dtheta)
            dy = dd*np.sin(self.state_variable_last[3]+0.5*dtheta)
            xt = self.state_variable_last[0] + dx
            yt = self.state_variable_last[1] + dy
            vt = dd/dt
            wt = dtheta/dt
            self.state_variable_current = np.array(
                [xt, yt, vt, theta_t, wt], dtype=np.float32)
            self.last_optical_data = copy.deepcopy(optical_data)
            self.state_variable_last = copy.deepcopy(
                self.state_variable_current)
            # print(self.state_variable_current)
            return self.state_variable_current

    def Predict(self, imu_data):
        start_time = time.time()
        if(self.save_data_to_file):
            # with open("data_fusion_sensor.dat", 'a+')as fid:
            self.fid.write(
                f"imu_data:{imu_data['t']} {imu_data['ax']} {imu_data['ay']} {imu_data['az']} {imu_data['wx']} {imu_data['wy']} {imu_data['wz']}\n")
        if(self.last_imu_data["t"] == 0):
            self.last_imu_data = copy.deepcopy(imu_data)
            return self.state_variable_current
        dt = imu_data["t"]-self.last_imu_data["t"]
        dtheta = self.last_imu_data["wz"]*dt
        theta_t = self.imu_state_variable_last[3]+dtheta
        wt = imu_data["wz"]
        vxt_1 = self.imu_state_variable_last[2] * \
            np.cos(self.imu_state_variable_last[3])
        vyt_1 = self.imu_state_variable_last[2] * \
            np.sin(self.imu_state_variable_last[3])
        vxt = imu_data["ax"]*np.cos(theta_t)*dt-imu_data["ay"] * \
            np.cos(np.pi/2-theta_t)*dt+vxt_1
        vyt = imu_data["ax"]*np.sin(theta_t)*dt+imu_data["ay"] * \
            np.sin(np.pi/2-theta_t)*dt+vyt_1
        vt = np.sqrt(vxt*vxt+vyt*vyt)
        xt = self.imu_state_variable_last[0]+(vxt+vxt_1)*dt/2
        yt = self.imu_state_variable_last[1]+(vyt+vyt_1)*dt/2
        # print(self.imu_state_variable_last)
        # print(f'**imu vxt:{vxt},vyt:{vyt}')
        # print("x:%f   y:%f   ax:%f   ay:%f   theta:%f    vx:%f    vy:%f   dt%f" %
        #       (xt, yt, imu_data["ax"], imu_data["ay"], theta_t, vxt, vyt, dt))
        self.state_variable_current = np.array(
            [xt, yt, vt, theta_t, wt], dtype=np.float32)
        # print(imu_data)
        state_jacobian_matrix = np.zeros(
            (self.state_variable_current.shape[0], self.state_variable_current.shape[0]))
        state_jacobian_matrix[0, 3] = dt * \
            (vyt_1-vyt)/2
        state_jacobian_matrix[1, 3] = dt * \
            (vxt-vxt_1)/2
        if(vt != 0):
            state_jacobian_matrix[2, 3] = dt * \
                ((vyt_1-vyt)*vyt+(vxt-vxt_1)*vxt)/vt
        state_jacobian_matrix[3, 4] = dt
        state_jacobian_matrix[4, 4] = 1
        self.Pt = state_jacobian_matrix@self.last_Pt@state_jacobian_matrix.T+self.Q
        self.last_imu_data = copy.deepcopy(imu_data)
        # print(f"pt:{self.Pt}")
        # print(f"predict cost time{time.time()-start_time}")
        self.imu_state_variable_last = copy.deepcopy(
            self.state_variable_current)
        return self.state_variable_current

    def Update(self, sensor_data, sensor_type):
        global encoder_resolution
        start_time = time.time()
        if(sensor_type == "encoder"):
            H = self.H_encoder
            measure_jacobian_matrix = H
            R = self.R_encoder
            sensor_data_array = np.array(
                [sensor_data["nr"]*2*np.pi/encoder_resolution, sensor_data['nl']*2*np.pi/encoder_resolution])
            error = sensor_data_array-H @ self.state_variable_current
            # print(
            #     f'error:{error},sensor:{sensor_data_array},predict:{H @ self.state_variable_current}')
        elif(sensor_type == "optical"):
            if(self.save_data_to_file):
                # with open("data_fusion_sensor.dat", 'a+')as fid:
                self.fid.write(
                    f"optical_data:{sensor_data['t']} {sensor_data['dx']} {sensor_data['dy']}\n")
            R = self.R_optcal
            sensor_data_array = np.array(
                [sensor_data["dx"], sensor_data['dy']])
            dx = self.state_variable_current[0]-self.ofs_state_variable_last[0]
            dy = self.state_variable_current[1]-self.ofs_state_variable_last[1]
            dd = np.sqrt(dx**2+dy**2)
            dtheta = self.state_variable_current[3] - \
                self.ofs_state_variable_last[3]
            if(dd != 0):
                H = np.array([[dx*np.cos(dtheta/2)/dd, dy*np.cos(dtheta/2)/dd, 0, dd*np.sin(dtheta/2), 0], [
                             dx*np.sin(dtheta/2)/dd, dy*np.sin(dtheta/2)/dd, 0, dd*np.cos(dtheta/2), 0]])
            else:
                H = np.array([[0, 0, 0, 0, 0], [0, 0, 0, 0, 0]])
            Z = np.array([np.cos(dtheta/2)*dd, dd*np.sin(dtheta/2)])
            error = sensor_data_array-Z
            # print(f'error:{error},sensor:{sensor_data_array},predict:{Z}')
        # self.state_variable_last = copy.deepcopy(self.state_variable_current)
        temp_P = H@self.Pt@H.T+R  # (2x2)
        kalman_gain = self.Pt@H.T@np.linalg.inv(temp_P)  # (5x2)
        correct_value = kalman_gain@(error)
        # print(f"kalman:{kalman_gain}")
        # print(f"correct:{correct_value}")
        self.state_variable_current = self.state_variable_current + correct_value
        self.Pt = self.Pt-kalman_gain@H@self.Pt
        self.last_Pt = copy.deepcopy(self.Pt)

        if(sensor_type == "encoder"):
            self.last_encoder_data = copy.deepcopy(sensor_data)
            self.encoder_state_variable_last = copy.deepcopy(
                self.state_variable_current)
        elif(sensor_type == "optical"):
            self.last_optical_data = copy.deepcopy(sensor_data)
            self.ofs_state_variable_last = copy.deepcopy(
                self.state_variable_current)
        # self.imu_state_variable_last = copy.deepcopy(
        #     self.state_variable_current)
        # print(f"update cost time{time.time()-start_time}")
        return self.state_variable_current


if __name__ == "__main__":
    control_robot_position_data_dict = Manager().dict({"px": 0, "py": 0, "vr":
                                                       0, "vl": 0, "v": 0, "theta": 0, "ax": 0, "ay": 0, "t": 0})
    app = App()
