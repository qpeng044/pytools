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


control_robot_position_data_key = [[0], [0], [0]]

motion_start_time = time.time()
world_sched = sched.scheduler(time.time, time.sleep)
world_sched_event = None
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


class App:
    is_init_motion_handle = 0
    frame_counter = 0
    control_robot_position_data = {"px": [0], "py": [0], "vr": [
        0], "vl": [0], "v": [0], "theta": [0], "t": [0]}
    key_press_status = {"a": 0, "w": 0, "s": 0, "d": 0}

    def __init__(self) -> None:
        self.Gui()
        word_simulink = WorldCoordinate()

    def Gui(self):
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
                    x=control_robot_position_data_key[0], y=control_robot_position_data_key[1], tag="raw_path", parent=f"simulink_plot_y_axis")
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
        global is_start_simulation
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
                for key in self.control_robot_position_data.keys():
                    self.control_robot_position_data[key].append(
                        temp_data[key])
            dpg.set_value("raw_path",
                          [self.control_robot_position_data["px"], self.control_robot_position_data["py"]])
            dpg.set_axis_limits("simulink_plot_x_axis", min(
                self.control_robot_position_data["px"])-pad, max(self.control_robot_position_data["px"])+pad)
            dpg.set_axis_limits("simulink_plot_y_axis", min(
                self.control_robot_position_data["py"])-pad, max(self.control_robot_position_data["py"])+pad)

    def StartSimulation(self, sender, appdata):
        global is_start_simulation, generation_queue, app_queue
        print("start sched")
        is_start_simulation.value = 1
        self.process = Process(target=GenerationData,
                               args=(is_start_simulation, generation_queue, app_queue,))
        self.process.start()

    def StopSimulation(self, sender, appdata):
        global world_sched, is_start_simulation
        print("stop sched")
        is_start_simulation.value = 0
        self.process.join()
        self.process.close()
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
        self.robot = Robot()

    def InitRobotMotion(self):
        pass

    def run(self):
        pass


class Robot:
    def __init__(self) -> None:
        self.imu = ImuSensor()
        self.wheel = WheelEncoder()
        self.optical = OpticalFlow()


class ImuSensor:
    imu_data = [[], [], [], [], [], []]  # [ax,ay,az,wx,wy,wz]
    imu_nosie = [[0, 0.1], [0, 0.1], [0, 0.1], [
        0, 0.01], [0, 0.01], [0, 0.01]]  # [mean,var]

    def __init__(self) -> None:
        global world_sched
        world_sched.enter(0.010, 2, self.generate_data)

    def generate_data(self):
        world_sched.enter(0.010, 2, self.generate_data)


class WheelEncoder:
    def __init__(self) -> None:
        pass


class OpticalFlow:
    def __init__(self) -> None:
        pass


class Datafusion:
    def __init__(self) -> None:
        pass


if __name__ == "__main__":
    app = App()
