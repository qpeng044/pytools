import dearpygui.dearpygui as dpg
from multiprocessing import Process, Queue, Lock
import time
import sched
import numpy as np
import pandas as ps
gui_label_ch = ["开始", "添加路径", "清除路径", "设置速度", "打开多图模式", "关闭多图模式", "日志"]
gui_label_en = ["Start", "AddPath", "DeletePath",
                "SetSpeed", "OpenMultiPlot", "CloseMultiPlot", "Logger"]

gui_label_tag = ["start", "add_path", "delete_path",
                 "set_speed", "open_multplot", "close_multplot", "logger"]


class App:
    is_init_motion_handle = 0

    def __init__(self) -> None:
        self.Gui()
        word_simulink = WorldCoordinate()
        self.process = Process(target=word_simulink.run())
        self.process.start()

    def Gui(self):
        dpg.create_context()
        dpg.create_viewport(title='数据融合', width=600, height=400)
        dpg.setup_dearpygui()
        with dpg.font_registry():
            with dpg.font(r"ZiTiGuanJiaFangSongTi-2.ttf", 15) as default_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Chinese_Full)
        dpg.bind_font(default_font)  # Binds the font globally
        with dpg.window(tag="Primary Window"):
            global head_direction_data, head_speed_data, control_time, control_robot_position_data_key
            with dpg.group(horizontal=True):
                dpg.add_button(label=gui_label_ch[0],
                               tag=gui_label_tag[0])
                dpg.add_button(label=gui_label_ch[1],
                               tag=gui_label_tag[1], callback=self.InitRobotMotion)
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

        with dpg.window(tag="robot_motion", on_close=self.CloseCallback, show=False):
            dpg.add_button(label=gui_label_ch[2],
                           tag=gui_label_tag[2], callback=self.ClearPointToMap)
            with dpg.plot(tag="motion_map"):
                dpg.add_plot_legend()

                # REQUIRED: create x and y axes
                dpg.add_plot_axis(dpg.mvXAxis, label="x",
                                  tag=f"x_axis")
                dpg.add_plot_axis(dpg.mvYAxis, label="y",
                                  tag=f"y_axis")
                dpg.add_line_series(
                    x=control_robot_position_data_key[0], y=control_robot_position_data_key[1], tag="head_direction_data", parent=f"y_axis")
            with dpg.item_handler_registry(tag="add_point_to_map") as handler:
                dpg.add_item_clicked_handler(callback=self.AddPointToMap)

            with dpg.item_handler_registry(tag="robot_motion_resize") as handler:
                dpg.add_item_resize_handler(
                    callback=self.RobotMotionResizeCallback)
            dpg.bind_item_handler_registry(
                "robot_motion", "robot_motion_resize")
            dpg.bind_item_handler_registry(
                "motion_map", "add_point_to_map")
            # dpg.add_table(tag="point_table")
        with dpg.window(label="Set Point Info", show=False, tag="set_point_info", no_title_bar=True, modal=True, no_background=True):
            with dpg.group(horizontal=True):
                dpg.add_text(gui_label_ch[3])
                # dpg.add_float_value(
                #     label=gui_label_ch[3], tag=gui_label_tag[3])
                # dpg.add_drag_float(max_value=10.0, speed=0.1)
                dpg.add_input_float(
                    tag=gui_label_tag[3], default_value=0.0)
            with dpg.group(horizontal=True):
                dpg.add_button(label="OK", width=75,
                               callback=self.SetposeSpeed)

        dpg.set_viewport_resize_callback(self.viewport_resize_callback)
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def SetposeSpeed(self, sender, appdata):
        dpg.configure_item("set_point_info", show=False)
        control_robot_position_data_key[2].append(
            dpg.get_value(gui_label_tag[3]))

    def viewport_resize_callback(self, sender, appdata):
        dpg.set_item_width("simulink_plot", dpg.get_viewport_width()-10)
        dpg.set_item_height("simulink_plot", dpg.get_viewport_height(
        )-dpg.get_item_pos(gui_label_tag[0])[1]-50)

    def InitRobotMotion(self, sender, appdata):
        dpg.configure_item("robot_motion", show=True)

    def ClearPointToMap(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data_key
        control_robot_position_data_key = [[0], [0], [0]]
        dpg.set_value("head_direction_data", [
                      control_robot_position_data_key[0], control_robot_position_data_key[1]])

    def CloseCallback(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data_key
        if(sender == "robot_motion"):
            dpg.set_value("raw_path", [
                control_robot_position_data_key[0], control_robot_position_data_key[1]])
            dpg.configure_item("robot_motion", show=False)

    def RobotMotionResizeCallback(self, send, appdata):
        dpg.set_item_width("motion_map", dpg.get_item_width("robot_motion")-10)

    def AddPointToMap(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data_key
        mx, my = dpg.get_plot_mouse_pos()
        control_robot_position_data_key[0].append(mx)
        control_robot_position_data_key[1].append(my)
        print(control_robot_position_data_key)
        print("\n")
        dpg.set_value("head_direction_data", [
                      control_robot_position_data_key[0], control_robot_position_data_key[1]])
        posex, posey = dpg.get_mouse_pos()
        dpg.configure_item("set_point_info", show=True, pos=[posex, posey])


head_direction_data = []
head_speed_data = [0]  # m/s
robot_position_data = []
control_time = []
control_direction_data = []
control_speed_data = []
control_robot_position_data_key = [[0], [0], [0]]
control_robot_position_data = ps.DataFrame(
    {"px": [0], "py": [0], "vr": [0], "vl": [0], "v": [0], "theta": [0], "t": [0]})
motion_start_time = time.time()
world_sched = sched.scheduler(time.time, time.sleep)
current_pose_index = 0
robot_max_acc = 0.5
robot_max_speed = 0.5
robot_max_yaw_rate = 0.5
robot_max_yaw_acc = 0.5


def GenerateRobotPath():
    dt = 0.001  # dt=1ms
    while True:
        if(control_direction_data["px"][-1] >= control_robot_position_data_key[0][current_pose_index] and control_direction_data["py"][-1] >= control_robot_position_data_key[1][current_pose_index]):
            current_pose_index += 1
            if(current_pose_index >= len(control_robot_position_data_key[0])):
                break
        else:
            target_theta = np.arctan(
                control_robot_position_data_key[1][current_pose_index]/control_robot_position_data_key[0][current_pose_index])
            if control_direction_data["theta"][-1] >= target_theta:
                #直行
                control_direction_data["theta"].append(control_direction_data["theta"][-1] )
            else:
                #转向
                



def GetRobotInfoBytime(current_time):
    dt = current_time-motion_start_time
    theta = np.arctan(
        control_robot_position_data_key[1][current_pose_index]/control_robot_position_data_key[0][current_pose_index])
    dx = dt * \
        control_robot_position_data_key[2][current_pose_index]*np.cos(theta)
    dy = dt * \
        control_robot_position_data_key[2][current_pose_index]*np.sin(theta)


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
