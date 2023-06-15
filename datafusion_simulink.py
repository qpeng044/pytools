import dearpygui.dearpygui as dpg
from multiprocessing import Process, Queue, Lock
import time
import sched
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
            with dpg.font(r"ZiTiGuanJiaFangSongTi-2.ttf", 20) as default_font:
                dpg.add_font_range_hint(dpg.mvFontRangeHint_Chinese_Full)
        dpg.bind_font(default_font)  # Binds the font globally
        with dpg.window(tag="Primary Window"):
            global head_direction_data, head_speed_data, control_time, control_robot_position_data
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
                    x=control_robot_position_data[0], y=control_robot_position_data[1], tag="raw_path", parent=f"simulink_plot_y_axis")

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
                    x=control_robot_position_data[0], y=control_robot_position_data[1], tag="head_direction_data", parent=f"y_axis")
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
        with dpg.window(label="Set Point Info", show=False, tag="set_point_info", no_title_bar=True):
            with dpg.group(horizontal=True):
                dpg.add_text(label=gui_label_ch[4])
                # dpg.add_float_value(tag=gui_label_tag[3])
            with dpg.group(horizontal=True):
                dpg.add_button(label="OK", width=75, callback=lambda: dpg.configure_item(
                    "set_point_info", show=False))
                dpg.add_button(label="Cancel", width=75, callback=lambda: dpg.configure_item(
                    "set_point_info", show=False))

        dpg.set_viewport_resize_callback(self.viewport_resize_callback)
        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def viewport_resize_callback(self, sender, appdata):
        dpg.set_item_width("simulink_plot", dpg.get_viewport_width()-10)
        dpg.set_item_height("simulink_plot", dpg.get_viewport_height(
        )-dpg.get_item_pos(gui_label_tag[0])[1]-50)

    def InitRobotMotion(self, sender, appdata):
        dpg.configure_item("robot_motion", show=True)

    def ClearPointToMap(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data
        control_robot_position_data = [[0], [0]]
        dpg.set_value("head_direction_data", [
                      control_robot_position_data[0], control_robot_position_data[1]])

    def CloseCallback(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data
        if(sender == "robot_motion"):
            dpg.set_value("raw_path", [
                control_robot_position_data[0], control_robot_position_data[1]])
            dpg.configure_item("robot_motion", show=False)

    def RobotMotionResizeCallback(self, send, appdata):
        dpg.set_item_width("motion_map", dpg.get_item_width("robot_motion")-10)

    def AddPointToMap(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data
        mx, my = dpg.get_plot_mouse_pos()
        control_robot_position_data[0].append(mx)
        control_robot_position_data[1].append(my)
        print(control_robot_position_data)
        dpg.set_value("head_direction_data", [
                      control_robot_position_data[0], control_robot_position_data[1]])
        dpg.configure_item("set_point_info", show=True)


head_direction_data = []
head_speed_data = []  # m/s
robot_position_data = []
control_time = []
control_direction_data = []
control_speed_data = []
control_robot_position_data = [[0], [0]]


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
    def __init__(self) -> None:
        self.data_timer = sched.scheduler(time.time, time.sleep)
        self.data_timer.enter(0.01, 0, self.generate_data, ())
        self.data_timer.run()
        pass

    def generate_data(self):
        pass


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
