import dearpygui.dearpygui as dpg
from multiprocessing import Process, Queue, Lock
import time
import sched
gui_label_ch = ["开始", "添加路径", "提取数据", "画图", "打开多图模式", "关闭多图模式", "日志"]
gui_label_en = ["Start", "AddPath", "InputPatten",
                "plot", "OpenMultiPlot", "CloseMultiPlot", "Logger"]

gui_label_tag = ["start", "add_path", "input_patten",
                 "plot", "open_multplot", "close_multplot", "logger"]


class App:
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
            with dpg.group(horizontal=True):
                dpg.add_button(label=gui_label_ch[0],
                               tag=gui_label_tag[0])
                dpg.add_button(label=gui_label_ch[1],
                               tag=gui_label_tag[1], callback=self.InitRobotMotion)
            dpg.add_plot(tag="simulink_plot",
                         width=dpg.get_viewport_width()-10, height=dpg.get_viewport_height()-dpg.get_item_pos(gui_label_tag[0])[1]-50)

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
        with dpg.window(tag="robot_motion"):
            dpg.add_plot(tag="motion_map")
            with dpg.item_handler_registry(tag="add_point_to_map") as handler:
                dpg.add_item_clicked_handler(callback=self.AddPointToMap)
            dpg.bind_item_handler_registry("motion_map", "add_point_to_map")
            dpg.add_line_series("motion_map", tag="head_direction_data")

            dpg.add_table(tag="point_table")

    def AddPointToMap(self, sender, appdata):
        global head_direction_data, head_speed_data, control_time, control_robot_position_data
        mx, my = dpg.get_plot_mouse_pos()
        control_robot_position_data[0].append(mx)
        control_robot_position_data[1].append(my)
        print(control_robot_position_data)
        dpg.set_value("head_direction_data", [
                      control_robot_position_data[0], control_robot_position_data[1]])


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
