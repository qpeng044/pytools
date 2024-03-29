import re
import dearpygui.dearpygui as dpg
import os


class GUI():
    log_info = ''
    file = ''
    plot_data = []
    plot_fig_num = 1
    pattern_history = []
    pattern_history_file = "./pattern_history_file.txt"
    multi_plot_mode = 0

    def __init__(self) -> None:
        if(os.path.exists(self.pattern_history_file)):
            with open(self.pattern_history_file, 'r') as fid:
                line = fid.readline()
                print(line)
                while(line):
                    self.pattern_history.append(line.replace("\n", ""))
                    line = fid.readline()
        dpg.create_context()
        dpg.create_viewport(width=600, height=400)
        dpg.setup_dearpygui()
        with dpg.window(tag="Primary Window"):
            # dpg.show_imgui_demo()
            with dpg.file_dialog(directory_selector=False, show=False, callback=self.callback, id="file_dialog_id", width=400, height=300):
                dpg.add_file_extension(".*")
                dpg.add_file_extension("", color=(150, 255, 150, 255))
                dpg.add_file_extension(
                    "Source files (*.cpp *.h *.hpp){.cpp,.h,.hpp}", color=(0, 255, 255, 255))
                dpg.add_file_extension(".log", color=(
                    255, 0, 255, 255), custom_text="[433]")
                dpg.add_file_extension(".py", color=(
                    0, 255, 0, 255), custom_text="[Python]")
            with dpg.group(horizontal=True):
                dpg.add_text("select log file to plot")
                dpg.add_button(label="select file",
                               callback=lambda: dpg.show_item("file_dialog_id"))
            with dpg.group(horizontal=True, tag="input_patten_and_history"):
                dpg.add_text("input patten:")
                dpg.add_input_text(tag="input_string", width=400)
                dpg.add_combo(items=self.pattern_history,
                              tag="pattern_history", callback=self.select_pattern)
            with dpg.group(horizontal=True):
                dpg.add_button(
                    label=f"plot{self.plot_fig_num}", callback=self.plot_callback, tag=f"plot_callback{self.plot_fig_num}")
                dpg.add_button(
                    label=f"+", callback=self.add_plot_callback, tag="add_plot")
                dpg.add_button(label=f"-", callback=self.delete_plot_callback)
                dpg.add_button(label=f"open multi-plot",
                               callback=self.multi_plot_callback, tag="multi_plot")
            with dpg.plot(label=f"figure{self.plot_fig_num}", width=-1, tag=f"plot{self.plot_fig_num}"):
                # optionally create legend
                dpg.add_plot_legend()

                # REQUIRED: create x and y axes
                dpg.add_plot_axis(dpg.mvXAxis, label="x",
                                  tag=f"x_axis{self.plot_fig_num}")
                dpg.add_plot_axis(dpg.mvYAxis, label="y",
                                  tag=f"y_axis{self.plot_fig_num}")

                # series belong to a y axis
                dpg.add_line_series(list(range(len(
                    self.plot_data))), self.plot_data, label="", parent=f"y_axis{self.plot_fig_num}", tag=f"series_tag{self.plot_fig_num}")
            dpg.add_text("logger:", tag="log_header")
            dpg.add_text('', tag='log_text')

        dpg.show_viewport()
        dpg.set_primary_window("Primary Window", True)
        dpg.start_dearpygui()
        dpg.destroy_context()

    def multi_plot_callback(self, sender, app_data):
        if self.multi_plot_mode:
            self.multi_plot_mode = 0
            dpg.set_item_label("multi_plot", "open multi-plot")
        else:
            self.multi_plot_mode = 1
            dpg.set_item_label("multi_plot", "close multi-plot")

    def select_pattern(self, sender, app_data):
        if dpg.get_value("pattern_history") != "history":
            dpg.set_value("input_string", dpg.get_value("pattern_history"))

    def add_plot_callback(self, sender, app_data):
        self.plot_fig_num += 1
        self.plot_data = []
        dpg.add_button(
            label=f"plot{self.plot_fig_num}", callback=self.plot_callback, tag=f"plot_callback{self.plot_fig_num}", before="add_plot")
        with dpg.plot(label=f"figure{self.plot_fig_num}", width=-1, tag=f"plot{self.plot_fig_num}", before="log_header"):
            # optionally create legend
            dpg.add_plot_legend()

            # REQUIRED: create x and y axes
            dpg.add_plot_axis(dpg.mvXAxis, label="x",
                              tag=f"x_axis{self.plot_fig_num}")
            dpg.add_plot_axis(dpg.mvYAxis, label="y",
                              tag=f"y_axis{self.plot_fig_num}")

            # series belong to a y axis
            dpg.add_line_series(list(range(len(
                self.plot_data))), self.plot_data, label="", parent=f"y_axis{self.plot_fig_num}", tag=f"series_tag{self.plot_fig_num}")

    def delete_plot_callback(self, sender, app_data):
        pass

    def callback(self, sender, app_data):
        print("Sender: ", sender)
        print("App Data: ", app_data)
        self.file = list(app_data['selections'].keys())[0]
        self.log_info += (self.file+' process done.\n')

        dpg.set_value('log_text', self.log_info)

    def plot_callback(self, sender, app_data):
        print("Sender: ", sender)
        print("App Data: ", app_data)
        pattern = dpg.get_value("input_string")
        if not (pattern) in self.pattern_history:
            self.pattern_history.append(pattern)
            with open(self.pattern_history_file, 'a+') as fid:
                fid.write(pattern+'\n')
            dpg.delete_item("pattern_history")
            dpg.add_combo(tag="pattern_history", items=self.pattern_history,
                          parent="input_patten_and_history", callback=self.select_pattern)
        self.plot_data = []
        if(self.file == ''):
            print("no file selected")
            return
        decimal_regex = r'(?<=\b{})\d+\.\d+'.format(pattern)
        print(decimal_regex)
        with open(self.file, 'r') as fid:
            line = fid.readline()
            while(line):
                matches = re.findall(decimal_regex, line)
                if(len(matches) != 0):
                    self.plot_data.append(float(matches[0]))
                line = fid.readline()
        dpg.set_value(
            f'series_tag{sender[-1]}', [list(range(len(self.plot_data))), self.plot_data])
        dpg.set_item_label(f'series_tag{sender[-1]}', pattern)
        # dpg.set_axis_limits("y_axis", 0, max(self.plot_data))
        # dpg.set_axis_limits("x_axis", 0, len(self.plot_data))


app = GUI()
