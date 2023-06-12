import re
import dearpygui.dearpygui as dpg
import os
class GUI():
    def __init__(self) -> None:
        dpg.create_context()
        dpg.create_viewport(width=600, height=400)
        dpg.setup_dearpygui()
        with dpg.window(tag="Primary Window"):
            with dpg.group("")