from kivy.app import App
from kivy.uix.popup import Popup
from kivy.uix.label import Label
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button


class DraggablePopup(Popup):
    def on_touch_move(self, touch):
        if self.collide_point(*touch.pos):
            self.x += touch.dx
            self.y += touch.dy


class TestApp(App):
    def build(self):
        layout = FloatLayout()
        button = Button(text='Open Popup', size_hint=(
            0.2, 0.1), pos_hint={'x': 0.4, 'y': 0.4})
        button.bind(on_press=self.show_popup)
        layout.add_widget(button)
        return layout

    def show_popup(self, widget):
        popup = DraggablePopup(title='Draggable Popup', content=Label(
            text='This is a draggable popup!'))
        popup.open()


TestApp().run()
