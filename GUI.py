import PySimpleGUI as GUI

class Drone_GUI:
    GUI.theme('Dark Amber')
    layout = [[GUI.Text('Drone status', size=(20, 1), font=('Helvetica', 14, 'bold'))],
              [GUI.Text('Drone pose:', font=('Helvetica', 12, 'bold')), GUI.Graph(canvas_size=(50, 50),
                                                      graph_bottom_left=(-50, -50),
                                                      graph_top_right=(50, 50),
                                                      pad=(0, 0), key='online'), GUI.Multiline('x_pos:\t0\t\tx_vel:\t0\t\troll:\t0\ny_pos:\t0\t\ty_vel:\t0\t\tpitch:\t0\nz_pos:\t0\t\tz_vel:\t0\t\tyaw:\t0', size=(70, 4), no_scrollbar=True, focus=False, do_not_clear=False, key='data')],
              [GUI.Text('Desired altitude:'), GUI.InputText(size=(10, 1)), GUI.Button("Submit", size=(5, 1))],
               [GUI.Button("Execute", size=(10, 2)), GUI.Button("Abort", size=(10, 2))]
              ]


    def __init__(self):
        self.window = GUI.Window('Drone control GUI', self.layout, default_element_size=None, auto_size_text=False,
                           no_titlebar=False,
                           finalize=True, auto_size_buttons=False, keep_on_top=True, grab_anywhere=True)

    def getinput(self):
        event, values = self.window.read(timeout=5)
        return event, values

    def update_text(self, key, text):
        self.window[key].update(text)
    def SetLED(self, key, color):
        graph = self.window[key]
        graph.erase()
        graph.draw_circle((0, 0), 20, fill_color=color, line_color=color)

    def close(self):
        self.window.close()


if __name__ == '__main__':
    GUI = Drone_GUI()
    while True:
        events, values = GUI.getinput()
        GUI.SetLED('online', 'red')
        if events == 'Submit':
            print(values[1])
