import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from tkinter import Tk, Canvas

class JoyButton:
    def __init__(self, parent_canvas, left_space, v_space, height, i):
        self.parent_canvas = parent_canvas
        lf = left_space + 15
        t = v_space + (height + v_space) * i
        self.parent_canvas.create_text(left_space, t + height / 2, text=str(i))
        self.circle_obj = self.parent_canvas.create_oval(lf, t, lf + height, t + height, width=2, fill="white")

    def update_value(self, value):
        color = "#FF0000" if value > 0 else "#FFFFFF"
        self.parent_canvas.itemconfigure(self.circle_obj, fill=color)

class JoyAxis:
    def __init__(self, parent_canvas, left_space, v_space, height, width, i):
        self.parent_canvas = parent_canvas
        self.left_space = left_space
        self.v_space = v_space
        self.height = height
        self.width = width
        self.i = i

        lf = left_space + 60
        t = v_space + (height + v_space) * i
        self.parent_canvas.create_text(left_space + 50, t + height / 2, text=str(i))

        self.fill_obj = self.parent_canvas.create_rectangle(lf, t, lf + width, t + height, width=0, fill="green")
        self.outline_obj = self.parent_canvas.create_rectangle(lf, t, lf + width, t + height, width=2, outline="black")

        self.val_txt = self.parent_canvas.create_text(left_space + 60 + width + 30, t + height / 2, text=str(i))

    def update_value(self, value):
        lf = self.left_space + 60
        t = self.v_space + (self.height + self.v_space) * self.i

        ww = self.width * (value + 1) / 2
        self.parent_canvas.coords(self.fill_obj, lf, t, lf + ww, t + self.height)
        self.parent_canvas.itemconfigure(self.val_txt, text=f'{value:.3f}')

class JoyTester(Node):
    def __init__(self):
        super().__init__('joy_tester')
        self.get_logger().info('Testing Joystick...')
        self.subscription = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        self.buttons = []
        self.axes = []
        self.initialised = False

        # Initialize Tkinter
        self.tk = Tk()
        self.canvas = Canvas(self.tk, width=800, height=480)
        self.tk.title("Joystick Test")
        self.canvas.pack(anchor='nw')

        # Initialize the GUI
        self.left_space = 10
        self.height = 25
        self.width = 80
        self.v_space = 5

    def joy_callback(self, joy_msg):
        if not self.initialised:
            for i in range(len(joy_msg.buttons)):
                self.buttons.append(JoyButton(self.canvas, self.left_space, self.v_space, self.height, i))
            for i in range(len(joy_msg.axes)):
                self.axes.append(JoyAxis(self.canvas, self.left_space, self.v_space, self.height, self.width, i))
            self.initialised = True
            self.get_logger().info('Joystick initialized. Buttons and axes added.')

        # Update Values
        for i, val in enumerate(joy_msg.buttons):
            self.buttons[i].update_value(val)

        for i, val in enumerate(joy_msg.axes):
            self.axes[i].update_value(val)

        # Refresh the GUI
        self.canvas.update()

    def run(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            self.tk.update_idletasks()
            self.tk.update()

def main(args=None):
    rclpy.init(args=args)
    joy_tester = JoyTester()
    try:
        joy_tester.run()
    finally:
        joy_tester.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
