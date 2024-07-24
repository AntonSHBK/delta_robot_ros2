#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import tkinter as tk
from tkinter import ttk

class GuiCommandSender(Node):
    def __init__(self):
        super().__init__('gui_command_sender')

        self.publisher_ = self.create_publisher(Point, 'target_position', 10)

        # Настройка GUI
        self.root = tk.Tk()
        self.root.title("Delta Robot Command Sender")

        # Поля ввода для координат
        self.label_x = ttk.Label(self.root, text="X:")
        self.label_x.grid(column=0, row=0, padx=10, pady=5)
        self.entry_x = ttk.Entry(self.root)
        self.entry_x.grid(column=1, row=0, padx=10, pady=5)

        self.label_y = ttk.Label(self.root, text="Y:")
        self.label_y.grid(column=0, row=1, padx=10, pady=5)
        self.entry_y = ttk.Entry(self.root)
        self.entry_y.grid(column=1, row=1, padx=10, pady=5)

        self.label_z = ttk.Label(self.root, text="Z:")
        self.label_z.grid(column=0, row=2, padx=10, pady=5)
        self.entry_z = ttk.Entry(self.root)
        self.entry_z.grid(column=1, row=2, padx=10, pady=5)

        # Кнопка отправки
        self.send_button = ttk.Button(self.root, text="Отправить", command=self.send_command)
        self.send_button.grid(column=0, row=3, columnspan=2, padx=10, pady=10)

        self.get_logger().info('GUI Command Sender has been started.')

    def send_command(self):
        x = float(self.entry_x.get())
        y = float(self.entry_y.get())
        z = float(self.entry_z.get())

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent command: x={x}, y={y}, z={z}')

    def run(self):
        self.root.mainloop()

def main(args=None):
    rclpy.init(args=args)
    node = GuiCommandSender()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
