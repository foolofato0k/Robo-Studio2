#!/usr/bin/env python3

import threading
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class PhotoGUI:
    def __init__(self, publish_callback):
        self.publish_callback = publish_callback
        self.root = tk.Tk()
        self.root.title("Live Camera Feed with Contours")
        self.root.geometry("1440x800")
        self.root.configure(bg="black")
        self.root.resizable(False, False)

        # Video capture settings
        self.cap = cv2.VideoCapture(0)
        self.VIDEO_WIDTH = 640
        self.VIDEO_HEIGHT = 480
        self.BORDER_SIZE = 20

        # ----- UI COMPONENTS -----
        # Titles
        tk.Label(self.root, text="Live Camera", font=("Arial",14,"bold"), fg="black", bg="white", width=15).place(x=350, y=31, anchor="center")
        tk.Label(self.root, text="Processed Image", font=("Arial",14,"bold"), fg="black", bg="white", width=15).place(x=1075, y=31, anchor="center")

        # White borders
        tk.Frame(self.root, width=self.VIDEO_WIDTH+self.BORDER_SIZE, height=self.VIDEO_HEIGHT+self.BORDER_SIZE, bg="white").place(x=40, y=45)
        tk.Frame(self.root, width=self.VIDEO_WIDTH+self.BORDER_SIZE, height=self.VIDEO_HEIGHT+self.BORDER_SIZE, bg="white").place(x=740, y=45)

        # Video labels
        self.label_normal = tk.Label(self.root, bg="black", width=self.VIDEO_WIDTH, height=self.VIDEO_HEIGHT)
        self.label_normal.place(x=50, y=55)
        self.label_contours = tk.Label(self.root, bg="black", width=self.VIDEO_WIDTH, height=self.VIDEO_HEIGHT)
        self.label_contours.place(x=750, y=55)

        # Countdown label
        self.countdown_label = tk.Label(self.root, text="", font=("Arial",30,"bold"), fg="red", bg="black")
        self.countdown_label.place_forget()

        # Timer dropdown
        self.selected_timer = tk.StringVar(value="None")
        timer_dropdown = ttk.Combobox(self.root, textvariable=self.selected_timer,
                                      values=["None","3s","5s","10s"], state="readonly",
                                      font=("Arial",12), width=10)
        timer_dropdown.place(x=350, y=700, anchor="center")
        tk.Label(self.root, text="Countdown Timer", font=("Arial",14,"bold"), fg="red", bg="black").place(x=350, y=675, anchor="center")

        # Style dropdown
        self.selected_style = tk.StringVar(value="None")
        style_dropdown = ttk.Combobox(self.root, textvariable=self.selected_style,
                                      values=["None","Wanted","Style 2","Style 3","Style 4"],
                                      state="readonly", font=("Arial",15), width=10)
        style_dropdown.place(x=1075, y=630, anchor="center")
        style_dropdown.bind("<<ComboboxSelected>>", self.update_style_label)
        self.style_label = tk.Label(self.root, text=f"Chosen Style: {self.selected_style.get()}",
                                    font=("Arial",14,"bold"), fg="red", bg="black")
        self.style_label.place(x=1075, y=590, anchor="center")

        # Take Photo button
        self.button_take_photo = tk.Button(self.root, text="Take Photo", font=("Arial",25,"bold"),
                                           fg="red", bg="white", command=self.start_timer)
        self.button_take_photo.place(x=350, y=600, anchor="center")

        # Progress bar and labels
        self.progress_label = tk.Label(self.root, text="", font=("Arial",14,"bold"), fg="white", bg="black")
        self.progress_percentage_label = tk.Label(self.root, text="0%", font=("Arial",14,"bold"), fg="white", bg="black")
        style = ttk.Style()
        style.configure("green.Horizontal.TProgressbar", foreground="green", background="green")
        self.progress_bar = ttk.Progressbar(self.root, orient="horizontal", length=400,
                                            mode="determinate", style="green.Horizontal.TProgressbar")

        # Yes/No buttons
        self.yes_button = tk.Button(self.root, text="Yes", font=("Arial",14,"bold"), fg="white", bg="green",
                                    width=10, command=self.confirm_photo)
        self.no_button = tk.Button(self.root, text="No", font=("Arial",14,"bold"), fg="white", bg="red",
                                   width=10, command=self.retake_photo)
        self.yes_button.place_forget()
        self.no_button.place_forget()

        # Internal state
        self.is_frozen = False
        self.frozen_frame = None

        # Start the frame update loop
        self.update_frame()

    def update_style_label(self, event=None):
        self.style_label.config(text=f"Chosen Style: {self.selected_style.get()}")

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contour_img = np.zeros_like(frame)
        cv2.drawContours(contour_img, contours, -1, (255,255,0), 2)
        return contour_img

    def update_frame(self):
        if not self.is_frozen:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, (self.VIDEO_WIDTH, self.VIDEO_HEIGHT))
                contour_frame = self.process_frame(frame.copy())
                # Display normal
                rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(rgb))
                self.label_normal.imgtk = img
                self.label_normal.config(image=img)
                # Display contours
                rgb_c = cv2.cvtColor(contour_frame, cv2.COLOR_BGR2RGB)
                img_c = ImageTk.PhotoImage(Image.fromarray(rgb_c))
                self.label_contours.imgtk = img_c
                self.label_contours.config(image=img_c)
        self.root.after(10, self.update_frame)

    def start_timer(self):
        self.button_take_photo.config(state=tk.DISABLED)
        timer = self.selected_timer.get()
        if timer == "None":
            self.countdown_label.config(text="Say\nCheese!!!")
            self.countdown_label.place(x=720, y=600, anchor="center")
            self.root.after(2000, lambda: self.countdown_label.place_forget())
            self.root.after(2000, self.take_photo)
        else:
            secs = int(timer.replace("s",""))
            self.countdown(secs)

    def countdown(self, secs):
        if secs > 0:
            self.countdown_label.config(text=str(secs), font=("Arial",50,"bold"))
            self.countdown_label.place(x=720, y=600, anchor="center")
            self.root.after(1000, lambda: self.countdown(secs-1))
        else:
            self.countdown_label.config(text="Say\nCheese!!!")
            self.countdown_label.place(x=720, y=600, anchor="center")
            self.root.after(1000, lambda: self.countdown_label.place_forget())
            self.root.after(1000, self.take_photo)

    def take_photo(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.is_frozen = True
        frame = cv2.resize(frame, (self.VIDEO_WIDTH, self.VIDEO_HEIGHT))
        contour_frame = self.process_frame(frame.copy())
        # Display frozen
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.label_normal.imgtk = img
        self.label_normal.config(image=img)
        rgb_c = cv2.cvtColor(contour_frame, cv2.COLOR_BGR2RGB)
        img_c = ImageTk.PhotoImage(Image.fromarray(rgb_c))
        self.label_contours.imgtk = img_c
        self.label_contours.config(image=img_c)
        # Ask for confirmation
        self.countdown_label.config(text="Are You Happy\nWith Your Photo?", font=("Arial",20,"bold"))
        self.countdown_label.place(x=720, y=600, anchor="center")
        self.yes_button.place(x=650, y=650, anchor="center")
        self.no_button.place(x=790, y=650, anchor="center")

    def confirm_photo(self):
        # Publish confirmation
        self.publish_callback()
        # Clear UI
        self.countdown_label.place_forget()
        self.yes_button.place_forget()
        self.no_button.place_forget()
        self.button_take_photo.config(state=tk.NORMAL)
        # Start progress
        self.show_progress()

    def retake_photo(self):
        self.is_frozen = False
        self.countdown_label.place_forget()
        self.yes_button.place_forget()
        self.no_button.place_forget()
        self.button_take_photo.config(state=tk.NORMAL)

    def show_progress(self):
        self.progress_label.place(x=720, y=660, anchor="center")
        self.progress_bar.place(x=720, y=700, anchor="center")
        self.progress_percentage_label.place(x=720, y=740, anchor="center")
        self.progress_bar['value'] = 0
        self.update_progress(0)

    def update_progress(self, val):
        if val <= 100:
            self.progress_bar['value'] = val
            if val < 33:
                self.progress_label.config(text="Processing Image...", fg="white")
            elif val < 66:
                self.progress_label.config(text="Path Planning...", fg="white")
            else:
                self.progress_label.config(text="Drawing...", fg="white")
            self.progress_percentage_label.config(text=f"{val}%")
            self.root.after(120, self.update_progress, val+1)
        else:
            self.progress_label.config(text="COMPLETED", fg="green")
            self.root.after(500, lambda: self.blink_completed(6))

    def blink_completed(self, times):
        if times > 0:
            text = self.progress_label.cget("text")
            self.progress_label.config(text="" if text=="COMPLETED" else "COMPLETED")
            self.root.after(1000, lambda: self.blink_completed(times-1))
        else:
            self.resume_live()

    def resume_live(self):
        self.is_frozen = False
        self.progress_bar.place_forget()
        self.progress_label.place_forget()
        self.progress_percentage_label.place_forget()
        self.button_take_photo.config(state=tk.NORMAL)

    def run(self):
        self.root.mainloop()
        self.cap.release()

class PhotoPublisherNode(Node):
    def __init__(self):
        super().__init__('photo_publisher')
        self.pub = self.create_publisher(Bool, 'photo_confirmed', 10)
        self.gui = PhotoGUI(self.publish_confirmation)

    def publish_confirmation(self):
        msg = Bool()
        msg.data = True
        self.pub.publish(msg)
        self.get_logger().info('Published photo_confirmed: True')


def main(args=None):
    rclpy.init(args=args)
    node = PhotoPublisherNode()
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    node.gui.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
