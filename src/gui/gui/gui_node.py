#!/usr/bin/env python3

import os
import platform
import time
import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ament_index_python.packages import get_package_share_directory

class PhotoGUI:
    def __init__(self, publish_callback):
        self.publish_callback = publish_callback
        self.root = tk.Tk()
        self.root.title("Live Camera Feed with Contours")
        self.root.geometry("1440x800")
        self.root.configure(bg="black")
        self.root.resizable(False, False)

        # load Haar cascade
        cascade_path = os.path.join(
            get_package_share_directory('gui'),
            'haarcascade_frontalface_default.xml'
        )
        self.face_cascade = cv2.CascadeClassifier(cascade_path)
        if self.face_cascade.empty():
            raise RuntimeError(f"Couldn’t load cascade at {cascade_path}")

        # Open camera
        self.cap = self.find_working_camera()
        if self.cap is None:
            raise RuntimeError("No accessible webcam found.")

        # Preview sizes
        self.VIDEO_WIDTH = 640
        self.VIDEO_HEIGHT = 480
        self.BORDER_SIZE = 20

        # UI: labels & frames
        tk.Label(self.root, text="Live Camera",
                 font=("Arial",14,"bold"),
                 fg="black", bg="white", width=15).place(x=350, y=31, anchor="center")
        tk.Label(self.root, text="Processed Image",
                 font=("Arial",14,"bold"),
                 fg="black", bg="white", width=15).place(x=1075, y=31, anchor="center")
        tk.Frame(self.root,
                 width=self.VIDEO_WIDTH + self.BORDER_SIZE,
                 height=self.VIDEO_HEIGHT + self.BORDER_SIZE,
                 bg="white").place(x=40, y=45)
        tk.Frame(self.root,
                 width=self.VIDEO_WIDTH + self.BORDER_SIZE,
                 height=self.VIDEO_HEIGHT + self.BORDER_SIZE,
                 bg="white").place(x=740, y=45)
        self.label_normal = tk.Label(self.root, bg="black",
                                     width=self.VIDEO_WIDTH,
                                     height=self.VIDEO_HEIGHT)
        self.label_normal.place(x=50, y=55)
        self.label_contours = tk.Label(self.root, bg="black",
                                       width=self.VIDEO_WIDTH,
                                       height=self.VIDEO_HEIGHT)
        self.label_contours.place(x=750, y=55)

        # Countdown label
        self.countdown_label = tk.Label(self.root, text="",
                                        font=("Arial",30,"bold"),
                                        fg="red", bg="black")

        # Timer dropdown
        self.selected_timer = tk.StringVar(value="None")
        ttk.Combobox(self.root,
                     textvariable=self.selected_timer,
                     values=["None","3s","5s","10s"],
                     state="readonly",
                     font=("Arial",12), width=10).place(x=350, y=700, anchor="center")
        tk.Label(self.root, text="Countdown Timer",
                 font=("Arial",14,"bold"), fg="red", bg="black").place(x=350, y=675, anchor="center")

        # Style dropdown
        self.selected_style = tk.StringVar(value="None")
        style_dropdown = ttk.Combobox(self.root,
                                      textvariable=self.selected_style,
                                      values=["None","Wanted"],
                                      state="readonly",
                                      font=("Arial",15), width=10)
        style_dropdown.place(x=1075, y=630, anchor="center")
        style_dropdown.bind("<<ComboboxSelected>>", self.update_style_label)
        self.style_label = tk.Label(self.root,
                                    text=f"Chosen Style: {self.selected_style.get()}",
                                    font=("Arial",14,"bold"), fg="red", bg="black")
        self.style_label.place(x=1075, y=590, anchor="center")

        # Take photo button
        self.button_take_photo = tk.Button(self.root,
                                           text="Take Photo",
                                           font=("Arial",25,"bold"),
                                           fg="red", bg="white",
                                           command=self.start_timer)
        self.button_take_photo.place(x=350, y=600, anchor="center")

        # Progress bar widgets (hidden until needed)
        self.progress_label = tk.Label(self.root, text="", font=("Arial",14,"bold"), fg="white", bg="black")
        self.progress_percentage_label = tk.Label(self.root, text="0%", font=("Arial",14,"bold"), fg="white", bg="black")
        style = ttk.Style()
        style.configure("green.Horizontal.TProgressbar", foreground="green", background="green")
        self.progress_bar = ttk.Progressbar(self.root,
                                            orient="horizontal",
                                            length=400, mode="determinate",
                                            style="green.Horizontal.TProgressbar")

        # Yes/No buttons
        self.yes_button = tk.Button(self.root, text="Yes", font=("Arial",14,"bold"),
                                    fg="white", bg="green", width=10, command=self.confirm_photo)
        self.no_button = tk.Button(self.root, text="No", font=("Arial",14,"bold"),
                                   fg="white", bg="red", width=10, command=self.retake_photo)

        # State
        self.is_frozen = False
        self.frozen_frame = None

        # Start live preview
        self.update_frame()

    def find_working_camera(self):
        is_linux = platform.system() == 'Linux'
        for i in range(5):
            dev = f"/dev/video{i}" if is_linux else i
            cap = (cv2.VideoCapture(dev, cv2.CAP_V4L2) if is_linux else cv2.VideoCapture(dev))
            if is_linux:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            time.sleep(1)
            if cap.isOpened():
                ret, _ = cap.read()
                if ret:
                    return cap
            cap.release()
        return None

    def update_style_label(self, event=None):
        self.style_label.config(text=f"Chosen Style: {self.selected_style.get()}")

    def process_frame(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5,5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        out = np.zeros_like(frame)
        cv2.drawContours(out, contours, -1, (255,255,0), 2)
        return out

    def update_frame(self):
        if not self.is_frozen:
            ret, frame = self.cap.read()
            if ret:
                frame_resized = cv2.resize(frame, (self.VIDEO_WIDTH, self.VIDEO_HEIGHT))
                contour = self.process_frame(frame_resized.copy())
                rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(Image.fromarray(rgb))
                self.label_normal.imgtk = img
                self.label_normal.config(image=img)
                rgb_c = cv2.cvtColor(contour, cv2.COLOR_BGR2RGB)
                img_c = ImageTk.PhotoImage(Image.fromarray(rgb_c))
                self.label_contours.imgtk = img_c
                self.label_contours.config(image=img_c)
        self.root.after(10, self.update_frame)

    def start_timer(self):
        self.button_take_photo.config(state=tk.DISABLED)
        t = self.selected_timer.get()
        if t == "None":
            self.countdown_label.config(text="Say\nCheese!!!")
            self.countdown_label.place(x=720, y=600, anchor="center")
            self.root.after(2000, self.countdown_label.place_forget)
            self.root.after(2000, self.take_photo)
        else:
            secs = int(t.replace("s",""))
            self.countdown(secs)

    def countdown(self, secs):
        if secs > 0:
            self.countdown_label.config(text=str(secs), font=("Arial",50,"bold"))
            self.countdown_label.place(x=720, y=600, anchor="center")
            self.root.after(1000, lambda: self.countdown(secs-1))
        else:
            self.countdown_label.config(text="Say\nCheese!!!")
            self.countdown_label.place(x=720, y=600, anchor="center")
            self.root.after(1000, self.countdown_label.place_forget)
            self.root.after(1000, self.take_photo)

    def take_photo(self):
        ret, frame = self.cap.read()
        if not ret:
            return
        self.is_frozen = True
        frame_resized = cv2.resize(frame, (self.VIDEO_WIDTH, self.VIDEO_HEIGHT))
        self.frozen_frame = frame_resized.copy()
        contour = self.process_frame(frame_resized.copy())
        rgb = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(Image.fromarray(rgb))
        self.label_normal.imgtk = img
        self.label_normal.config(image=img)
        rgb_c = cv2.cvtColor(contour, cv2.COLOR_BGR2RGB)
        img_c = ImageTk.PhotoImage(Image.fromarray(rgb_c))
        self.label_contours.imgtk = img_c
        self.label_contours.config(image=img_c)
        self.countdown_label.config(text="Are You Happy\nWith Your Photo?", font=("Arial",20,"bold"))
        self.countdown_label.place(x=720, y=600, anchor="center")
        self.yes_button.place(x=650, y=650, anchor="center")
        self.no_button.place(x=790, y=650, anchor="center")

    def confirm_photo(self):
        self.countdown_label.place_forget()
        self.yes_button.place_forget()
        self.no_button.place_forget()
        self.publish_callback()
        # save out
        script_dir = os.path.dirname(__file__)
        save_dir = os.path.join(script_dir,'capture_image')
        os.makedirs(save_dir, exist_ok=True)
        cv2.imwrite(os.path.join(save_dir, 'webcam_img.jpg'), self.frozen_frame)
        print(f"[INFO] Photo captured and saved")
        # kick off pipeline + progress
        style = self.selected_style.get().strip().lower()
        self.display_pipeline(self.frozen_frame, style=style)

    def retake_photo(self):
        self.is_frozen = False
        self.countdown_label.place_forget()
        self.yes_button.place_forget()
        self.no_button.place_forget()
        self.button_take_photo.config(state=tk.NORMAL)

    # ─── NEW SMOOTH-PROGRESS METHODS ───────────────────────────────────────────

    def show_progress(self, total_stages, delay_ms):
        total_time = total_stages * delay_ms
        interval = 50  # ms between bar-updates
        steps = total_time / interval
        self._prog_inc = 100.0 / steps

        # init bar + labels
        self.progress_label.config(text="Processing Image...")
        self.progress_label.place(x=720, y=660, anchor="center")
        self.progress_bar['value'] = 0
        self.progress_bar.place(x=720, y=700, anchor="center")
        self.progress_percentage_label.config(text="0%")
        self.progress_percentage_label.place(x=720, y=740, anchor="center")

        # start the loop
        self._update_progress(interval)

    def _update_progress(self, interval):
        new_val = self.progress_bar['value'] + self._prog_inc
        if new_val < 100:
            self.progress_bar['value'] = new_val
            self.progress_percentage_label.config(text=f"{int(new_val)}%")
            self.root.after(interval, self._update_progress, interval)
        else:
            # finalize at 100%
            self.progress_bar['value'] = 100
            self.progress_percentage_label.config(text="100%")
            self.progress_label.config(text="Completed", fg="green")
            # wait 1 s, then blink 6 times
            self.root.after(1000, lambda: self.blink_completed(6))
            
            
    def blink_completed(self, times):
        if times > 0:
            current = self.progress_label.cget("text")
            if current == "Completed":
                self.progress_label.config(text="")             # leave fg as-is, it won’t show
            else:
                self.progress_label.config(text="Completed", fg="green")
            self.root.after(1000, self.blink_completed, times-1)
        else:
            self.resume_live()


    # ──────────────────────────────────────────────────────────────────────────

    def display_pipeline(self, frame, delay_ms=1500, style="default"):
        # detect face for ROI
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = self.face_cascade.detectMultiScale(gray, 1.3, 5)
        if len(faces):
            x,y,w,h = faces[0]
        else:
            x,y,w,h = 0,0,frame.shape[1],frame.shape[0]
        self._roi_w, self._roi_h = w,h

        # build stages
        stages = []
        stages.append(frame.copy())
        stages.append(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR))
        bb = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(bb, (x,y),(x+w,y+h), (0,255,0), 2)
        stages.append(bb)
        stages.append(cv2.cvtColor(gray[y:y+h, x:x+w], cv2.COLOR_GRAY2BGR))
        blur = cv2.GaussianBlur(gray[y:y+h, x:x+w], (5,5), 0)
        edges = cv2.Canny(blur, 50, 150)
        cont = np.zeros_like(stages[-1])
        cts,_ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(cont, cts, -1, (255,255,0), 2)
        stages.append(cont)

        # center & convert for Tk
        self._pipeline = [
            self.center_image(cv2.cvtColor(s, cv2.COLOR_BGR2RGB))
            for s in stages
        ]
        total = len(self._pipeline)
        
        self._pipeline_idx = 0

        # start smooth progress
        self.show_progress(total, delay_ms)

        # then run your existing stage‐by‐stage display
        self._show_next_stage(delay_ms, style, total)

    def _show_next_stage(self, delay_ms, style, total):
        if getattr(self, "_pipeline_idx", 0) < total:
            idx = self._pipeline_idx
            img = self._pipeline[idx]
            imgtk = ImageTk.PhotoImage(Image.fromarray(img))
            self.label_contours.imgtk = imgtk
            self.label_contours.config(image=imgtk)
            self._pipeline_idx += 1
            self.root.after(delay_ms, lambda: self._show_next_stage(delay_ms, style, total))
        else:
            # final style overlay if needed
            if style == "wanted":
                bgr = cv2.cvtColor(self._pipeline[-1], cv2.COLOR_RGB2BGR)
                x_off = (self.VIDEO_WIDTH - self._roi_w)//2
                y_off = (self.VIDEO_HEIGHT - self._roi_h)//2
                cv2.rectangle(bgr, (x_off, y_off),
                              (x_off+self._roi_w, y_off+self._roi_h),
                              (255,255,0), 2)
                font = cv2.FONT_HERSHEY_SIMPLEX
                texts = [
                    ("WANTED", 2, 3, y_off-10),
                    ("DEAD OR ALIVE", 1, 2, y_off+self._roi_h+40),
                    ("$10", 1, 2, y_off+self._roi_h+70),
                ]
                for txt, scale, th, ypos in texts:
                    (tw,_),_ = cv2.getTextSize(txt, font, scale, th)
                    x_txt = (self.VIDEO_WIDTH - tw)//2
                    cv2.putText(bgr, txt, (x_txt, ypos), font, scale, (255,255,0), th)
                rgb_txt = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
                imgtk = ImageTk.PhotoImage(Image.fromarray(rgb_txt))
                self.label_contours.imgtk = imgtk
                self.label_contours.config(image=imgtk)

    def center_image(self, img_array):
        h, w = img_array.shape[:2]
        canvas = np.zeros((self.VIDEO_HEIGHT, self.VIDEO_WIDTH, 3), dtype=np.uint8)
        x_off = (self.VIDEO_WIDTH - w)//2
        y_off = (self.VIDEO_HEIGHT - h)//2
        canvas[y_off:y_off+h, x_off:x_off+w] = img_array
        return canvas

    def resume_live(self):
        self.is_frozen = False
        self.progress_label.place_forget()
        self.progress_bar.place_forget()
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
    node.gui.run()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
