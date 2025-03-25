import cv2
import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import numpy as np

# Window setup
root = tk.Tk()
root.title("GUI")
root.geometry("1440x800")
root.configure(bg="black")
root.resizable(False, False)

# Video feed dimensions
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480

# Webcam setup
cap = cv2.VideoCapture(0)

# Freeze logic
is_frozen = False
frozen_frame = None

# Style dropdown values
style_options = ["None", "Wanted", "Style 2", "Style 3", "Style 4"]
selected_style = tk.StringVar(value="None")

# Style dropdown
style_dropdown = ttk.Combobox(root, textvariable=selected_style, values=style_options, state="readonly", font=("Arial", 15), width=10)
style_dropdown.place(x=1075, y=600, anchor="center")

# Labels for video feeds
label_normal = tk.Label(root, bg="black", width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
label_normal.place(x=50, y=55)

label_contour = tk.Label(root, bg="black", width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
label_contour.place(x=750, y=55)

# Freeze logic
def take_photo():
    global is_frozen, frozen_frame
    is_frozen = True
    ret, frame = cap.read()
    if ret:
        frozen_frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
    root.after(5000, unfreeze)

def unfreeze():
    global is_frozen
    is_frozen = False

# "Take Photo" button
button_capture = tk.Button(root, text="Take Photo", font=("Arial", 25, "bold"), fg="red", bg="white", command=take_photo)
button_capture.place(x=350, y=600, anchor="center")

# Contour processing
def process_frame(frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour_image = np.zeros_like(frame)
    cv2.drawContours(contour_image, contours, -1, (255, 255, 0), 2)
    return contour_image

# Update function for feeds
def update_frame():
    global frozen_frame

    if not is_frozen:
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))
    else:
        frame = frozen_frame

    if frame is not None:
        # Left: Live feed
        live_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        img_live = Image.fromarray(live_rgb)
        imgtk_live = ImageTk.PhotoImage(image=img_live)
        label_normal.imgtk = imgtk_live
        label_normal.config(image=imgtk_live)

        # Right: Contour feed
        contour = process_frame(frame.copy())
        contour_rgb = cv2.cvtColor(contour, cv2.COLOR_BGR2RGB)
        img_contour = Image.fromarray(contour_rgb)
        imgtk_contour = ImageTk.PhotoImage(image=img_contour)
        label_contour.imgtk = imgtk_contour
        label_contour.config(image=imgtk_contour)

    label_normal.after(10, update_frame)

# Start GUI loop
update_frame()
root.mainloop()
cap.release()
cv2.destroyAllWindows()
