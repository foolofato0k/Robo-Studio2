import cv2
import tkinter as tk
from tkinter import ttk  # For dropdown and progress bar
from PIL import Image, ImageTk
import numpy as np

# Create the main window
root = tk.Tk()
root.title("Live Camera Feed with Contours")
root.geometry("1440x800")  # Adjusted window size for proper spacing
root.configure(bg="black")  # Set background to black
root.resizable(False, False)  # Make the window non-resizable

# Open the webcam (0 = default camera)
cap = cv2.VideoCapture(0)

# Set fixed dimensions for the video feeds
VIDEO_WIDTH = 640
VIDEO_HEIGHT = 480
BORDER_SIZE = 20  # White border thickness

# Style for dropdowns
style = ttk.Style()
style.configure("TCombobox", font=("Arial", 14))  # Increase font size

# Title labels above video feeds
title_live = tk.Label(root, text="Live Camera", font=("Arial", 14, "bold"), fg="black", bg="white", width=15)
title_live.place(x=350, y=31, anchor="center")

title_processed = tk.Label(root, text="Processed Image", font=("Arial", 14, "bold"), fg="black", bg="white", width=15)
title_processed.place(x=1075, y=31, anchor="center")

# Create white border frames for both feeds
frame_border_live = tk.Frame(root, width=VIDEO_WIDTH + BORDER_SIZE, height=VIDEO_HEIGHT + BORDER_SIZE, bg="white")
frame_border_live.place(x=40, y=45)  # Left feed with border

frame_border_processed = tk.Frame(root, width=VIDEO_WIDTH + BORDER_SIZE, height=VIDEO_HEIGHT + BORDER_SIZE, bg="white")
frame_border_processed.place(x=740, y=45)  # Right feed with border

# Labels for displaying the camera feeds
label_normal = tk.Label(root, bg="black", width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
label_normal.place(x=50, y=55)  # Left: Normal feed

label_contours = tk.Label(root, bg="black", width=VIDEO_WIDTH, height=VIDEO_HEIGHT)
label_contours.place(x=750, y=55)  # Right: Contour feed

# Timer countdown label (Hidden initially)
countdown_label = tk.Label(root, text="", font=("Arial", 30, "bold"), fg="red", bg="black")
countdown_label.place(x=640, y=550, anchor="center")
countdown_label.place_forget()  # Hide initially

# Create progress bar labels
progress_label = tk.Label(root, text="", font=("Arial", 14, "bold"), fg="white", bg="black")  # Text above progress bar
progress_percentage_label = tk.Label(root, text="0%", font=("Arial", 14, "bold"), fg="white", bg="black")  # Percentage below bar

# Progress bar itself
progress_bar = ttk.Progressbar(root, orient="horizontal", length=400, mode="determinate", style="green.Horizontal.TProgressbar")

# Style for green progress bar
style = ttk.Style()
style.configure("green.Horizontal.TProgressbar", foreground="green", background="green")

# Dropdown for countdown timer
timer_options = ["None", "3s", "5s", "10s"]
selected_timer = tk.StringVar(value="None")
# Countdown Timer Dropdown (Bigger size)
timer_dropdown = ttk.Combobox(root, textvariable=selected_timer, values=timer_options, state="readonly", font=("Arial", 12), width=10)
timer_dropdown.place(x=350, y=700, anchor="center")

# Dropdown for style selection
style_options = ["None", "Wanted", "Style 2", "Style 3", "Style 4"] 
selected_style = tk.StringVar(value="None")  # Default value is "None"

# Style text display
style_label = tk.Label(root, text="Chosen Style: ", font=("Arial", 14, "bold"), fg="red", bg="black")
style_label.place(x=1075, y=590, anchor="center")

# Function to update the chosen style label dynamically
def update_style_label(event=None):
    """Updates the 'Chosen Style' label based on dropdown selection."""
    style_label.config(text=f"Chosen Style: {selected_style.get()}")

# Style Selection Dropdown (Bigger size)
style_dropdown = ttk.Combobox(root, textvariable=selected_style, values=style_options, state="readonly", font=("Arial", 15), width=10)
style_dropdown.place(x=1075, y=630, anchor="center")
style_dropdown.bind("<<ComboboxSelected>>", update_style_label)  # Trigger label update on selection

# Style text display (Default set to "None")
style_label = tk.Label(root, text=f"Chosen Style: {selected_style.get()}", font=("Arial", 14, "bold"), fg="red", bg="black")
style_label.place(x=1075, y=590, anchor="center")

# Countdown text display
countdown_title_label = tk.Label(root, text="Countdown Timer", font=("Arial", 14, "bold"), fg="red", bg="black")
countdown_title_label.place(x=350, y=675, anchor="center")

# Take Photo Button
button_take_photo = tk.Button(root, text="Take Photo", font=("Arial", 25, "bold"), fg="red", bg="white", command=lambda: start_timer())
button_take_photo.place(x=350, y=600, anchor="center")


# Variable to track if we're showing a frozen frame
is_frozen = False
frozen_frame = None  # Store the captured frame

def process_frame(frame):
    """Applies contour detection on the frame."""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Draw contours on a black image
    contour_image = np.zeros_like(frame)
    cv2.drawContours(contour_image, contours, -1, (255, 255, 0), 2)  # Cyan contours

    return contour_image

def update_frame():
    """Capture frame from webcam and update the labels."""
    global is_frozen, frozen_frame
    if not is_frozen:
        ret, frame = cap.read()
        if ret:
            frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))  # Ensure consistent size
            contour_frame = process_frame(frame.copy())

            # Convert normal feed to Tkinter image
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            img_normal = Image.fromarray(frame)
            imgtk_normal = ImageTk.PhotoImage(image=img_normal)
            
            # Convert contour feed to Tkinter image
            contour_frame = cv2.cvtColor(contour_frame, cv2.COLOR_BGR2RGB)
            img_contour = Image.fromarray(contour_frame)
            imgtk_contour = ImageTk.PhotoImage(image=img_contour)

            # Update labels with the images
            label_normal.imgtk = imgtk_normal
            label_normal.config(image=imgtk_normal)

            label_contours.imgtk = imgtk_contour
            label_contours.config(image=imgtk_contour)

    label_normal.after(10, update_frame)

def start_timer():
    """Starts countdown timer if needed, otherwise shows 'Say Cheese' and takes photo."""
    global is_frozen
    button_take_photo.config(state=tk.DISABLED)  # Disable button to prevent multiple presses

    timer_value = selected_timer.get()

    if timer_value == "None":
        # No countdown, just display "Say" and "Cheese!!!" before taking the photo
        countdown_label.config(text="Say\nCheese!!!", font=("Arial", 30, "bold"))
        countdown_label.place(x=720, y=600, anchor="center")  # Show label in the center

        # Hide text and take photo after 2 seconds
        root.after(2000, lambda: countdown_label.place_forget())
        root.after(2000, take_photo)
    else:
        # Extract the countdown time (convert "3s" -> 3)
        countdown_seconds = int(timer_value.replace("s", ""))  
        countdown(countdown_seconds)

def countdown(seconds):
    """Displays countdown and calls take_photo() when finished."""
    if seconds > 0:
        countdown_label.config(text=str(seconds), font=("Arial", 50, "bold"))  # Increase font size
        countdown_label.place(x=720, y=600, anchor="center")  # Show countdown
        root.after(1000, countdown, seconds - 1)
    else:
        # After countdown finishes or if timer is "None", display "Say" and "Cheese!!!" below it
        countdown_label.config(text="Say\nCheese!!!", font=("Arial", 30, "bold"))
        countdown_label.place(x=720, y=600, anchor="center")  # Reposition if needed

        # Wait 1 seconds, then hide the text and take the photo
        root.after(1000, lambda: countdown_label.place_forget())
        root.after(1000, take_photo)

def take_photo():
    """Capture a frame, freeze the images, and ask for confirmation before progress bar."""
    global is_frozen, frozen_frame
    ret, frame = cap.read()
    if ret:
        is_frozen = True
        frozen_frame = cv2.resize(frame, (VIDEO_WIDTH, VIDEO_HEIGHT))

        # Process contour detection on the frozen frame
        contour_frame = process_frame(frozen_frame.copy())

        # Convert to Tkinter images
        frozen_frame = cv2.cvtColor(frozen_frame, cv2.COLOR_BGR2RGB)
        img_normal = Image.fromarray(frozen_frame)
        imgtk_normal = ImageTk.PhotoImage(image=img_normal)

        contour_frame = cv2.cvtColor(contour_frame, cv2.COLOR_BGR2RGB)
        img_contour = Image.fromarray(contour_frame)
        imgtk_contour = ImageTk.PhotoImage(image=img_contour)

        # Show frozen images
        label_normal.imgtk = imgtk_normal
        label_normal.config(image=imgtk_normal)

        label_contours.imgtk = imgtk_contour
        label_contours.config(image=imgtk_contour)

        # Ask for confirmation before showing progress bar
        countdown_label.config(text="Are You Happy\n With Your Photo?", font=("Arial", 20, "bold"))
        countdown_label.place(x=720, y=600, anchor="center")

        # Show the Yes/No buttons
        yes_button.place(x=650, y=650, anchor="center")
        no_button.place(x=790, y=650, anchor="center")

def confirm_photo():
    """User confirms they are happy with the photo -> Show progress bar."""
    # Hide confirmation message and buttons
    countdown_label.place_forget()
    yes_button.place_forget()
    no_button.place_forget()

    # Show progress bar with labels
    progress_label.place(x=720, y=660, anchor="center")  # Text above bar
    progress_bar.place(x=720, y=700, anchor="center")  # Progress bar
    progress_percentage_label.place(x=720, y=740, anchor="center")  # Percentage below bar

    progress_bar["value"] = 0
    update_progress(0)

def retake_photo():
    """User is not happy with the photo -> Reset and return to live feed."""
    global is_frozen
    is_frozen = False  # Resume live feed

    # Hide confirmation message and buttons
    countdown_label.place_forget()
    yes_button.place_forget()
    no_button.place_forget()

    button_take_photo.config(state=tk.NORMAL)  # Re-enable "Take Photo" button

def blink_completed_text(times):
    """Blink 'COMPLETED' text every second for 6 seconds."""
    if times > 0:
        # Toggle visibility by checking current text
        if progress_label.cget("text") == "COMPLETED":
            progress_label.config(text="")  # Hide text
        else:
            progress_label.config(text="COMPLETED")  # Show text

        # Call itself every second, reducing blink count
        root.after(1000, lambda: blink_completed_text(times - 1))
    else:
        resume_live_feed()  # After blinking for 6 seconds, hide progress bar
        
def update_progress(value):
    """Update the progress bar over 12 seconds with different labels."""
    if value <= 100:
        progress_bar["value"] = value

        # Change label text based on progress duration
        if value < 33:  # First 4 seconds
            progress_label.config(text="Processing Image...", fg="white")
        elif value < 66:  # Next 4 seconds
            progress_label.config(text="Path Planning...", fg="white")
        else:  # Last 4 seconds
            progress_label.config(text="Drawing...", fg="white")

        # Update percentage label below progress bar
        progress_percentage_label.config(text=f"{value}%")

        # Increase progress in 1% steps every 120ms (12 seconds total)
        root.after(120, update_progress, value + 1)
    
    else:
        # Once the progress reaches 100%, show "Completed" and trigger blinking effect
        progress_label.config(text="COMPLETED", fg="green")
        root.after(500, lambda: blink_completed_text(6))  # Blink for 6 seconds before hiding

def resume_live_feed():
    """Unfreeze the live feed and hide progress elements."""
    global is_frozen
    is_frozen = False
    progress_bar.place_forget()
    progress_label.place_forget()
    progress_percentage_label.place_forget()  # Hide percentage when done

    button_take_photo.config(state=tk.NORMAL)  # Re-enable "Take Photo" button

# Create Yes/No buttons for photo confirmation (initially hidden)
yes_button = tk.Button(root, text="Yes", font=("Arial", 14, "bold"), fg="white", bg="green", width=10, command=confirm_photo)
yes_button.place_forget()

no_button = tk.Button(root, text="No", font=("Arial", 14, "bold"), fg="white", bg="red", width=10, command=retake_photo)
no_button.place_forget()

# Start updating frames
update_frame()

# Run Tkinter loop
root.mainloop()

cap.release()
cv2.destroyAllWindows()

