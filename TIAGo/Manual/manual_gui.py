import tkinter as tk
from tkinter import messagebox
import keyboard_controls
import controller_controls
import subprocess

# Globals to hold the process objects
keyboard_process = None
controller_process = None

# Track which process is active
active_process = None  # 'keyboard', 'controller'

proc = None

def run_keyboard_control():
    global active_process, proc
    if proc is not None:
        proc.terminate()
        print("Stopped Controller Control")
    proc = subprocess.run("python tiago_keyboard.py", shell=True, text=True)
    active_process = 'keyboard'
    print("Started Keyboard Control")

def run_controller_control():
    global active_process, proc
    if proc is not None:
        proc.terminate()
        print("Stopped Keyboard Control")
    proc = subprocess.run("python tiago_controller.py", shell=True, text=True)
    active_process = 'controller'
    print("Started Controller Control")

def show_controls_window():
    show_keyboard_controls()
    show_controller_controls()

def show_keyboard_controls():
    print("Opening Keyboard Controls Window...")
    keyboard_controls.key_controls()

def show_controller_controls():
    print("Opening Controller Controls Window...")
    controller_controls.con_controls()
def create_gui():
    # Create the main window
    window = tk.Tk()
    window.title("Tiago Control Panel")
    window.geometry("600x400") 
    window.configure(bg="white")

    # Button styling
    button_style = {
        "bg": "#4CAF50",          # Nice green color
        "fg": "white",            # White text
        "activebackground": "#45a049",  # Darker green when pressed
        "font": ("Arial", 14),    # Bigger font
        "width": 20,              # Wider button
        "height": 3,              # Taller button
        "bd": 0,                  # No border
        "relief": "flat",         # Flat look
        "cursor": "hand2"         # Pointer cursor on hover
    }

    # Create a frame to hold the two control buttons side by side
    button_frame = tk.Frame(window, bg="white")
    button_frame.pack(expand=True)

    # Button 1 - Run Keyboard Control
    button1 = tk.Button(
        button_frame, 
        text="Run Keyboard Control", 
        command=lambda: run_keyboard_control(),
        **button_style
    )
    button1.pack(side="left", padx=20, pady=10)

    # Button 2 - Run Controller Control
    button2 = tk.Button(
        button_frame, 
        text="Run Controller Control", 
        command=lambda: run_controller_control(),
        **button_style
    )
    button2.pack(side="left", padx=20, pady=10)

    # Show Controls Button
    show_controls_button = tk.Button(
        window,
        text="Show Controls",
        command=show_controls_window,
        bg="#2196F3",                # Blue color for difference
        fg="white",
        activebackground="#1976D2",  # Darker blue on click
        font=("Arial", 14),
        width=20,
        height=2,
        bd=0,
        relief="flat",
        cursor="hand2"
    )
    show_controls_button.pack(pady=40)

    # Run the window loop
    window.mainloop()

if __name__ == "__main__":
    create_gui()
