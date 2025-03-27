import tkinter as tk
from tkinter import messagebox
import keyboard_controls
import controller_controls
import subprocess

# ========== Globals ========== #

# Holds the currently running subprocess
proc = None
active_process = None  # 'keyboard' or 'controller'

# ========== Control Launchers ========== #

def run_keyboard_control():
    """Start keyboard control script, stopping controller if active."""
    global proc, active_process
    if proc is not None:
        proc.terminate()
        print("Stopped Controller Control")
    proc = subprocess.run("python tiago_keyboard.py", shell=True, text=True)
    active_process = 'keyboard'
    print("Started Keyboard Control")

def run_controller_control():
    """Start controller control script, stopping keyboard if active."""
    global proc, active_process
    if proc is not None:
        proc.terminate()
        print("Stopped Keyboard Control")
    proc = subprocess.run("python tiago_controller.py", shell=True, text=True)
    active_process = 'controller'
    print("Started Controller Control")

# ========== Controls Window Handlers ========== #

def show_controls_window():
    """Show both keyboard and controller control windows."""
    show_keyboard_controls()
    show_controller_controls()

def show_keyboard_controls():
    """Open keyboard control help window."""
    print("Opening Keyboard Controls Window...")
    keyboard_controls.key_controls()

def show_controller_controls():
    """Open controller control help window."""
    print("Opening Controller Controls Window...")
    controller_controls.con_controls()

# ========== GUI Setup ========== #

def create_gui():
    """Create and display the main Tiago Control Panel window."""
    window = tk.Tk()
    window.title("Tiago Control Panel")
    window.geometry("600x400")
    window.configure(bg="white")

    # Button styling options
    button_style = {
        "bg": "#4CAF50",            # Green
        "fg": "white",
        "activebackground": "#45a049",
        "font": ("Arial", 14),
        "width": 20,
        "height": 3,
        "bd": 0,
        "relief": "flat",
        "cursor": "hand2"
    }

    # Button container
    button_frame = tk.Frame(window, bg="white")
    button_frame.pack(expand=True)

    # Keyboard Control Button
    tk.Button(
        button_frame,
        text="Run Keyboard Control",
        command=run_keyboard_control,
        **button_style
    ).pack(side="left", padx=20, pady=10)

    # Controller Control Button
    tk.Button(
        button_frame,
        text="Run Controller Control",
        command=run_controller_control,
        **button_style
    ).pack(side="left", padx=20, pady=10)

    # Show Controls Button
    tk.Button(
        window,
        text="Show Controls",
        command=show_controls_window,
        bg="#2196F3",
        fg="white",
        activebackground="#1976D2",
        font=("Arial", 14),
        width=20,
        height=2,
        bd=0,
        relief="flat",
        cursor="hand2"
    ).pack(pady=40)

    # Start the GUI event loop
    window.mainloop()

# ========== Main Execution ========== #

if __name__ == "__main__":
    create_gui()
