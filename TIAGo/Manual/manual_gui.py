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
