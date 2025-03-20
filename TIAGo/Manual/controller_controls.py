import tkinter as tk

def con_controls():   
    controls_window = tk.Toplevel()
    controls_window.title("Controller Controls")
    controls_window.geometry("1000x1000")
    controls_window.configure(bg="white")

    label = tk.Label(controls_window, text="Controller Control Instructions", font=("Arial", 14), bg="white")
    label.pack(pady=20)

    instructions = """
    Left Stick - Move & Turn
    Right Stick Move - Look Around
    Right Stick Click - Reset Head
    
    Right Trigger - Close Grip
    Left Trigger - Open Grip
    
    Right Bumper - Offer Hand
    Left Bumper - Retract Hand
    
    DPad - Torso Up/Down
    
    A - Say Hello
    B - Exit Program
    Y - Return to Home
    X - Cancel Movement
    """
    text_widget = tk.Label(controls_window, text=instructions, font=("Arial", 12), bg="white", justify="left")
    text_widget.pack(pady=10)
