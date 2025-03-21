import tkinter as tk

def key_controls():   
    controls_window = tk.Toplevel()
    controls_window.title("Keyboard Controls")
    controls_window.geometry("1000x1000")
    controls_window.configure(bg="white")

    label = tk.Label(controls_window, text="Keyboard Control Instructions", font=("Arial", 14), bg="white")
    label.pack(pady=20)

    instructions = """
    
    You must hit enter to send keyboard command. CASE SENSITIVE COMMANDS.
    For the following 4 controls, press the key more to move further.
    
    w - Move Forward
    s - Move Backward
    a - Turn Left
    d - Turn Right
    i - Look Up
    k - Look Down
    j - Look Left
    l - Look Right
    
    Only send one key for the following
    
    W - Raise Torso
    S - Lower Torso
    R - Release Grip
    T - Tighten Grip
    y - Say Hello
    H - Home Position
    F - Offer Hand
    G - Return Hand
    E - Cancel Action/Quit Program
        
    Press 'q' to enter advanced arm control mode. Then select a joint by pressing a number (1-9).
    Then control using r and t (send multiple of same key to move further). After selecting
    a joint, press 'q' to go back to joint selection, and press again to go back to basic
    control.
    
    
    """
    text_widget = tk.Label(controls_window, text=instructions, font=("Arial", 12), bg="white", justify="left")
    text_widget.pack(pady=10)
