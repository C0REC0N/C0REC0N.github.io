import tkinter as tk

def key_controls():   
    """Display keyboard control instructions in a new window."""
    controls_window = tk.Toplevel()
    controls_window.title("Keyboard Controls")
    controls_window.geometry("1000x1000")
    controls_window.configure(bg="white")

    # Header Label
    label = tk.Label(
        controls_window,
        text="Keyboard Control Instructions",
        font=("Arial", 14, "bold"),
        bg="white"
    )
    label.pack(pady=20)

    # Instructions text block
    instructions = """
You must hit Enter to send a keyboard command.
CASE SENSITIVE COMMANDS.

For the following 4 controls, press the key multiple times to move further:
    w - Move Forward
    s - Move Backward
    a - Turn Left
    d - Turn Right

Head Movement (repeat key for more movement):
    i - Look Up
    k - Look Down
    j - Look Left
    l - Look Right

Single-key commands (do not repeat keys):
    W - Raise Torso
    S - Lower Torso
    R - Release Grip
    T - Tighten Grip
    y - Say Hello
    H - Home Position
    F - Offer Hand
    G - Return Hand
    E - Cancel Action / Quit Program

Advanced Arm Control:
    - Press 'q' to enter joint control mode.
    - Select a joint (1-9) by pressing a number.
    - Use 'r' and 't' to move the joint (repeat for more movement).
    - Press 'q' again to return to joint selection, and again to return to normal control.
    """

    # Instruction Label
    text_widget = tk.Label(
        controls_window,
        text=instructions,
        font=("Arial", 12),
        bg="white",
        justify="left",
        anchor="nw"
    )
    text_widget.pack(pady=10, padx=30, anchor="w")
