import pygame
pygame.quit()
import subprocess
import time

# ========== Setup and Globals ========== #

joystick = None
linear_speed = 0.0
angular_speed = 0.0
pan = 0
tilt = 0
torso_pos = 0.15  # 0.0 (down) to 0.34 (up)
step = 0.05       # Step size for torso

def setup():
    """Initialize pygame and the first connected joystick."""
    global joystick
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Please connect a Logitech controller.")
        exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to: {joystick.get_name()}")

# ========== Base Movement ========== #

def move_base():
    """Send velocity command to Tiago's base."""
    global linear_speed, angular_speed

    command = ["timeout", "3", "rostopic", "pub", "--rate", "10", "/mobile_base_controller/cmd_vel", "geometry_msgs/Twist", f"""
linear:
  x: {linear_speed}
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: {angular_speed}"""]

    process = subprocess.Popen(command)
    check_for_stop(process)

def check_for_stop(process):
    """Interrupt base movement if a specific button is pressed."""
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.JOYBUTTONDOWN and event.button == 2:
                running = False
                process.terminate()
            if event is not None:
                running = False

# ========== Arm and Grip ========== #

def tighten():
    print("Tightening Grip")
    speak("Caution: Tightening Right Grip")
    subprocess.run("rosrun play_motion run_motion_python_node.py close_right", shell=True, text=True)

def release():
    print("Releasing Grip")
    speak("Caution: Releasing Right Grip")
    subprocess.run("rosrun play_motion run_motion_python_node.py open_right", shell=True, text=True)

def offer():
    print("Offering Hand")
    speak("Caution: Offering Right Hand")
    subprocess.run("rosrun play_motion run_motion_python_node.py offer_right", shell=True, text=True)

def home_right():
    print("Retracting Hand")
    speak("Caution: Retracting Right Hand")
    subprocess.run("rosrun play_motion run_motion_python_node.py home_right", shell=True, text=True)

# ========== Head Control ========== #

def move_head():
    """Send head pan/tilt command to Tiago."""
    global pan, tilt
    if pan != 0:
        command = f"rosrun play_motion move_joint head_1_joint {pan} 1.0"
    else:
        command = f"rosrun play_motion move_joint head_2_joint {tilt} 1.0"

    subprocess.run(command, shell=True, text=True)
    pan = 0.0
    tilt = 0.0

def lookX(value):
    global pan
    pan = value
    move_head()

def lookY(value):
    global tilt
    tilt = value
    move_head()

def resetHead():
    """Reset head orientation."""
    subprocess.run("rosrun play_motion move_joint head_1_joint 0.0 1.0", shell=True, text=True)
    subprocess.run("rosrun play_motion move_joint head_2_joint 0.0 1.0", shell=True, text=True)

# ========== Torso Control ========== #

def move_torso():
    """Send torso lift command."""
    command = f"rosrun play_motion move_joint torso_lift_joint {torso_pos} 2.0"
    subprocess.run(command, shell=True, text=True)

def torsoUp():
    global torso_pos
    if torso_pos < 0.35:
        torso_pos += step
        print("Rising")
        move_torso()

def torsoDown():
    global torso_pos
    if torso_pos > 0.0:
        torso_pos -= step
        print("Lowering")
        move_torso()

# ========== General Actions ========== #

def speak(text):
    """Text-to-speech using ROS."""
    command = f"""timeout 3 rostopic pub -1 /tts/goal pal_interaction_msgs/TtsActionGoal "{{
        goal: {{
            rawtext: {{
                text: '{text}',
                lang_id: 'en_GB'
            }}
        }}
    }}" """
    print("Tiago is Speaking")
    subprocess.run(command, shell=True, text=True)

def home():
    print("Returning to home position")
    speak("Caution: Return to Home Position")
    subprocess.run("rosrun play_motion run_motion_python_node.py home", shell=True, text=True)

# ========== Controller Input Handlers ========== #

def handle_button_press(button):
    """Handle discrete button presses."""
    if button == 0:
        speak("Hello")
    elif button == 3:
        home()
    elif button == 4:
        home_right()
    elif button == 5:
        offer()
    elif button == 10:
        resetHead()

def handle_left_joy(value):
    """Handle left joystick movement."""
    global linear_speed, angular_speed

    print(value)
    if value == (0, 1):       # Forward
        linear_speed = 0.3
        angular_speed = 0.0
    elif value == (1, 0):     # Right
        linear_speed = 0.0
        angular_speed = -0.3
    elif value == (-1, 0):    # Left
        linear_speed = 0.0
        angular_speed = 0.3
    elif value == (0, -1):    # Backward
        linear_speed = -0.3
        angular_speed = 0.0

    move_base()

def handle_other_buttons(axis, value):
    """Handle triggers and other axes."""
    if axis == 1:  # D-Pad Vertical
        if value <= -1:
            torsoUp()
        elif value >= 1:
            torsoDown()

    elif axis == 2:  # Left Trigger
        if value == 1:
            release()

    elif axis == 3:  # Right Stick X (Pan)
        if value <= -1:
            lookX(1)
        elif value >= 1:
            lookX(-1)

    elif axis == 4:  # Right Stick Y (Tilt)
        if value <= -1:
            lookY(1)
        elif value >= 1:
            lookY(-1)

    elif axis == 5:  # Right Trigger
        if value == 1:
            tighten()

# ========== Joystick Runtime ========== #

def reset_joystick_state():
    """Force joystick state to refresh."""
    for button in range(joystick.get_numbuttons()):
        joystick.get_button(button)

def runController():
    """Main loop to handle joystick input."""
    setup()
    running = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == 1:
                    running = False
                else:
                    handle_button_press(event.button)

            elif event.type == 1538:  # Left Joystick movement
                handle_left_joy(event.value)

            elif event.type == 1536:  # Triggers and Stick Axes
                handle_other_buttons(event.axis, event.value)

        reset_joystick_state()

    pygame.quit()

# ========== Main ========== #

if __name__ == "__main__":
    runController()
