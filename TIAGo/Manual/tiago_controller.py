import pygame
pygame.quit()
import subprocess
import time

joystick = None

def setup():
    global joystick
    # Initialize pygame and joystick
    pygame.init()
    pygame.joystick.init()

    # Check if a joystick is connected
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Please connect a Logitech controller.")
        exit()

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Connected to: {joystick.get_name()}")


# Basic Arm Command  

def tighten():
    command = "rosrun play_motion run_motion_python_node.py close_right"
    print("Tightening Grip") 
    speak("Caution: Tightening Right Grip")
    subprocess.run(command, shell=True, text=True)

def release():
    command = "rosrun play_motion run_motion_python_node.py open_right"
    print("Releasing Grip") 
    speak("Caution: Releasing Right Grip")  
    subprocess.run(command, shell=True, text=True)
    
def offer():
    command = "rosrun play_motion run_motion_python_node.py offer_right"
    print("Offering Hand")
    speak("Caution: Offering Right Hand")
    subprocess.run(command, shell=True, text=True)

def home_right():
    command = "rosrun play_motion run_motion_python_node.py home_right"
    print("Retracting Hand")
    speak("Caution: Retracting Right Hand")
    subprocess.run(command, shell=True, text=True)

def speak(text):
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

pan = 0
tilt = 0

def move_head():
    global pan
    global tilt
    """Send the head position command to Tiago."""
    if pan != 0:
        command = f"rosrun play_motion move_joint head_1_joint {pan} 1.0"
    else:
        command = f"rosrun play_motion move_joint head_2_joint {tilt} 1.0"
    
    subprocess.run(command, shell=True, text=True)
    pan = 0.0
    tilt = 0.0

def resetHead():
    command = f"rosrun play_motion move_joint head_1_joint 0.0 1.0"
    subprocess.run(command, shell=True, text=True)
    command = f"rosrun play_motion move_joint head_2_joint 0.0 1.0"
    subprocess.run(command, shell=True, text=True)

def lookY(value):
    global tilt
    tilt = value
    move_head()

def lookX(value):
    global pan
    pan = value
    move_head()

torso_pos = 0.15  # 0.0 (down) to 0.34 (up)
step = 0.05  # How much to move per key press

def move_torso():
    global torso_pos
    """Send the torso lift position command to Tiago."""
    command = f"rosrun play_motion move_joint torso_lift_joint {torso_pos} 2.0"
    subprocess.run(command, shell=True, text=True)

# Define key actions
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

def handle_button_press(button):
    if button == 0:
        print("Button A pressed!")
    elif button == 1:
        print("Button B pressed!")
    elif button == 2:
        print("Button X pressed!")
    elif button == 3:
        print("Button Y pressed!")
    elif button == 4:
        home_right()
    elif button == 5:
        offer()
    elif button == 9:
        print("Left Stick")
    elif button == 10:
        resetHead()

def handle_left_joy(value):

    forward = (0,1)
    right = (1,0)
    left = (-1,0)
    backward = (0,-1)

def handle_other_buttons(axis, value):
        
    if axis == 1: #torso. -1 = up. 1 = down
        value = 0
    elif axis == 2: #leftT 1 = press
        if value == 1:
            release()
    elif axis == 3: #lookX -1 = up. 1 = down
        print (value)
        if value <= -1:
            lookX(1)
        elif value >= 1:
            lookX(-1)
    elif axis == 4: #lookY -1 = left. 1 = right
        print (value)
        if value <= -1:
            lookY(1)
        elif value >= 1:
            lookY(-1)
    elif axis == 5: #rightT 1 = press
        if value == 1:
            tighten()

def reset_joystick_state():
    global joystick
    for button in range(joystick.get_numbuttons()):
        joystick.get_button(button)  # Force the button states to be refreshed

def runController():
    setup()
    # Main loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            print(event.type)
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.JOYBUTTONDOWN:  # Check if a button is pressed
                button = event.button
                handle_button_press(button)
            elif event.type == 1538: # Left Joystick
                handle_left_joy(event.value)
                value = event.value
            elif event.type == 1536:
                handle_other_buttons(event.axis, event.value)

      

        reset_joystick_state()
    
    pygame.quit()

if __name__ == "__main__":
    runController()

