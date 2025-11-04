from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json
import os

# Define the relative and absolute path to the RoboDK project file
relative_path = "src/roboDK/SurgeryRobotics.rdk"
absolute_path = os.path.abspath(relative_path)
# Constants
UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 1024
ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

# Global data containers
Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None            # we will store servomotor torques in this variable
data_lock = threading.Lock()    # semaphore to manage data from threads

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
#print(f"Listening on {UDP_IP}:{UDP_PORT}")

# Initialize RoboDK
def initialize_robodk(absolute_path):
    RDK = Robolink()
    time.sleep(2)  # wait for RoboDK to be ready
    RDK.AddFile(absolute_path)
    time.sleep(2)
    robot = RDK.Item(ROBOT_NAME)
    base = RDK.Item(f'{ROBOT_NAME} Base')
    endowrist = RDK.Item('Endowrist')
    gripper = RDK.Item('Gripper')
    needle = RDK.Item('Needle')
    Init_target = RDK.Item('Init')
    robot.setPoseFrame(base)
    robot.setPoseTool(endowrist)
    gripper_init = TxyzRxyz_2_Pose([0, 5, -105, 0, 0, 0])
    gripper.setParent(endowrist)
    gripper.setPose(gripper_init)
    needle_init = TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0])
    needle.setParent(gripper)
    needle.setPose(needle_init)
    robot.setSpeed(50)
    robot.MoveL(Init_target)
    return RDK, robot, base, gripper, needle

# Transformation Endowrist to base
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

# Function to update the label with text
def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    full_text = f"Tool orientation: {tool_orientation}\nGripper orientation: {gripper_orientation}\n{status_message}\nTorque Values: {torque_values}"
    label.after(0, lambda: label.config(text=full_text))

# Update torque button color (GUI thread safe)
def update_torque_button_color(total_torque):
    """
    Change the torque_button color based on total torque.
    Find which should be the thresholds!!!!!
    """
    try:
        # thresholds (adjust as needed)
        if total_torque < 30:
            color = "#2ecc71"  # green
        elif total_torque < 100:
            color = "#f1c40f"  # yellow
        elif total_torque < 200:
            color = "#e67e22"  # orange
        else:
            color = "#e74c3c"  # red

        # torque_button is created in main(); use after to update in GUI thread
        torque_button.after(0, lambda: torque_button.config(bg=color))
    except Exception:
        pass

# Function to read UDP data and update the global variable
def read_data_UDP():
    global Endowrist_rpy, Gripper_rpy, Servo_torques, data_lock
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE) 
            try:
                received_data = json.loads(data.decode())
                device_id = received_data.get("device")
                if device_id == "G2_Endo":
                    with data_lock:
                        Endowrist_rpy = received_data
                elif device_id == "G2_Gri":
                    with data_lock:
                        Gripper_rpy = received_data
                elif device_id == "G2_Servo":      #ASSEGURARME QUE EL ID DEL SERVO ES AQUEST!!!
                    # Expect JSON containing torque fields, e.g.:
                    # {"device":"G2_Servo","Torque_roll1":12.3,"Torque_pitch":4.5,"Torque_yaw":3.2,"Torque_roll2":10.1}
                    with data_lock:
                        Servo_torques = received_data
                else:
                    # Unknown device - ignore or log
                    pass
            except json.JSONDecodeError:
                print("Error decoding JSON data")
        except socket.error as e:
            #print(f"Socket error in UDP reader: {e}")
            sock.close()
            print("Socket closed.")
            break

# Function to process the latest UDP data and move the robot
def move_robot(robot, gripper, needle, text_label):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, data_lock, Servo_torques
    global e_roll, e_pitch, e_yaw, g_roll, g_pitch, g_yaw, s1, s2, s3, s4
    
    endowrist_orientation_msg = ""
    gripper_orientation_msg = ""
    status_message = ""
    servo_torques_msg = ""
    
    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy
            current_Servo_torques = Servo_torques # new addition

        if current_Endowrist_rpy:
            e_roll = Endowrist_rpy.get("roll")
            e_pitch = Endowrist_rpy.get("pitch")
            e_yaw = Endowrist_rpy.get("yaw")
            s3 = Endowrist_rpy.get("s3")
            s4 = Endowrist_rpy.get("s4")
            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            #print(f"Endowrist: {endo_roll}, {endo_pitch}, {endo_yaw}")
            
            # Move Endowrist
            endowrist_pose = robot.Pose()
            Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
            endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))
            if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                robot.MoveL(endowrist_pose_new, True)
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = ""
            else:
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = "Robot cannot reach the position"
                
            if s3 == 0 or s4 == 0:
                current_pose = robot.Pose()
                # Z movement based on S3 and S4 buttons
                Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                new_pose = current_pose * Tz  # translació relativa

                status_message = "⬆ Botó S3 premut: pujant" if s3 == 0 else "⬇ Botó S4 premut: baixant"

                if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                    robot.MoveL(new_pose, True)
                else:
                    status_message = "❌ No es pot moure més en Z (relatiu)"
                    
        if current_Gripper_rpy:
            g_roll = Gripper_rpy.get("roll")
            g_pitch = Gripper_rpy.get("pitch")
            g_yaw = Gripper_rpy.get("yaw")
            s1 = Gripper_rpy.get("s1")
            s2 = Gripper_rpy.get("s2")
            #print(f"Gripper: {g_roll}, {g_pitch}, {g_yaw}")
            # Move Gripper
            gripper_pose = gripper.Pose()
            Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
            gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
            gripper.setPose(gripper_pose_new)
            gripper_orientation_msg = f"R={round(g_roll)} P={round(g_pitch)} W={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"     
            if s1 == 0:
                #Obre la pinça → deixa anar l’agulla
                needle.setParentStatic(base)
                status_message = "🟢 S1 premut: agulla alliberada"

            elif s1 == 1:
                #Tanca la pinça → agafa l’agulla
                needle.setParent(gripper)
                needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                status_message = "🔵 S1 no premut: agulla agafada"
        
        # --- Servo torques (new) ---
        servo_torques_msg = ""
        if current_Servo_torques:
            # Extract expected fields, default to 0.0 if not present
            t_roll1 = float(current_Servo_torques.get("Torque_roll1", 0.0))
            t_pitch = float(current_Servo_torques.get("Torque_pitch", 0.0))
            t_yaw = float(current_Servo_torques.get("Torque_yaw", 0.0))
            t_roll2 = float(current_Servo_torques.get("Torque_roll2", 0.0))
            total_torque = t_roll1 + t_pitch + t_yaw + t_roll2

            servo_torques_msg = (
                f"Roll1={t_roll1:.2f} | Pitch={t_pitch:.2f} | "
                f"Yaw={t_yaw:.2f} | Roll2={t_roll2:.2f} | Total={total_torque:.2f}"
            )

            # Update the color-coded button (GUI)
            update_torque_button_color(total_torque)

        # Update the label with the latest values (including torque string)
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, servo_torques_msg)

        time.sleep(READ_INTERVAL_S)  # define the reading interval

def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        print("Ending Socket")
        #initialize_robodk()
        #print("Program INITIALIZED")
    except Exception as e:
        #print(f"Error al tancar el socket: {e}")
        pass
    root.destroy()

# Update functions for sliders
def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)

# Main function
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label, absolute_path
    global torque_button   # GUI element used by helper
    RDK, robot, base, gripper, needle = initialize_robodk(absolute_path)

    root = tk.Tk()
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing)  # Proper closing
    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)

    # Create a color-coded button for torque visualization (NEW)
    torque_button = tk.Button(root, text="Torque Level", bg="gray", fg="white", width=18, height=2)
    torque_button.pack(pady=6)

    # Add sliders for ZERO_YAW_TOOL and ZERO_YAW_GRIPPER
    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                                    command=lambda value: set_zero_yaw_tool(float(value)), length=200)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                        command=lambda value: set_zero_yaw_gripper(float(value)), length=200)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    # Start the UDP reading thread
    udp_thread = threading.Thread(target=read_data_UDP)
    udp_thread.daemon = True
    udp_thread.start()

    # Start the robot movement thread
    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label))
    robot_thread.daemon = True
    robot_thread.start()

    root.mainloop()
    print("Pop-up menu closed")
    RDK.CloseRoboDK()
    print("RoboDK closed")

if __name__ == "__main__":
    main()
