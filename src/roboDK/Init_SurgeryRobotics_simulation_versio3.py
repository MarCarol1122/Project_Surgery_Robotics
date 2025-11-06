from robodk.robolink import *
from robodk.robomath import *
import time
import math
import tkinter as tk
import threading
import socket
import json
import os

# -----------------------------
# CONFIGURACIÓN GENERAL
# -----------------------------
relative_path = "src/roboDK/SurgeryRobotics.rdk"
absolute_path = os.path.abspath(relative_path)

UDP_IP = "0.0.0.0"
UDP_PORT = 12345
BUFFER_SIZE = 1024

ROBOT_NAME = 'UR5e'
ZERO_YAW_TOOL = 0
ZERO_YAW_GRIPPER = 0
READ_INTERVAL_S = 0.01

# -----------------------------
# VARIABLES GLOBALES
# -----------------------------
Endowrist_rpy = None
Gripper_rpy = None
Servo_torques = None
data_lock = threading.Lock()

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

# -----------------------------
# FUNCIONES DE INICIALIZACIÓN
# -----------------------------
def initialize_robodk(absolute_path):
    RDK = Robolink()
    time.sleep(2)
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

# -----------------------------
# FUNCIONES AUXILIARES
# -----------------------------
def endowrist2base_orientation(roll, pitch, yaw):
    roll2 = (roll + 90) % 360
    pitch2 = pitch % 360
    yaw2 = yaw % 360
    return roll2, pitch2, yaw2

def update_text_label(label, tool_orientation, gripper_orientation, status_message, torque_values):
    full_text = f"Tool orientation: {tool_orientation}\nGripper orientation: {gripper_orientation}\n{status_message}\nTorque Values: {torque_values}"
    label.after(0, lambda: label.config(text=full_text))

def update_torque_button_color(total_torque):
    try:
        if total_torque < 30:
            color = "#2ecc71"  # verde
        elif total_torque < 100:
            color = "#f1c40f"  # amarillo
        elif total_torque < 200:
            color = "#e67e22"  # naranja
        else:
            color = "#e74c3c"  # rojo
        torque_button.after(0, lambda: torque_button.config(bg=color))
    except Exception:
        pass

# -----------------------------
# LECTURA DE UDP (ENDOWRIST, GRIPPER, SERVOS)
# -----------------------------
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

                elif device_id == "G2_Servos":  # ✅ nombre correcto del dispositivo de servos
                    # Ejemplo esperado:
                    # {"device":"G2_Servos","t_roll1":12.3,"t_pitch":4.5,"t_yaw":3.2,"t_roll2":10.1}
                    with data_lock:
                        Servo_torques = received_data

            except json.JSONDecodeError:
                print("⚠️ Error decoding JSON data")

        except socket.error as e:
            print(f"Socket error: {e}")
            sock.close()
            break

# -----------------------------
# MOVIMIENTO DEL ROBOT Y ACTUALIZACIÓN DE GUI
# -----------------------------
def move_robot(robot, gripper, needle, text_label):
    global ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, Endowrist_rpy, Gripper_rpy, data_lock, Servo_torques
    global e_roll, e_pitch, e_yaw, g_roll, g_pitch, g_yaw, s1, s2, s3, s4

    while True:
        with data_lock:
            current_Endowrist_rpy = Endowrist_rpy
            current_Gripper_rpy = Gripper_rpy
            current_Servo_torques = Servo_torques

        endowrist_orientation_msg = ""
        gripper_orientation_msg = ""
        status_message = ""
        servo_torques_msg = ""

        # ----------------- ENDOWRIST -----------------
        if current_Endowrist_rpy:
            e_roll = current_Endowrist_rpy.get("roll")
            e_pitch = current_Endowrist_rpy.get("pitch")
            e_yaw = current_Endowrist_rpy.get("yaw")
            s3 = current_Endowrist_rpy.get("s3")
            s4 = current_Endowrist_rpy.get("s4")

            endo_roll, endo_pitch, endo_yaw = endowrist2base_orientation(e_roll, e_pitch, e_yaw)
            endowrist_pose = robot.Pose()
            Xr, Yr, Zr, rr, pr, yr = Pose_2_TxyzRxyz(endowrist_pose)
            endowrist_pose_new = transl(Xr, Yr, Zr) * rotz(math.radians(ZERO_YAW_TOOL)) * rotz(math.radians(endo_yaw)) * roty(math.radians(endo_pitch)) * rotx(math.radians(endo_roll))

            if robot.MoveL_Test(robot.Joints(), endowrist_pose_new) == 0:
                robot.MoveL(endowrist_pose_new, True)
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
            else:
                endowrist_orientation_msg = f"R={round(endo_roll)} P={round(endo_pitch)} W={round((endo_yaw+ZERO_YAW_TOOL)%360)}"
                status_message = "⚠️ Robot cannot reach position"

            if s3 == 0 or s4 == 0:
                current_pose = robot.Pose()
                Tz = transl(0, 0, 5) if s3 == 0 else transl(0, 0, -5)
                new_pose = current_pose * Tz
                status_message = "⬆ S3 pressed: moving up" if s3 == 0 else "⬇ S4 pressed: moving down"

                if robot.MoveL_Test(robot.Joints(), new_pose) == 0:
                    robot.MoveL(new_pose, True)
                else:
                    status_message = "❌ Cannot move further in Z"

        # ----------------- GRIPPER -----------------
        if current_Gripper_rpy:
            g_roll = current_Gripper_rpy.get("roll")
            g_pitch = current_Gripper_rpy.get("pitch")
            g_yaw = current_Gripper_rpy.get("yaw")
            s1 = current_Gripper_rpy.get("s1")
            s2 = current_Gripper_rpy.get("s2")

            gripper_pose = gripper.Pose()
            Xg, Yg, Zg, rg, pg, yg = Pose_2_TxyzRxyz(gripper_pose)
            gripper_pose_new = transl(Xg, Yg, Zg) * rotz(math.radians(ZERO_YAW_GRIPPER)) * rotz(math.radians(g_yaw)) * roty(math.radians(g_pitch)) * rotx(math.radians(g_roll))
            gripper.setPose(gripper_pose_new)
            gripper_orientation_msg = f"R={round(g_roll)} P={round(g_pitch)} W={round((g_yaw+ZERO_YAW_GRIPPER)%360)}"

            if s1 == 0:
                needle.setParentStatic(base)
                status_message = "🟢 S1 pressed: needle released"
            else:
                needle.setParent(gripper)
                needle.setPose(TxyzRxyz_2_Pose([0, 0, 0, 0, 0, 0]))
                status_message = "🔵 S1 not pressed: needle held"

        # ----------------- TORQUES -----------------
        if current_Servo_torques:
            t_roll1 = float(current_Servo_torques.get("t_roll1", 0.0))
            t_pitch = float(current_Servo_torques.get("t_pitch", 0.0))
            t_yaw = float(current_Servo_torques.get("t_yaw", 0.0))
            t_roll2 = float(current_Servo_torques.get("t_roll2", 0.0))
            total_torque = t_roll1 + t_pitch + t_yaw + t_roll2

            servo_torques_msg = (
                f"Roll1={t_roll1:.2f} | Pitch={t_pitch:.2f} | "
                f"Yaw={t_yaw:.2f} | Roll2={t_roll2:.2f} | Total={total_torque:.2f}"
            )
            update_torque_button_color(total_torque)

        # ----------------- GUI UPDATE -----------------
        update_text_label(text_label, endowrist_orientation_msg, gripper_orientation_msg, status_message, servo_torques_msg)
        time.sleep(READ_INTERVAL_S)

# -----------------------------
# FUNCIONES DE INTERFAZ
# -----------------------------
def on_closing():
    global root, sock
    print("Closing...")
    try:
        sock.close()
        print("Socket closed.")
    except Exception as e:
        print(f"Error closing socket: {e}")
    root.destroy()

def set_zero_yaw_tool(value):
    global ZERO_YAW_TOOL
    ZERO_YAW_TOOL = float(value)

def set_zero_yaw_gripper(value):
    global ZERO_YAW_GRIPPER
    ZERO_YAW_GRIPPER = float(value)

# -----------------------------
# MAIN
# -----------------------------
def main():
    global root, ZERO_YAW_TOOL, ZERO_YAW_GRIPPER, robot, gripper, base, text_label, torque_button
    RDK, robot, base, gripper, needle = initialize_robodk(absolute_path)

    root = tk.Tk()
    root.title("Suture Process")
    root.protocol("WM_DELETE_WINDOW", on_closing)

    text_label = tk.Label(root, text="", wraplength=300)
    text_label.pack(padx=20, pady=20)

    torque_button = tk.Button(root, text="Torque Level", bg="gray", fg="white", width=18, height=2)
    torque_button.pack(pady=6)

    tool_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Tool Yaw",
                               command=lambda value: set_zero_yaw_tool(float(value)), length=200)
    tool_yaw_slider.set(ZERO_YAW_TOOL)
    tool_yaw_slider.pack()

    gripper_yaw_slider = tk.Scale(root, from_=-180, to=180, orient=tk.HORIZONTAL, label="Gripper Yaw",
                                  command=lambda value: set_zero_yaw_gripper(float(value)), length=200)
    gripper_yaw_slider.set(ZERO_YAW_GRIPPER)
    gripper_yaw_slider.pack()

    udp_thread = threading.Thread(target=read_data_UDP)
    udp_thread.daemon = True
    udp_thread.start()

    robot_thread = threading.Thread(target=move_robot, args=(robot, gripper, needle, text_label))
    robot_thread.daemon = True
    robot_thread.start()

    root.mainloop()
    print("Pop-up menu closed")
    RDK.CloseRoboDK()
    print("RoboDK closed")

if __name__ == "__main__":
    main()
