import tkinter as tk
from tkinter import messagebox, ttk, scrolledtext, filedialog, simpledialog
import collections
import threading
import time
import queue
import os
import math
import sys
import ast
import traceback
import difflib
import re
from pathlib import Path

if getattr(sys, 'frozen', False):
    base_dir = Path(sys._MEIPASS)
    root_dir = Path(sys.executable).parent
else:
    base_dir = Path(__file__).parent
    root_dir = base_dir.parent

for p in [str(base_dir), str(root_dir)]:
    if p not in sys.path:
        sys.path.insert(0, p)

from PYTHON_SDK.GentoRobot import GentoRobot, RobotDataManager, HandDataManager, ArmsSynchronousPlanningParams, error_dict, FXObjType, FXLogMask, FXObjMask, FXRefOriType, robot_type_map, state_map, FXHandType,FXHandAction,FXHandState, FX_InvKineSolverParams, ToolDynTaskStatus, LoadDynamicPara, FXUserFbkType

_matplotlib_error = None
try:
    import matplotlib
    matplotlib.use("TkAgg")
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
except Exception as _e:  # pragma: no cover - depends on the local install
    _matplotlib_error = _e
    Figure = None
    FigureCanvasTkAgg = None
    NavigationToolbar2Tk = None


# ==================== Real-time data flattening ====================
_VIZ_SCALARS = [
    # (key, path into the rt dict)
    ("frame_serial", ("frame_serial",)),
    ("head/cmd_tag", ("head", "cdm_tag")),
    ("body/cmd_tag", ("body", "cmd_tag")),
    ("lift/cmd_tag", ("lift", "cmd_tag")),
]


def flatten_rt(rt):
    """Flatten a get_rt_dict() dictionary into {signal_key: float}.

    Only numeric leaves are emitted; anything missing or non-numeric is skipped
    so a controller running older firmware simply shows fewer signals instead of
    raising from inside the sampling thread.
    """
    if not isinstance(rt, dict) or "error" in rt:
        return {}

    out = {}

    def put(key, value):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return
        out[key] = float(value)

    def put_list(prefix, values):
        if not isinstance(values, (list, tuple)):
            return
        for i, v in enumerate(values):
            put(f"{prefix}/{i}", v)

    for key, path in _VIZ_SCALARS:
        node = rt
        for part in path:
            node = node.get(part) if isinstance(node, dict) else None
        put(key, node)

    put_list("pdo_state", rt.get("pdo_state"))
    put_list("robot_imu", rt.get("robot_imu"))
    put_list("agv_imu", rt.get("agv_imu"))

    # arms[0]/arms[1] -> arms/0/<group>/<field>/<index>
    for arm_idx, arm in enumerate(rt.get("arms") or []):
        if not isinstance(arm, dict):
            continue
        base = f"arms/{arm_idx}"
        put(f"{base}/state/cur", (arm.get("state") or {}).get("cur"))
        put(f"{base}/state/err", (arm.get("state") or {}).get("err"))
        for group in ("cmd", "fb"):
            fields = arm.get(group)
            if not isinstance(fields, dict):
                continue
            for field, values in fields.items():
                put_list(f"{base}/{group}/{field}", values)

    for name in ("head", "body", "lift"):
        node = rt.get(name)
        if not isinstance(node, dict):
            continue
        put(f"{name}/state/cur", (node.get("state") or {}).get("cur"))
        put(f"{name}/state/err", (node.get("state") or {}).get("err"))
        # cmd_pos / cmd_tag are scalars-or-lists, fb_* are lists; put/put_list
        # each ignore the shape they cannot handle.
        for field, values in node.items():
            if field in ("state", "cmd_tag", "cdm_tag"):
                continue
            put_list(f"{name}/{field}", values)

    fbk = rt.get("system_user_fbk")
    if isinstance(fbk, dict):
        put_list("system_user_fbk/types", fbk.get("types"))
        # Channels are a list of lists; unfold it so every channel element gets
        # its own stable scalar key.
        for chn_idx, chn in enumerate(fbk.get("chn") or []):
            if isinstance(chn, (list, tuple)):
                for i, v in enumerate(chn):
                    put(f"system_user_fbk/chn{chn_idx}/{i}", v)
            else:
                put(f"system_user_fbk/chn{chn_idx}", chn)

    return out


# Human-readable labels for a flat signal key, used by the tree view.
_VIZ_FIELD_LABELS = {
    "fb_pos": "joint pos",
    "fb_vel": "joint vel",
    "cmd_pos": "cmd pos",
    "fb_sensor": "sensor torque",
    "fb_ext_torque": "external torque",
    "fb_gravity_torque": "gravity torque",
    "base_force": "base force",
    "flange_force": "flange force",
    "joint_pos": "cmd joint pos",
    "joint_trq": "cmd joint torque",
    "force_dir": "cmd force dir",
    "torque_dir": "cmd torque dir",
    "ref_orientation": "cmd ref ori",
    "pdo_state": "pdo state",
    "robot_imu": "robot IMU",
    "agv_imu": "agv IMU",
    "types": "fbk types",
}


def viz_signal_label(key):
    """Turn 'arms/0/fb/fb_pos/2' into 'arm0 fb joint pos 2' for display.

    Output must stay unique per key: the dialog uses it as the matplotlib legend
    label, and duplicate labels would collapse into one legend entry.
    """
    parts = key.split("/")
    if len(parts) >= 3 and parts[0] == "arms":
        group = parts[2]
        field = _VIZ_FIELD_LABELS.get(parts[3], parts[3])
        head = f"arm{parts[1]} {group} {field}"
        if len(parts) >= 5:
            head += f" {parts[4]}"
        return head
    if len(parts) >= 2 and parts[0] in ("head", "body", "lift"):
        field = _VIZ_FIELD_LABELS.get(parts[1], parts[1])
        head = f"{parts[0]} {field}"
        if len(parts) >= 3:
            head += f" {parts[2]}"
        return head
    if len(parts) >= 2 and parts[0] == "system_user_fbk":
        name = _VIZ_FIELD_LABELS.get(parts[1], parts[1])
        return " ".join(["fbk", name] + parts[2:])
    return " ".join([_VIZ_FIELD_LABELS.get(p, p) for p in parts])


# Default selection when the dialog opens: the seven Arm0 joint angles.
_VIZ_DEFAULT_KEYS = [f"arms/0/fb/fb_pos/{i}" for i in range(7)]

# Tick marks in the signal tree. These two are deliberately in GB2312: a Treeview
# label is drawn with the Tk system font, and a glyph outside the console codepage
# ("☑"/"☐") renders as a box or not at all on a Chinese Windows install.
_VIZ_TICK = "● "      # ● selected
_VIZ_UNTICK = "○ "    # ○ not selected

# Redraw period (20 Hz). The plot does not need to match the sampling rate: a
# 1000 Hz buffer drawn at 1000 fps just burns CPU.
_VIZ_DRAW_INTERVAL_MS = 50

# Sampling rate bounds for the rate slider. The SDK timer runs at 1 kHz, so
# 1000 Hz is the fastest rate that still yields independent samples.
_VIZ_RATE_MIN = 1
_VIZ_RATE_MAX = 1000
_VIZ_RATE_DEFAULT = 1000

# Above this many curves the legend covers more of the plot than it explains.
_VIZ_LEGEND_MAX = 20


class App:
    def __init__(self, root):
        self.root = root
        self.root.title("FXPlatform")
        self.root.geometry("1350x800")
        self.root.configure(bg="#f0f0f0")

        self.data_manager = None
        self.rt = None
        self.sg = None
        self._log_running = False
        self._error_log_history = []   # latest 5 error messages, newest first
        self._warning_log_history = []  # latest 5 warning messages, newest first
        self._log_history = []  # latest 5 controller messages, newest first

        # IMU settings window state (lazily created in imu_settings)
        self._imu_win = None
        self._imu_refresh_id = None
        self._imu_labels = {"robot": [], "agv": []}


        # Data Visualization window state (lazily created in
        # data_visualization_dialog). Declared up-front so _viz_close can run
        # even if the dialog was never opened.
        self._viz_win = None
        self._viz_thread = None
        self._viz_draw_id = None
        self._viz_alive = False
        self._viz_running = False
        self._viz_buf = {}
        self._viz_lines = {}
        self._viz_checked = {}
        self._viz_selected = []
        self._viz_color_slots = []
        self._viz_view_end = 0.0

        self.servo_versions = {}
        self.servo_cfg_versions = {}
        self.sensor_versions = {}
        self.sensor_serials = {}
        self.sys_version = "Unknown"
        self.sdk_version = sdk_version
        self.drag_mode = False
        self.ini_file_path = ""
        self.system_file_path = ""

        self.params = []
        self.init_kd_variables()
        self.points1 = []
        self.points2 = []
        self.body_points = []
        self.head_points = []
        self.lift_points = []
        self.hand_points1 = []
        self.hand_points2 = []

        self.command1 = []
        self.command2 = []

        self.display_mode = 0
        self.mode_names = ["Position", "CmdPosition", "Velocity", "SensorTorque", "TorqueExt", "MotorTorque",
                           "ExtPosition", "FlangForce","Temperature","GravityTorque"]
        self.data_keys = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'), ('fb_ext_torque'), ('joint_torque'),
                          ('ext_pos'), ('flange_force'),('joint_temp'),('fb_gravity_torque')]
        self.arm_rt_key = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'), ('fb_ext_torque'), 
                           ('flange_force'),('fb_gravity_torque')]
        self.arm_sg_key = [('joint_torque'), ('ext_pos'),('joint_temp')]
        self.body_rt_key = [('fb_pos'), ('cmd_pos'), ('fb_vel'), ('fb_sensor'),('fb_gravity_torque')]
        self.body_sg_key = [('joint_torque'), ('ext_pos'),('joint_temp')]
        self.head_rt_key = [('fb_pos'), ('cmd_pos')]
        self.head_sg_key = [('ext_pos'),('joint_temp')]
        self.lift_rt_key = [('fb_pos'), ('cmd_pos')]
        self.lift_sg_key = [('joint_torque')]

        self.hand_rt_key=[('fb_pos'),('vel'),('cmd_pos')]
        self.hand_sg_key = [('joint_torque'),('joint_temp')]

        self.widgets = {}

        # Create control panel
        self.create_control_components()

        # Create main content area
        self.create_main_content()

        # Create component frames
        self.create_left_arm_components()
        self.create_separator()
        self.create_right_arm_components()
        self.create_separator()
        self.create_body_components()
        self.create_separator()
        self.create_head_components()
        self.create_separator()
        self.create_lift_components()
        self.create_separator()
        self.create_user_fbk_components()
        # self.create_separator()
        # self.create_hand0_components()
        # self.create_separator()
        # self.create_hand1_components()

        # Create status bar
        self.create_status_bar()
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.connected = False
        self.data_subscriber = None
        self.stop_event = threading.Event()
        self.thread = None
        self._link_monitor_id = None
        self._last_robot_ip = ""

    def create_main_content(self):
        self.main_container = tk.Frame(self.root, bg="white", padx=5, pady=10)
        self.main_container.pack(fill="both", expand=True)
        self.main_canvas = tk.Canvas(self.main_container, bg="white", highlightthickness=0)
        self.main_scrollbar = ttk.Scrollbar(self.main_container, orient="vertical", command=self.main_canvas.yview)
        self.scrollable_frame = tk.Frame(self.main_canvas, bg="white")
        self.stop_frame = tk.Frame(self.main_canvas, bg='white')
        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all"))
        )
        self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.main_canvas.configure(yscrollcommand=self.main_scrollbar.set)
        self.main_canvas.pack(side="left", fill="both", expand=True, padx=(0, 5))
        self.main_scrollbar.pack(side="right", fill="y")
        self.main_canvas.bind_all("<MouseWheel>", self.on_mousewheel)

    def update_vertical_scrollbar(self, *args):
        self.v_scrollbar.set(*args)
        self.main_canvas.yview(*args)

    def update_horizontal_scrollbar(self, *args):
        self.h_scrollbar.set(*args)
        self.main_canvas.xview(*args)

    def scroll_horizontally(self, *args):
        self.main_canvas.xview(*args)

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_shift_mousewheel(self, event):
        self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_horizontal_mousewheel(self, event):
        if event.delta:
            self.main_canvas.xview_scroll(int(-1 * (event.delta / 120)), "units")
        else:
            if event.num == 4:
                self.main_canvas.xview_scroll(-1, "units")
            elif event.num == 5:
                self.main_canvas.xview_scroll(1, "units")

    def create_separator(self):
        separator = tk.Frame(self.scrollable_frame, height=2, bg="#7F888C")
        separator.pack(fill="x", pady=(5, 10))

    def create_left_arm_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # ---------------------------- First column: status info ----------------------------
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="ARM0", font=('Arial', 11, 'bold'),
                 fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0, 5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        # Status row
        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=12, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.left_state_main.pack(side="left")

        # Drag flag
        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Drag:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_1 = tk.Label(row2, text='Drag off', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_1.pack(side="left")

        # Low speed flag
        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(anchor="w", pady=(0, 5))
        tk.Label(row3, text="Motion:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_2 = tk.Label(row3, text='Stopped', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_2.pack(side="left")

        # Error code
        row4 = tk.Frame(status_info_frame, bg="white")
        row4.pack(anchor="w", pady=(0, 5))
        tk.Label(row4, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.left_state_3 = tk.Label(row4, text='None', font=('Arial', 9),
                                     fg='#34495e', bg='white', width=12, pady=3,
                                     relief=tk.SUNKEN, bd=1)
        self.left_state_3.pack(side="left")

        # Error detail (wraps to multiple lines, keep fill)
        row5 = tk.Frame(status_info_frame, bg="white")
        row5.pack(fill="x", pady=(0, 5))
        self.left_arm_error = tk.Label(row5, text="", font=('Arial', 9),
                                       fg='#2c3e50', bg='white', pady=5,
                                       anchor='w', wraplength=100, justify='left')
        self.left_arm_error.pack(fill="x", padx=5)

        # ---------------------------- Second column: control functions ----------------------------
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings area
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10,
                                     relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.left_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_speed_entry.pack(side="left")
        self.left_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.left_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.left_accel_entry.pack(side="left")
        self.left_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        speed_btn1 = tk.Button(param_row, text="Confirm Speed", width=15,
                               command=lambda: self.vel_acc_set('Arm0'),
                               bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn1.pack(side="left", padx=(0, 20))

        self.left_impedance_btn = tk.Button(param_row, text="Impedance Params", width=15,
                                            command=lambda: self.show_impedance_dialog('Arm0'),
                                            bg="#9C27B0", fg="white", font=("Arial", 9, "bold"))
        self.left_impedance_btn.pack(side="left")

        # Status switching + error handling (horizontal layout)
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))

        # Status switching area
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))

        self.reset_button = tk.Button(state_row1, text="IDLE", width=10,
                                      command=lambda: self.idle_state('Arm0'),
                                      bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.reset_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.position_button = tk.Button(state_row1, text="Position", width=10,
                                         command=lambda: self.position_state('Arm0'), bg="#A8D5BA", fg="black",
                                         font=("Arial", 10, "bold"))
        self.position_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        drag_frame = tk.Frame(state_row1, bg="white")
        drag_frame.pack(side="left", padx=(0, 0))
        self.drag_combo = ttk.Combobox(drag_frame, values=["joint", "cartX", "cartY", "cartZ", "cartR"],
                                       state="readonly", width=4)
        self.drag_combo.current(0)
        self.drag_combo.pack(side="left", padx=(0, 0))
        self.drag_btn = tk.Button(drag_frame, text="Drag", width=5,
                                  command=lambda: self.drag_state('Arm0'),
                                  bg="#D9B0B0", fg="black", font=("Arial", 9, "bold"))
        self.drag_btn.pack(side="left")


        state_row2 = tk.Frame(state_switch_frame, bg="white")
        state_row2.pack(fill="x", pady=(0, 5))

        self.pd_button = tk.Button(state_row2, text="PD", width=10,
                                   command=lambda: self.pd_state('Arm0'), bg="#B0D9D9", fg="black",
                                   font=("Arial", 10, "bold"))
        self.pd_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.jointimp_button = tk.Button(state_row2, text="JointImp", width=10,
                                         command=lambda: self.jointImp_state('Arm0'), bg="#C5B8D9", fg="black",
                                         font=("Arial", 10, "bold"))
        self.jointimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))

        state_row3 = tk.Frame(state_switch_frame, bg="white")
        state_row3.pack(fill="x", pady=(0, 5))
        self.cartimp_button = tk.Button(state_row3, text="CartImp", width=10,
                                        command=lambda: self.cartImp_state('Arm0'),
                                        bg="#A8C4D9", fg="black", font=("Arial", 10, "bold"))
        self.cartimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.forceimp_button = tk.Button(state_row3, text="ForceImp", width=10,
                                         command=lambda: self.forceImp_state('Arm0'), bg="#D9C5A8", fg="black",
                                         font=("Arial", 10, "bold"))
        self.forceimp_button.pack(side="left", pady=(0, 5), padx=(0, 5))


        # Error handling area
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.reset_btn_arm0 = tk.Button(servo_frame, text="Reset", width=10,
                                        command=lambda: self.reset_error('Arm0'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"),
                                        relief=tk.RAISED, bd=2)
        self.reset_btn_arm0.pack(side="left", padx=(0, 5))

        self.get_servo_error_left_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                                  command=lambda: self.error_get('Arm0'),
                                                  font=("Arial", 10, "bold"))
        self.get_servo_error_left_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")

        self.release_collab_left_btn = tk.Button(control_frame, text="CR", width=5,
                                                 command=lambda: self.cr_state('Arm0'),
                                                 bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.release_collab_left_btn.pack(side="left", padx=(0, 5))

        self.release_brake_left_btn = tk.Button(control_frame, text="Brake", width=10,
                                                command=lambda: self.brake('Arm0'),
                                                font=("Arial", 10, "bold"))
        self.release_brake_left_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_left_btn = tk.Button(control_frame, text="UnBrake", width=10,
                                             command=lambda: self.release_brake('Arm0'),
                                             font=("Arial", 10, "bold"))
        self.hold_brake_left_btn.pack(side="left")

        # ---------------------------- Third column: realtime data + position command ----------------------------
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=1,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row
        joint_pos_frame = tk.Frame(data_frame, bg="white")
        joint_pos_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_pos_frame, text="J1~J7:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.left_joint_text = tk.Text(joint_pos_frame, width=55, height=1,
                                       font=('Arial', 9), bg='white',
                                       relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.left_joint_text.tag_configure("center", justify='center')
        self.left_joint_text.pack(side="left")
        self.left_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        pose_frame = tk.Frame(data_frame, bg="white")
        pose_frame.pack(fill="x", pady=(0, 5))
        tk.Label(pose_frame, text="XYZABC:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.left_pose_text = tk.Text(pose_frame, width=55, height=1,
                                      font=('Arial', 9), bg='white',
                                      relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.left_pose_text.tag_configure("center", justify='center')
        self.left_pose_text.pack(side="left")
        self.left_pose_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.left_pose_text.tag_add("center", "1.0", "end")
        self.left_pose_text.config(state="disabled")

        # Position command area
        joint_cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10,
                                         relief=tk.GROOVE, borderwidth=2,
                                         style="MyCustom.TLabelframe")
        joint_cmd_frame.pack(fill="x")

        # First row: Get current position + input + Add
        row_cmd1 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))

        self.btn_add3 = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Arm0'))
        self.btn_add3.pack(side="left", padx=(0, 5))

        self.entry_var = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry = tk.Entry(row_cmd1, textvariable=self.entry_var, width=45)
        self.entry.pack(side="left", padx=(5, 5))

        self.btn_add1 = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Arm0'))
        self.btn_add1.pack(side="left", padx=(20, 5))

        # Second row: Delete + point selection + Run
        row_cmd2 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))

        self.btn_del1 = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Arm0'))
        self.btn_del1.pack(side="left", padx=(0, 5))

        self.combo1 = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.combo1.pack(side="left", padx=(0, 5))

        self.btn_run1 = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Arm0'),
                                  font=("Arial", 11, "bold"), fg='white',
                                  bg='#EC2A23', border=5)
        self.btn_run1.pack(side="left", padx=(0, 5))

    def create_right_arm_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # ---------------------------- First column: status info ----------------------------
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="ARM1", font=('Arial', 11, 'bold'),
                 fg='#2c3e50', bg="white").pack(anchor="w", padx=40, pady=(0, 5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                         fg='#34495e', bg='white', width=15, pady=3,
                                         relief=tk.SUNKEN, bd=1)
        self.right_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Drag:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_1 = tk.Label(row2, text='Drag off', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_1.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(anchor="w", pady=(0, 5))
        tk.Label(row3, text="Motion:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_2 = tk.Label(row3, text='Stopped', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_2.pack(side="left")

        row4 = tk.Frame(status_info_frame, bg="white")
        row4.pack(anchor="w", pady=(0, 5))
        tk.Label(row4, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.right_state_3 = tk.Label(row4, text='None', font=('Arial', 9),
                                      fg='#34495e', bg='white', width=15, pady=3,
                                      relief=tk.SUNKEN, bd=1)
        self.right_state_3.pack(side="left")

        row5 = tk.Frame(status_info_frame, bg="white")
        row5.pack(fill="x", pady=(0, 5))
        self.right_arm_error = tk.Label(row5, text="", font=('Arial', 9),
                                        fg='#2c3e50', bg='white', pady=5,
                                        anchor='w', wraplength=100, justify='left')
        self.right_arm_error.pack(fill="x", padx=5)

        # ---------------------------- Second column: control functions ----------------------------
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings area
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10,
                                     relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))

        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")

        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.right_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_speed_entry.pack(side="left")
        self.right_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.right_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.right_accel_entry.pack(side="left")
        self.right_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))

        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15,
                              command=lambda: self.vel_acc_set('Arm1'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        self.right_impedance_btn = tk.Button(param_row, text="Impedance Params", width=15,
                                             command=lambda: self.show_impedance_dialog('Arm1'),
                                             bg="#9C27B0", fg="white", font=("Arial", 9, "bold"))
        self.right_impedance_btn.pack(side="left")

        # Status switching + error handling (horizontal layout)
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))

        # Status switching area
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))

        self.reset_button_r = tk.Button(state_row1, text="IDLE", width=10,
                                        command=lambda: self.idle_state('Arm1'),
                                        bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.reset_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.position_button_r = tk.Button(state_row1, text="Position", width=10,
                                           command=lambda: self.position_state('Arm1'), bg="#A8D5BA", fg="black",
                                           font=("Arial", 10, "bold"))
        self.position_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        drag_frame = tk.Frame(state_row1, bg="white")
        drag_frame.pack(side="left", padx=(0, 0))

        self.drag_combo_r = ttk.Combobox(drag_frame, values=["joint", "cartX", "cartY", "cartZ", "cartR"],
                                         state="readonly", width=4)
        self.drag_combo_r.current(0)
        self.drag_combo_r.pack(side="left", padx=(0, 0))
        self.drag_btn_r = tk.Button(drag_frame, text="Drag", width=5,
                                    command=lambda: self.drag_state('Arm1'),
                                    bg="#D9B0B0", fg="black", font=("Arial", 9, "bold"))
        self.drag_btn_r.pack(side="left")

        state_row2 = tk.Frame(state_switch_frame, bg="white")
        state_row2.pack(fill="x", pady=(0, 5))

        self.pd_button_r = tk.Button(state_row2, text="PD", width=10,
                                     command=lambda: self.pd_state('Arm1'), bg="#B0D9D9", fg="black",
                                     font=("Arial", 10, "bold"))
        self.pd_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.jointimp_button_r = tk.Button(state_row2, text="JointImp", width=10,
                                           command=lambda: self.jointImp_state('Arm1'), bg="#C5B8D9", fg="black",
                                           font=("Arial", 10, "bold"))
        self.jointimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        state_row3 = tk.Frame(state_switch_frame, bg="white")
        state_row3.pack(fill="x", pady=(0, 5))

        self.cartimp_button_r = tk.Button(state_row3, text="CartImp", width=10,
                                          command=lambda: self.cartImp_state('Arm1'),
                                          bg="#A8C4D9", fg="black", font=("Arial", 10, "bold"))
        self.cartimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))

        self.forceimp_button_r = tk.Button(state_row3, text="ForceImp", width=10,
                                           command=lambda: self.forceImp_state('Arm1'), bg="#D9C5A8", fg="black",
                                           font=("Arial", 10, "bold"))
        self.forceimp_button_r.pack(side="left", pady=(0, 5), padx=(0, 5))


        # Error handling area
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10,
                                            relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))

        self.reset_btn_arm1 = tk.Button(servo_frame, text="Reset", width=10,
                                        command=lambda: self.reset_error('Arm1'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"),
                                        relief=tk.RAISED, bd=2)
        self.reset_btn_arm1.pack(side="left", padx=(0, 5))

        self.get_servo_error_right_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                                   command=lambda: self.error_get('Arm1'),
                                                   font=("Arial", 10, "bold"))
        self.get_servo_error_right_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")

        self.release_collab_right_btn = tk.Button(control_frame, text="CR", width=5,
                                                  command=lambda: self.cr_state('Arm1'),
                                                  bg="#4CAF50", fg="white", font=("Arial", 10, "bold"))
        self.release_collab_right_btn.pack(side="left", padx=(0, 5))

        self.release_brake_right_btn = tk.Button(control_frame, text="Brake", width=10,
                                                 command=lambda: self.brake('Arm1'),

                                                 font=("Arial", 10, "bold"))
        self.release_brake_right_btn.pack(side="left", padx=(0, 5))

        self.hold_brake_right_btn = tk.Button(control_frame, text="UnBrake", width=10,
                                              command=lambda: self.release_brake('Arm1'),
                                              font=("Arial", 10, "bold"))
        self.hold_brake_right_btn.pack(side="left")

        # ---------------------------- Third column: realtime data + position command ----------------------------
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=1,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # Joint positions row (right arm)
        joint_frame = tk.Frame(data_frame, bg="white")
        joint_frame.pack(fill="x", pady=(0, 5))
        tk.Label(joint_frame, text="J1~J7:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.r_joint_text = tk.Text(joint_frame, width=55, height=1,
                                    font=('Arial', 9), bg='white',
                                    relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.r_joint_text.tag_configure("center", justify='center')
        self.r_joint_text.pack(side="left", fill="x")
        self.r_joint_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000,0.000")
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        pose_frame = tk.Frame(data_frame, bg="white")
        pose_frame.pack(fill="x", pady=(0, 5))
        tk.Label(pose_frame, text="XYZABC:", font=('Arial', 10, 'bold'), width=8,
                 bg='white').pack(side="left", padx=(0, 2))
        self.right_pose_text = tk.Text(pose_frame, width=55, height=1,
                                       font=('Arial', 9), bg='white',
                                       relief=tk.SUNKEN, bd=1, wrap=tk.NONE)
        self.right_pose_text.tag_configure("center", justify='center')
        self.right_pose_text.pack(side="left")
        self.right_pose_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.right_pose_text.tag_add("center", "1.0", "end")
        self.right_pose_text.config(state="disabled")

        # Position command area
        joint_cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10,
                                         relief=tk.GROOVE, borderwidth=2,
                                         style="MyCustom.TLabelframe")
        joint_cmd_frame.pack(fill="x")

        # Realtime data area
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10,
                                    relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))

        # First row: Get current position + input + Add
        row_cmd1 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))

        self.btn_get_cur_r = tk.Button(row_cmd1, text="GetCurPos", width=8,
                                       command=lambda: self.get_current_pos('Arm1'))
        self.btn_get_cur_r.pack(side="left", padx=(0, 5))

        self.entry_var1 = tk.StringVar(value="0,0,0,0,0,0,0")
        self.entry1 = tk.Entry(row_cmd1, textvariable=self.entry_var1, width=45)
        self.entry1.pack(side="left", padx=(5, 5))

        self.btn_add_r = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Arm1'))
        self.btn_add_r.pack(side="left", padx=(20, 5))

        # Second row: Delete + point selection + Run
        row_cmd2 = tk.Frame(joint_cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))

        self.btn_del_r = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Arm1'))
        self.btn_del_r.pack(side="left", padx=(0, 5))

        self.combo2 = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.combo2.pack(side="left", padx=(5, 5))

        self.btn_run_r = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Arm1'),
                                   font=("Arial", 11, "bold"), fg='white',
                                   bg='#EC2A23', border=5)
        self.btn_run_r.pack(side="left", padx=(0, 5))

    # ==================== Body Component ====================
    def create_body_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left side: status info
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="BODY", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.body_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.body_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.body_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.body_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.body_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.body_error_detail.pack(fill="x", padx=5)

        # Middle area: parameters, status switching, error handling
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="y", expand=True, padx=(0, 15))

        # Parameter settings
        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.body_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.body_speed_entry.pack(side="left")
        self.body_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.body_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.body_accel_entry.pack(side="left")
        self.body_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Body'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))
        self.body_impedance_btn = tk.Button(param_row, text="PD Params", width=15,
                                            command=lambda: self.show_impedance_dialog('Body'), bg="#9C27B0",
                                            fg="white", font=("Arial", 9, "bold"))
        self.body_impedance_btn.pack(side="left")

        # Status switching
        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))
        self.body_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Body'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.body_idle_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.body_pos_btn = tk.Button(state_row1, text="Position", width=10,
                                      command=lambda: self.position_state('Body'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        self.body_pos_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.body_jointimp_btn = tk.Button(state_row1, text="PD", width=10,
                                           command=lambda: self.pd_state('Body'), bg="#d9d0ca", fg="black",
                                           font=("Arial", 10, "bold"))
        self.body_jointimp_btn.pack(side="left", pady=(0, 5), padx=(0, 5))

        # Error handling
        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))
        self.body_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Body'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)
        self.body_reset_btn.pack(side="left", padx=(0, 5))
        self.body_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                            command=lambda: self.error_get('Body'), font=("Arial", 10, "bold"))
        self.body_get_error_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")
        self.body_brake_btn = tk.Button(control_frame, text="Brake", width=10, command=lambda :self.brake('Body'),
                                        font=("Arial", 10, "bold"))
        self.body_brake_btn.pack(side="left", padx=(0, 5))
        self.body_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, command=lambda :self.release_brake('Body'),
                                          font=("Arial", 10, "bold"))
        self.body_unbrake_btn.pack(side="left")

        # Right side: realtime data and position command
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        # Realtime data
        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="Pos(J1~J6):", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.body_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.body_pos_text.tag_configure("center", justify='center')
        self.body_pos_text.pack(side="left")
        self.body_pos_text.insert("1.0", "0.000,0.000,0.000,0.000,0.000,0.000")
        self.body_pos_text.tag_add("center", "1.0", "end")
        self.body_pos_text.config(state="disabled")

        # Position command
        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.body_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Body'))
        self.body_get_btn.pack(side="left", padx=(0, 5))
        self.body_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.body_cmd_entry.insert(0, "0,0,0,0,0,0")
        self.body_cmd_entry.pack(side="left", padx=(5, 5))
        self.body_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Body'))
        self.body_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.body_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Body'))
        self.body_del_btn.pack(side="left", padx=(0, 5))
        self.body_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.body_combo.pack(side="left", padx=(0, 5))
        self.body_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Body'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.body_run_btn.pack(side="left", padx=(0, 5))

        self.body_points = []
        self.body_combo['values'] = []

    # ==================== Head Component ====================
    def create_head_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="HEAD", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.head_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.head_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.head_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.head_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.head_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.head_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.head_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.head_speed_entry.pack(side="left")
        self.head_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.head_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.head_accel_entry.pack(side="left")
        self.head_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Head'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))
        self.head_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Head'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.head_idle_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.head_pos_btn = tk.Button(state_row1, text="Position", width=10,
                                      command=lambda: self.position_state('Head'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        self.head_pos_btn.pack(side="left", pady=(0, 5), padx=(0, 5))

        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))
        self.head_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Head'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)
        self.head_reset_btn.pack(side="left", padx=(0, 5))
        self.head_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                            command=lambda: self.error_get('Head'), font=("Arial", 10, "bold"))
        self.head_get_error_btn.pack(side="left", padx=(0, 20))

        control_frame = tk.Frame(error_handle_frame, bg='white')
        control_frame.pack(fill="x")
        self.head_brake_btn = tk.Button(control_frame, text="Brake", width=10, command=lambda :self.brake('Head'), font=("Arial", 10, "bold"))
        self.head_brake_btn.pack(side="left", padx=(0, 5))
        self.head_unbrake_btn = tk.Button(control_frame, text="UnBrake", width=10, command=lambda :self.release_brake('Head'),
                                          font=("Arial", 10, "bold"))
        self.head_unbrake_btn.pack(side="left")

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="y")
        right_frame.pack_propagate(False)

        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="Pos(J1~J3):", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.head_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.head_pos_text.tag_configure("center", justify='center')
        self.head_pos_text.pack(side="left")
        self.head_pos_text.insert("1.0", "0.000,0.000,0.000")
        self.head_pos_text.tag_add("center", "1.0", "end")
        self.head_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.head_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Head'))
        self.head_get_btn.pack(side="left", padx=(0, 5))
        self.head_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.head_cmd_entry.insert(0, "0,0,0")
        self.head_cmd_entry.pack(side="left", padx=(5, 5))
        self.head_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Head'))
        self.head_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.head_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Head'))
        self.head_del_btn.pack(side="left", padx=(0, 5))
        self.head_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.head_combo.pack(side="left", padx=(0, 5))
        self.head_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Head'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.head_run_btn.pack(side="left", padx=(0, 5))

        self.head_points = []
        self.head_combo['values'] = []

    # ==================== Lift Component ====================
    def create_lift_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        content = tk.Frame(container, bg="white")
        content.pack(fill="x")

        # Left status
        left_status_frame = tk.Frame(content, bg="white", width=arm_main_state_with)
        left_status_frame.pack(side="left", fill="y", padx=(0, 10))
        left_status_frame.pack_propagate(False)

        status_title_frame = tk.Frame(left_status_frame, bg="white")
        status_title_frame.pack(fill="x", pady=(0, 10))
        tk.Label(status_title_frame, text="LIFT", font=('Arial', 11, 'bold'), fg='#2c3e50', bg="white").pack(anchor="w",
                                                                                                             padx=40,
                                                                                                             pady=(0,
                                                                                                                   5))

        status_info_frame = tk.Frame(left_status_frame, bg="white")
        status_info_frame.pack(fill="both", expand=True, anchor="nw")

        row1 = tk.Frame(status_info_frame, bg="white")
        row1.pack(anchor="w", pady=(0, 5))
        tk.Label(row1, text="Status:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.lift_state_main = tk.Label(row1, text='IDLE', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.lift_state_main.pack(side="left")

        row2 = tk.Frame(status_info_frame, bg="white")
        row2.pack(anchor="w", pady=(0, 5))
        tk.Label(row2, text="Error:", font=('Arial', 9), fg='#2c3e50', width=6,
                 bg="white").pack(side="left", padx=(0, 5))
        self.lift_error_code = tk.Label(row2, text='None', font=('Arial', 9),
                                        fg='#34495e', bg='white', width=15, pady=3,
                                        relief=tk.SUNKEN, bd=1)
        self.lift_error_code.pack(side="left")

        row3 = tk.Frame(status_info_frame, bg="white")
        row3.pack(fill="x", pady=(0, 5))
        self.lift_error_detail = tk.Label(row3, text="", font=('Arial', 9),
                                          fg='#2c3e50', bg='white', pady=5,
                                          anchor='w', wraplength=100, justify='left')
        self.lift_error_detail.pack(fill="x", padx=5)

        # Middle area
        middle_frame = tk.Frame(content, bg="white", width=300)
        middle_frame.pack(side="left", fill="both", expand=True, padx=(0, 15))

        param_frame = ttk.LabelFrame(middle_frame, text="Parameters", padding=10, relief=tk.GROOVE, borderwidth=2,
                                     style="MyCustom.TLabelframe")
        param_frame.pack(fill="x", pady=(0, 10))
        param_row = tk.Frame(param_frame, bg="white")
        param_row.pack(fill="x")
        tk.Label(param_row, text="Speed:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.lift_speed_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.lift_speed_entry.pack(side="left")
        self.lift_speed_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        tk.Label(param_row, text="Accel:", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 2))
        self.lift_accel_entry = tk.Entry(param_row, width=5, font=('Arial', 9), justify='center')
        self.lift_accel_entry.pack(side="left")
        self.lift_accel_entry.insert(0, "20")
        tk.Label(param_row, text="1%-100%", font=('Arial', 9), bg='white').pack(side="left", padx=(0, 5))
        speed_btn = tk.Button(param_row, text="Confirm Speed", width=15, command=lambda: self.vel_acc_set('Lift'),
                              bg="#58C3EE", font=("Arial", 9, "bold"))
        speed_btn.pack(side="left", padx=(0, 20))

        top_mid = tk.Frame(middle_frame, bg="white")
        top_mid.pack(fill="x", pady=(0, 5))
        state_switch_frame = ttk.LabelFrame(top_mid, text="Status switching", padding=10, relief=tk.GROOVE,
                                            borderwidth=2, style="MyCustom.TLabelframe")
        state_switch_frame.pack(side="left", fill="both", expand=True, padx=(0, 5))

        state_row1 = tk.Frame(state_switch_frame, bg="white")
        state_row1.pack(fill="x", pady=(0, 5))
        self.lift_idle_btn = tk.Button(state_row1, text="IDLE", width=10, command=lambda: self.idle_state('Lift'),
                                       bg="#2196F3", fg="white", font=("Arial", 10, "bold"))
        self.lift_idle_btn.pack(side="left", pady=(0, 5), padx=(0, 5))
        self.lift_pos_btn = tk.Button(state_row1, text="Position", width=10,
                                      command=lambda: self.position_state('Lift'), bg="#9fd4cf", fg="black",
                                      font=("Arial", 10, "bold"))
        self.lift_pos_btn.pack(side="left", pady=(0, 5), padx=(0, 5))

        error_handle_frame = ttk.LabelFrame(top_mid, text="Error Handling", padding=10, relief=tk.GROOVE, borderwidth=2,
                                            style="MyCustom.TLabelframe")
        error_handle_frame.pack(side="left", fill="both", expand=True)

        servo_frame = tk.Frame(error_handle_frame, bg="white")
        servo_frame.pack(fill="x", pady=(0, 10))
        self.lift_reset_btn = tk.Button(servo_frame, text="Reset", width=10, command=lambda: self.reset_error('Lift'),
                                        bg="#a0ebc8", fg="black", font=("Arial", 10, "bold"), relief=tk.RAISED, bd=2)
        self.lift_reset_btn.pack(side="left", padx=(0, 5))
        self.lift_get_error_btn = tk.Button(servo_frame, text="GetSroErr", width=10,
                                            command=lambda: self.error_get('Lift'), font=("Arial", 10, "bold"))
        self.lift_get_error_btn.pack(side="left", padx=(0, 20))

        # Right area
        right_frame = tk.Frame(content, bg="white", width=650)
        right_frame.pack(side="left", fill="both", expand=True)

        data_frame = ttk.LabelFrame(right_frame, text="Realtime Data", padding=10, relief=tk.GROOVE, borderwidth=2,
                                    style="MyCustom.TLabelframe")
        data_frame.pack(fill="x", pady=(0, 5))
        tk.Label(data_frame, text="Pos(J1):", font=('Arial', 10, 'bold'), bg='white').pack(side="left", padx=(0, 2))
        self.lift_pos_text = tk.Text(data_frame, width=55, height=1, font=('Arial', 9), bg='white', relief=tk.SUNKEN,
                                     bd=1, wrap=tk.NONE)
        self.lift_pos_text.tag_configure("center", justify='center')
        self.lift_pos_text.pack(side="left")
        self.lift_pos_text.insert("1.0", "0.000,0.000")
        self.lift_pos_text.tag_add("center", "1.0", "end")
        self.lift_pos_text.config(state="disabled")

        cmd_frame = ttk.LabelFrame(right_frame, text="Position Cmd", padding=10, relief=tk.GROOVE, borderwidth=2,
                                   style="MyCustom.TLabelframe")
        cmd_frame.pack(fill="x")

        row_cmd1 = tk.Frame(cmd_frame, bg='white')
        row_cmd1.pack(fill="x", pady=(0, 5))
        self.lift_get_btn = tk.Button(row_cmd1, text="GetCurPos", width=8, command=lambda: self.get_current_pos('Lift'))
        self.lift_get_btn.pack(side="left", padx=(0, 5))
        self.lift_cmd_entry = tk.Entry(row_cmd1, width=45)
        self.lift_cmd_entry.insert(0, "0,0")
        self.lift_cmd_entry.pack(side="left", padx=(5, 5))
        self.lift_add_btn = tk.Button(row_cmd1, text="Add", width=8, command=lambda: self.add_pos('Lift'))
        self.lift_add_btn.pack(side="left", padx=(20, 5))

        row_cmd2 = tk.Frame(cmd_frame, bg='white')
        row_cmd2.pack(fill="x", pady=(0, 5))
        self.lift_del_btn = tk.Button(row_cmd2, text="Delete", width=8, command=lambda: self.delete_pos('Lift'))
        self.lift_del_btn.pack(side="left", padx=(0, 5))
        self.lift_combo = ttk.Combobox(row_cmd2, state="readonly", width=45)
        self.lift_combo.pack(side="left", padx=(0, 5))
        self.lift_run_btn = tk.Button(row_cmd2, text="Run", width=8, command=lambda: self.run_pos('Lift'),
                                      font=("Arial", 11, "bold"), fg='white', bg='#EC2A23', border=5)
        self.lift_run_btn.pack(side="left", padx=(0, 5))

        self.lift_points = []
        self.lift_combo['values'] = []

    # ==================== User Feedback ====================
    def create_user_fbk_components(self):
        container = tk.Frame(self.scrollable_frame, bg="white", pady=5)
        container.pack(fill="x", pady=(0, 5))

        frame = ttk.LabelFrame(container, text="User Feedback", padding=10, relief=tk.GROOVE,
                               borderwidth=2, style="MyCustom.TLabelframe")
        frame.pack(fill="x")

        # ---- set row ----
        set_row = tk.Frame(frame, bg="white")
        set_row.pack(fill="x", pady=(0, 8))
        tk.Label(set_row, text="chn_id:", bg="white").pack(side="left", padx=(0, 3))
        self.user_fbk_chn_var = tk.StringVar(value="0")
        self.user_fbk_chn_combo = ttk.Combobox(
            set_row, textvariable=self.user_fbk_chn_var, state="readonly",
            width=4, font=("Arial", 9))
        self.user_fbk_chn_combo['values'] = ["0", "1", "2", "3"]
        self.user_fbk_chn_combo.current(0)
        self.user_fbk_chn_combo.pack(side="left", padx=(0, 15))
        tk.Label(set_row, text="fbk_type:", bg="white").pack(side="left", padx=(0, 3))

        # Build the FXUserFbkType value -> label map from the enum.
        # Enum members are named like USER_FBK_*; tolerate an optional FX_ prefix
        # so both the old and current naming conventions are picked up.
        self._user_fbk_type_map = {}  # {value:int -> label:str}
        for name in dir(FXUserFbkType):
            if name.startswith("USER_FBK_"):
                val = int(getattr(FXUserFbkType, name))
                self._user_fbk_type_map[val] = f"{val}: {name}"
        # Sort by value for a stable, readable dropdown.
        fbk_values = [self._user_fbk_type_map[v] for v in sorted(self._user_fbk_type_map)]

        self.user_fbk_type_var = tk.StringVar()
        self.user_fbk_type_combo = ttk.Combobox(
            set_row, textvariable=self.user_fbk_type_var, state="readonly",
            width=60, font=("Arial", 9))
        self.user_fbk_type_combo['values'] = fbk_values
        # Default to Arm0 FFD torque.
        default_label = self._user_fbk_type_map.get(
            int(FXUserFbkType.USER_FBK_ARM0_INTERNAL_FFDTOR), fbk_values[0])
        self.user_fbk_type_var.set(default_label)
        self.user_fbk_type_combo.pack(side="left", padx=(0, 15))
        tk.Button(set_row, text="Set", width=8, bg="#A2CD5A",
                  command=self.set_user_fbk).pack(side="left")

        # ---- display rows: 4 feedback channels ----
        self.user_fbk_disps = []
        for i in range(4):
            row = tk.Frame(frame, bg="white")
            row.pack(fill="x", pady=2)
            tk.Label(row, text=f"chn{i}:", bg="white", width=6).pack(side="left", padx=(0, 5))
            disp = tk.Entry(row, width=70, justify="left", state="readonly")
            disp.pack(side="left", fill="x", expand=True)
            self.user_fbk_disps.append(disp)

    def set_user_fbk(self):
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            chn_id = int(self.user_fbk_chn_var.get().strip())
        except ValueError:
            messagebox.showerror('Error', 'chn_id must be an integer')
            return
        if chn_id < 0 or chn_id > 3:
            messagebox.showerror('Error', 'chn_id must be 0-3')
            return
        # Extract the integer value from the selected dropdown label ("<val>: <NAME>").
        sel = self.user_fbk_type_var.get()
        try:
            fbk_type = int(sel.split(':', 1)[0].strip())
        except (ValueError, IndexError):
            messagebox.showerror('Error', 'Invalid fbk_type selection')
            return
        ret = robot.system_set_user_fbk_type(chn_id, fbk_type)
        if ret != 0:
            messagebox.showerror('Failed', f'set user fbk type failed. Error msg: {robot._get_operate_error_msg(ret)}')
        else:
            messagebox.showinfo('OK', f'chn{chn_id} set to fbk_type {fbk_type} ({self._user_fbk_type_map.get(fbk_type, "")})')

           
            
    # ==================== Control Panel ====================
    def create_control_components(self):
        """Create the top control panel"""
        self.control_frame = tk.Frame(self.root, bg="#e0e0e0", pady=5)
        self.control_frame.pack(fill="x")

        # Connect button
        self.connect_btn = tk.Button(
            self.control_frame,
            text="Connect Robot",
            width=15,
            command=self.toggle_connection,
            bg="#4CAF50",
            fg="white",
            font=("Arial", 10, "bold"))
        self.connect_btn.pack(side="left", padx=5)

        self.arm_ip_entry = tk.Entry(self.control_frame)
        self.arm_ip_entry.insert(0, "6.6.7.190")
        self.arm_ip_entry.pack(side="left", padx=5)

        # more func
        self.more_features_btn = tk.Button(
            self.control_frame,
            text="More Features",
            width=15,
            command=self.show_more_features,
            bg="#3BA4FD",
            fg="white",
            font=("Arial", 10, "bold")
        )
        self.more_features_btn.pack(side="right", padx=5)

        # Estop
        self.mode_btn = tk.Button(
            self.control_frame,
            text="EmergencyStop",
            width=20,
            command=self.Estop,
            bg="#ebfa1e",
            fg="black",
            font=("Arial", 14, "bold"))
        self.mode_btn.pack(side="right", padx=5)

        self.mode_combo = ttk.Combobox(
            self.control_frame,
            values=self.mode_names,
            state="readonly",
            width=13,
            font=("Arial", 10)
        )
        self.mode_combo.current(self.display_mode)
        self.mode_combo.bind("<<ComboboxSelected>>", self.on_mode_selected)
        self.mode_combo.pack(side="right", padx=5)

        # Status indicator
        status_frame = tk.Frame(self.control_frame, bg="#e0e0e0")
        status_frame.pack(side="right", padx=5)
        self.status_light = tk.Label(status_frame, text="●", font=("Arial", 16), fg="red")
        self.status_light.pack(side="left", padx=5)
        self.status_label = tk.Label(status_frame, text="disconnected", bg="#e0e0e0", font=("Arial", 9))
        self.status_label.pack(side="left")
        tk.Label(status_frame, text="Realtime data:", bg="#3BA4FD", fg="white", font=("Arial", 13, 'bold')).pack(
            side="right", padx=(20, 0))

    def create_status_bar(self):
        """Create the bottom status bar with log on a separate row"""
        self.status_bar = tk.Frame(self.root, height=210)
        self.status_bar.pack(side="bottom", fill="x")
        self.status_bar.pack_propagate(False)

        self.status_bar.columnconfigure(0, weight=1) 
        self.status_bar.columnconfigure(1, weight=0)
        self.status_bar.columnconfigure(2, weight=0)
        self.status_bar.columnconfigure(3, weight=1) 

        # ==========Row 0==========
        self.which_robot_label = tk.Label(
            self.status_bar, text="Robot type:", fg="black", font=("Arial", 9))
        self.which_robot_label.grid(row=0, column=0, sticky="w", padx=(15, 10))

        self.version_label = tk.Label(
            self.status_bar, text="Controller version:", fg="black", font=("Arial", 9))
        self.version_label.grid(row=0, column=1, sticky="w", padx=(0, 10))

        self.sdk_version_label = tk.Label(
            self.status_bar, text=f"SDK version:{self.sdk_version}", fg="black", font=("Arial", 9))
        self.sdk_version_label.grid(row=0, column=2, sticky="w", padx=(0, 50))

        self.time_label = tk.Label(
            self.status_bar, text="", fg="black", font=("Arial", 9))
        self.time_label.grid(row=0, column=3, sticky="e", padx=15)
        self.update_time()  

        self.log_title_label = tk.Label(
            self.status_bar, text="Controller Log:", fg="black", font=("Arial", 9, "bold"))
        self.log_title_label.grid(row=1, column=0, columnspan=4, sticky="w", padx=(15, 0))

        # 5 vertically stacked labels showing the latest 5 controller messages
        self.log_labels = []
        for i in range(5):
            lbl = tk.Label(self.status_bar, text="", fg="black", font=("Arial", 9),
                           anchor="w", justify="left")
            lbl.grid(row=2 + i, column=0, columnspan=4, sticky="ew", padx=(25, 5), pady=(0, 0))
            self.log_labels.append(lbl)

        # Two dropdowns: latest 5 errors and latest 5 warnings, stacked vertically
        self.error_log_var = tk.StringVar(value="Errors: (none)")
        self.error_log_combo = ttk.Combobox(
            self.status_bar, textvariable=self.error_log_var,
            state="readonly", font=("Arial", 9), width=60)
        self.error_log_combo['values'] = ["ERRO: (none)"]
        self.error_log_combo.grid(row=7, column=0, columnspan=4, sticky="ew", padx=(15, 5), pady=(2, 2))
        self.error_log_combo.current(0)

        self.warning_log_var = tk.StringVar(value="WARN: (none)")
        self.warning_log_combo = ttk.Combobox(
            self.status_bar, textvariable=self.warning_log_var,
            state="readonly", font=("Arial", 9), width=60)
        self.warning_log_combo['values'] = ["WARN: (none)"]
        self.warning_log_combo.grid(row=8, column=0, columnspan=4, sticky="ew", padx=(15, 5), pady=(2, 2))
        self.warning_log_combo.current(0)
        self.update_log()
    # ==================== Core Control Methods (Adapted to new MarvinRobot API) ====================
    def init_kd_variables(self):
        self.cart_k_b_entry = tk.StringVar(value="3000,3000,3000,100,100,100,20")
        self.cart_k_a_entry = tk.StringVar(value="3000,3000,3000,100,100,100,20")
        self.cart_d_a_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")
        self.cart_d_b_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")

        self.k_a_entry = tk.StringVar(value="3,3,3,2,1,1,1")
        self.k_b_entry = tk.StringVar(value="3,3,3,2,1,1,1")
        self.d_a_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")
        self.d_b_entry = tk.StringVar(value="0.2,0.2,0.2,0.2,0.2,0.2,0.2")

        self.pdp_entry = tk.StringVar(value="28,26,28,14,6,4,0")
        self.pdd_entry = tk.StringVar(value="5.5,3.7,2.0,2.2,2.0,0.6,0")

        # Arm0 / Arm1 PD parameters (7 elements each)
        self.pdp_a_entry = tk.StringVar(value="14,14,14,10.5,5.6,5.6,5.6")
        self.pdd_a_entry = tk.StringVar(value="0.3,0.3,0.3,0.3,0.3,0.3,0.3")
        self.pdp_b_entry = tk.StringVar(value="14,14,14,10.5,5.6,5.6,5.6")
        self.pdd_b_entry = tk.StringVar(value="0.3,0.3,0.3,0.3,0.3,0.3,0.3")

        self.hand0_kp_entry=tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand0_kd_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand0_tor_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand1_kp_entry=tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand1_kd_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')
        self.hand1_tor_entry = tk.StringVar(value='0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0')

        self.force_a_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_a_entry = tk.StringVar(value="0,1,0,5,10")
        self.force_b_entry = tk.StringVar(value="0,1,0,25,25")
        self.torque_b_entry = tk.StringVar(value="0,1,0,5,10")

        self.arm0_tool_dyn_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        self.arm1_tool_dyn_entry = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")

        self.arm0_tool_kine_entry = tk.StringVar(value="0,0,0,0,0,0")
        self.arm1_tool_kine_entry = tk.StringVar(value="0,0,0,0,0,0")


        self.ref_ori_a_entry = tk.StringVar(value="0.0,0.0,0.0")
        self.ref_ori_b_entry = tk.StringVar(value="0.0,0.0,0.0")

        # reference orientation type, backed by StringVar so the value is always
        # readable even before the impedance dialog (which creates the combobox
        # widgets) has been opened.
        self.refori_var_l = tk.StringVar(value="disable")
        self.refori_var_r = tk.StringVar(value="disable")

    def kine_initial(self):
        if robot.init_single_arm_config(0) !=0:
            print("[ERROR] Failed to initialize arm0 kinematics")
            return -1
        if robot.init_single_arm_config(1) !=0:
            print("[ERROR] Failed to initialize arm1 kinematics")
            return -1
        print("\narms kinematics initialized")
        robot.kine_log_level(FXLogMask.FX_LOG_DEBG_FLAG)
        return 0

    def toggle_connection(self):
        def validate_and_parse_ip(ip_str):
            ip_str = ip_str.strip()
            if not ip_str:
                return None
            pattern = re.compile(
                r'^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')
            if not pattern.match(ip_str):
                return None
            ip_parts = [int(x) for x in ip_str.split('.')]
            return ip_parts
        if not self.connected:
            try:
                global_robot_ip = self.arm_ip_entry.get()
                ip_parts = validate_and_parse_ip(global_robot_ip)
                if ip_parts is None or len(ip_parts) != 4:
                    messagebox.showerror("IP error", f"Your input ip: {global_robot_ip}\nplease enter IPv4 address, eg: 6.6.7.190")
                    return
                # if len(ip_parts) != 4:
                #     messagebox.showerror("IP error", "please enter IPv4 address, eg: 6.6.7.190")
                #     return
                if self._connect_robot(ip_parts):
                    self._last_robot_ip = global_robot_ip
            except Exception as e:
                messagebox.showerror('Error', f"Connection failed: {e}")
        else:
            self._disconnect_robot()

    def _connect_robot(self, ip_parts):
        """Link to the robot, run post-connect init and start monitoring.

        Returns True when the connection is established.
        """
        try:
            self.connect_btn.config(state="disabled")
            self.status_label.config(text="Connecting...")
            self.status_light.config(fg="blue")
            self.root.update_idletasks()
            log_mask=FXLogMask.FX_LOG_ALL_FLAG
            ret = robot.link(ip_parts[0], ip_parts[1], ip_parts[2], ip_parts[3],log_mask)
            if ret <0:
                messagebox.showerror('Failed!', f"Robot connection failed. Error msg: {robot._get_operate_error_msg(ret)}")
                self._reset_ui_after_disconnect()
                return False
            else:
                # After link() the data arrival can lag (especially on a
                # reconnect right after a link loss), so poll the link state
                # for up to ~1 s instead of a single immediate check.
                ret_link = 0
                for _ in range(10):
                    ret_link = robot.check_link_state()
                    if ret_link == 1:
                        break
                    time.sleep(0.1)
                if ret_link==1:
                    # Mark connected and update the UI first; auxiliary init
                    # below must not leave the UI showing "disconnected" while
                    # the SDK link is actually alive.
                    self.connected = True
                    self.connect_btn.config(text="Disconnect", bg="#F44336", state="normal")
                    self.status_label.config(text="Connected")
                    self.status_light.config(fg="green")
                    self.mode_btn.config(state="normal")
                    try:
                        self.kine_initial()
                        robot.param_set_int("R.BASIC.LanguageType",1)
                        time.sleep(0.5)
                        self.update_version_control()
                    except Exception as e:
                        print(f"[ERROR] post-connect init failed: {e}")
                    # Stop any managers left over from the previous link
                    # (a reconnect arrives here without a disconnect).
                    if getattr(self, 'data_manager', None):
                        self.data_manager.stop()
                        self.data_manager = None
                    if getattr(self, 'hand_data_manager', None) and self.hand_data_manager.is_running:
                        self.hand_data_manager.stop()
                        if hasattr(self, 'hand_mgr_btn'):
                            self.hand_mgr_btn.config(text="Start Hand Data Manager", bg="#A2CD5A")
                            self.hand_mgr_status.config(text="stopped", fg="gray")
                    self.data_manager = RobotDataManager(robot)
                    self._start_update_data_loop()
                    # Refresh the error/warning dropdowns so they no longer
                    # show the "(not connected)" placeholder after a reconnect.
                    self._reset_log_combos()
                    self.refresh_params_from_realtime_data()
                    self.update_log()
                    self._start_link_monitor()

                elif ret_link==-1:
                    try:
                        robot.unlink()
                    except Exception:
                        pass
                    messagebox.showerror('Linked', f"Link is established, but no data arrived,\n please check the cables and firewall ")
                    self._reset_ui_after_disconnect()
                    return False
                else:
                    try:
                        robot.unlink()
                    except Exception:
                        pass
                    messagebox.showerror('Failed!', f"Link is not established")
                    self._reset_ui_after_disconnect()
                    return False
        except Exception as e:
            messagebox.showerror('Error', f"Connection failed: {e}")
            self._reset_ui_after_disconnect()
            return False
        return True

    def _reset_log_combos(self, errors_text="Errors: (no errors)", warnings_text="Warnings: (no warnings)"):
        """Clear the error/warning histories and refresh the two dropdowns."""
        self._error_log_history = []
        self._warning_log_history = []
        self.error_log_combo['values'] = [errors_text]
        self.error_log_var.set(errors_text)
        self.error_log_combo.current(0)
        self.warning_log_combo['values'] = [warnings_text]
        self.warning_log_var.set(warnings_text)
        self.warning_log_combo.current(0)

    def _disconnect_robot(self):
        """Unlink from the robot and reset the connection UI state."""
        try:
            robot.unlink()
            self.connect_btn.config(state="disabled")
            if hasattr(self, 'data_manager') and self.data_manager:
                self.data_manager.stop()
                self.data_manager = None
            if getattr(self, 'hand_data_manager', None) and self.hand_data_manager.is_running:
                self.hand_data_manager.stop()

            self._stop_link_monitor()
            self.connected = False
            self._stop_update_data_loop()
            self._log_running = False
            self._log_history = []
            for lbl in self.log_labels:
                lbl.config(text="")
            self._reset_log_combos()
            self._reset_ui_after_disconnect()
            self.mode_btn.config(state="disabled")
        except Exception as e:
            messagebox.showerror('Error', f"Disconnect failed: {e}")
            self.connect_btn.config(state="normal")

    def _start_link_monitor(self):
        """Periodically poll check_link_state while connected."""
        self._stop_link_monitor()
        self._link_monitor_id = self.root.after(1000, self._link_monitor_tick)

    def _stop_link_monitor(self):
        if self._link_monitor_id is not None:
            self.root.after_cancel(self._link_monitor_id)
            self._link_monitor_id = None

    def _link_monitor_tick(self):
        if not self.connected:
            self._link_monitor_id = None
            return
        try:
            ret_link = robot.check_link_state()
        except Exception as e:
            ret_link = None
            print(f"[ERROR] check_link_state failed: {e}")
        if ret_link == 1:
            self._link_monitor_id = self.root.after(1000, self._link_monitor_tick)
            return
        # Link lost: pause the polling loop and ask the user what to do.
        self._link_monitor_id = None
        self.status_label.config(text="Link lost")
        self.status_light.config(fg="orange")
        self._show_link_lost_dialog()

    def _show_link_lost_dialog(self):
        answer = messagebox.askquestion(
            'Link lost',
            'Robot connection lost (link state: not connected).\n'
            'Do you want to reconnect?',
            icon='warning')
        if answer == 'yes':
            self._attempt_reconnect()
        else:
            self._disconnect_robot()

    def _attempt_reconnect(self):
        ip_str = self._last_robot_ip.strip()
        pattern = re.compile(
            r'^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')
        if not pattern.match(ip_str):
            messagebox.showerror("IP error", f"Saved ip: {ip_str}\nplease enter IPv4 address, eg: 6.6.7.190")
            self._disconnect_robot()
            return
        ip_parts = [int(x) for x in ip_str.split('.')]
        # Unlink first so the SDK does not hold a stale link while relinking.
        try:
            robot.unlink()
        except Exception:
            pass
        if self._connect_robot(ip_parts):
            self.status_label.config(text="Connected (reconnected)")
        else:
            self._disconnect_robot()

    def _reset_ui_after_disconnect(self):
        self.connect_btn.config(text="Connect Robot", bg="#4CAF50", state="normal")
        self.status_label.config(text="Disconnected")
        self.status_light.config(fg="red")

    def update_version_control(self):
            self.sys_version = robot.get_controller_version()
            self.version_label.config(text=f"System version: {self.sys_version}")
            self.sdk_version_label.config(text=f"SDK version: {self.sdk_version}")
            sub_robot=robot.param_get_string('R.BASIC.SubName')
            if sub_robot[0]==0:
                self.which_robot_label.config(text=f"Robot type: {robot.get_robot_type()}"+ f"    Sub name:{   str(sub_robot[1])}")
            else:
                self.which_robot_label.config(text=f"Robot type: {robot.get_robot_type()}")

            devices_phys = ["Arm0", "Arm1", "Body", "Head", "Lift"]
            obj_types_phys = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1,
                              FXObjType.OBJ_BODY, FXObjType.OBJ_HEAD, FXObjType.OBJ_LIFT]
            if not hasattr(self, 'physical_states'):
                self.physical_states = {}
            state_map = {
                0: "not used",
                1: "virtual",
                2: "real",
            }
            for device, obj_type in zip(devices_phys, obj_types_phys):
                try:
                    ret, state = robot.get_ctrl_obj_physical_state(obj_type)
                    if ret == 0:
                        state_text = state_map.get(state, f"State {state}")
                    else:
                        state_text = f"Failed (ret={robot._get_operate_error_msg(ret)}, details: {robot._get_operate_error_msg(ret)})"

                    self.physical_states[device] = state_text
                except Exception as e:
                    self.physical_states[device] = f"Error: {e}"
                if hasattr(self, 'physical_state_labels') and device in self.physical_state_labels:
                    self.physical_state_labels[device].config(text=self.physical_states[device])

            devices_servo = ["Arm0", "Arm1", "Body", "Head"]
            obj_types_servo = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1,
                               FXObjType.OBJ_BODY, FXObjType.OBJ_HEAD]
            for device, obj_type in zip(devices_servo, obj_types_servo):
                try:
                    ret, fw_versions, cfg_versions = robot.get_ctrl_obj_servo_version(obj_type)
                    if ret < 0:
                        self.servo_versions[device] = f"Error msg: {robot._get_operate_error_msg(ret)}"
                        self.servo_cfg_versions[device] = f"Error msg: {robot._get_operate_error_msg(ret)}"
                        return
                    fw_str = ", ".join(str(v) for v in fw_versions)
                    cfg_str = ", ".join(str(v) for v in cfg_versions)
                    self.servo_versions[device] = fw_str
                    self.servo_cfg_versions[device] = cfg_str
                except Exception as e:
                    self.servo_versions[device] = f"Error: {e}"
                    self.servo_cfg_versions[device] = f"Error: {e}"
                if hasattr(self, 'servo_version_labels') and device in self.servo_version_labels:
                    self.servo_version_labels[device].config(text=self.servo_versions[device])
                if hasattr(self, 'servo_config_version_labels') and device in self.servo_config_version_labels:
                    self.servo_config_version_labels[device].config(text=self.servo_cfg_versions[device])

            devices_sensor = ["Arm0", "Arm1", "Body"]
            obj_types_sensor = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1, FXObjType.OBJ_BODY]
            for device, obj_type in zip(devices_sensor, obj_types_sensor):
                try:
                    ret, ver, serial = robot.get_ctrl_obj_sensor_version_and_serial(obj_type)
                    if ret < 0:
                        self.sensor_versions[device] = f"Error msg: {robot._get_operate_error_msg(ret)}"
                        self.sensor_serials[device]= f"Error msg: {robot._get_operate_error_msg(ret)}"
                        return
                    if isinstance(ver, (list, tuple)):
                        ver_str = ", ".join(str(v) for v in ver)
                    else:
                        ver_str = str(ver)
                    if isinstance(serial, (list, tuple)):
                        serial_str = ", ".join(str(s) for s in serial)
                    else:
                        serial_str = str(serial)

                    self.sensor_versions[device] = ver_str
                    self.sensor_serials[device] = serial_str
                except Exception as e:
                    self.sensor_versions[device] = f"Error: {e}"
                    self.sensor_serials[device] = f"Error: {e}"
                if hasattr(self, 'sensor_version_labels') and device in self.sensor_version_labels:
                    self.sensor_version_labels[device].config(text=self.sensor_versions[device])
                if hasattr(self, 'sensor_serial_labels') and device in self.sensor_serial_labels:
                    self.sensor_serial_labels[device].config(text=self.sensor_serials[device])

    def refresh_params_from_realtime_data(self):
        if not self.connected or not self.data_manager:
            return
        sg_dict = self.data_manager.latest_sg
        rt_dict = self.data_manager.latest_rt
        if not sg_dict or "error" in sg_dict:
            try:
                sg_dict = robot.get_sg_dict()
            except:
                return
        if not rt_dict or "error" in rt_dict:
            try:
                rt_dict = robot.get_rt_dict()
            except:
                return

        def has_nonzero(lst, tol=1e-6):
            return any(abs(v) > tol for v in lst)

        def update_if_valid(var, new_list, default_list):
            # Display the realtime value as-is, including all-zeros. The
            # "Load default parameters" button in the impedance dialog is the
            # only path that should restore defaults, so we never fall back to
            # a stored default here.
            if new_list is not None and len(new_list) > 0:
                var.set(','.join(str(v) for v in new_list))

        # --- Joint K/D tools info for Arm0 and Arm1 ---
        for arm_idx, arm_prefix in enumerate(['a', 'b']):
            arm_data = sg_dict['arms'][arm_idx]['set']
            joint_k = arm_data.get('joint_k', [])
            if joint_k:
                update_if_valid(getattr(self, f'k_{arm_prefix}_entry'), joint_k, None)
            joint_d = arm_data.get('joint_d', [])
            if joint_d:
                update_if_valid(getattr(self, f'd_{arm_prefix}_entry'), joint_d, None)
            cart_k = arm_data.get('cart_k', [])
            if cart_k:
                update_if_valid(getattr(self, f'cart_k_{arm_prefix}_entry'), cart_k, None)
            cart_d = arm_data.get('cart_d', [])
            if cart_d:
                update_if_valid(getattr(self, f'cart_d_{arm_prefix}_entry'), cart_d, None)

            # --- PD K/D (7 values) ---
            pd_k = arm_data.get('pd_k', [])
            if pd_k:
                update_if_valid(getattr(self, f'pdp_{arm_prefix}_entry'), pd_k, None)
            pd_d = arm_data.get('pd_d', [])
            if pd_d:
                update_if_valid(getattr(self, f'pdd_{arm_prefix}_entry'), pd_d, None)

            # --- Tool kinematics (6 values) and dynamics (10 values) ---
            tool_kine = arm_data.get('tool_kine', [])
            if tool_kine:
                update_if_valid(getattr(self, f'arm{arm_idx}_tool_kine_entry'), tool_kine, None)
            tool_dyna = arm_data.get('tool_dyna', [])
            if tool_dyna:
                update_if_valid(getattr(self, f'arm{arm_idx}_tool_dyn_entry'), tool_dyna, None)

        # --- Body PD ---
        # SDK returns 6 values for body pdk/pdd, but the UI entries display 7
        # values (to match load_default_body_pd, e.g. "28,26,28,14,6,4,0").
        # Pad a 0 at the end so both entries always show 7 comma-separated values.
        body_set = sg_dict.get('body', {}).get('set', {})
        body_pdk = body_set.get('pdk', [])
        if body_pdk:
            update_if_valid(self.pdp_entry, list(body_pdk) + [0.0], None)
        body_pdd = body_set.get('pdd', [])
        if body_pdd:
            update_if_valid(self.pdd_entry, list(body_pdd) + [0.0], None)

        # vel & acc — for every control part, take the value straight from the
        # realtime data. Even when the value is 0 we display 0 as-is, so that the
        # "Load default parameters" button in the impedance dialog is the only
        # thing that restores defaults.
        def set_entry(entry, value):
            if value is None:
                return
            entry.delete(0, tk.END)
            entry.insert(0, str(int(value)))

        set_entry(self.left_speed_entry, sg_dict['arms'][0]['set']['vel_ratio'])
        set_entry(self.left_accel_entry, sg_dict['arms'][0]['set']['acc_ratio'])
        set_entry(self.right_speed_entry, sg_dict['arms'][1]['set']['vel_ratio'])
        set_entry(self.right_accel_entry, sg_dict['arms'][1]['set']['acc_ratio'])
        set_entry(self.body_speed_entry, sg_dict['body']['set']['vel_ratio'])
        set_entry(self.body_accel_entry, sg_dict['body']['set']['acc_ratio'])
        set_entry(self.head_speed_entry, sg_dict['head']['set']['vel_ratio'])
        set_entry(self.head_accel_entry, sg_dict['head']['set']['acc_ratio'])
        set_entry(self.lift_speed_entry, sg_dict['lift']['set']['vel_ratio'])
        set_entry(self.lift_accel_entry, sg_dict['lift']['set']['acc_ratio'])

        # force & torque — display the realtime value directly, including 0.
        fb_force_arm0 = rt_dict['arms'][0]['cmd']['force_dir']
        if fb_force_arm0 is not None and len(fb_force_arm0) > 0:
            self.force_a_entry.set(','.join(str(v) for v in fb_force_arm0))
        fb_torque_arm0 = rt_dict['arms'][0]['cmd']['torque_dir']
        if fb_torque_arm0 is not None and len(fb_torque_arm0) > 0:
            self.torque_a_entry.set(','.join(str(v) for v in fb_torque_arm0))

        fb_force_arm1 = rt_dict['arms'][1]['cmd']['force_dir']
        if fb_force_arm1 is not None and len(fb_force_arm1) > 0:
            self.force_b_entry.set(','.join(str(v) for v in fb_force_arm1))
        fb_torque_arm1 = rt_dict['arms'][1]['cmd']['torque_dir']
        if fb_torque_arm1 is not None and len(fb_torque_arm1) > 0:
            self.torque_b_entry.set(','.join(str(v) for v in fb_torque_arm1))

    def _start_update_data_loop(self):
        """(Re)start the UI refresh loop, cancelling any pending tick first.

        Safe to call repeatedly (e.g. on every reconnect): the previous scheduled
        tick is cancelled so loops never stack up.
        """
        if getattr(self, '_update_data_id', None) is not None:
            try:
                self.root.after_cancel(self._update_data_id)
            except Exception:
                pass
            self._update_data_id = None
        self._update_data_loop_running = True
        self.update_data()

    def _stop_update_data_loop(self):
        """Cancel the pending refresh tick and clear the running flag."""
        self._update_data_loop_running = False
        if getattr(self, '_update_data_id', None) is not None:
            try:
                self.root.after_cancel(self._update_data_id)
            except Exception:
                pass
            self._update_data_id = None

    def update_data(self):
        # The next tick MUST always be rescheduled, even if update_ui() raises.
        # Otherwise a single transient exception (e.g. a None kinematics result
        # or an unexpected state value) kills the loop permanently: the UI stops
        # refreshing while the rest of the app stays responsive.
        if not self.connected:
            self._update_data_loop_running = False
            self._update_data_id = None
            return
        try:
            if self.data_manager:
                self.rt = self.data_manager.latest_rt
                self.sg = self.data_manager.latest_sg
                self.update_ui()
        except Exception as e:
            print(f"[update_data] exception: {e!r}")
        finally:
            if self.connected:
                self._update_data_id = self.root.after(200, self.update_data)
            else:
                self._update_data_loop_running = False
                self._update_data_id = None

    def update_ui(self):
        if self.rt is None or self.sg is None:
            return
        joint_pos_l = ''
        joint_pos_r = ''
        body_pos = ''
        head_pos = ''
        lift_pos = ''
        hand0_pos=''
        hand1_pos=''
        key = self.data_keys[self.display_mode]

        # # ==================== HANDS ====================
        # hand0_state = self.rt["hands"][0]["fb"]["state"]
        # if hand0_state==FXHandState.FX_HAND_STATE_ERROR:
        #     self.hand0_state_main.config(text=f"Error")
        # elif hand0_state==FXHandState.FX_HAND_STATE_ENABLED:
        #     self.hand0_state_main.config(text=f"Enable")
        # elif hand0_state==FXHandState.FX_HAND_STATE_DISABLED:
        #     self.hand0_state_main.config(text=f"Disable")
        # hand1_state = self.rt["hands"][1]["fb"]["state"]
        # if hand1_state == FXHandState.FX_HAND_STATE_ERROR:
        #     self.hand1_state_main.config(text=f"Error")
        # elif hand1_state==FXHandState.FX_HAND_STATE_ENABLED:
        #     self.hand1_state_main.config(text=f"Enable")
        # elif hand1_state==FXHandState.FX_HAND_STATE_DISABLED:
        #     self.hand1_state_main.config(text=f"Disable")

        # if key in self.hand_rt_key:
        #     hand0_pos = self.rt["hands"][0]["fb"][key]
        #     hand1_pos = self.rt["hands"][1]["fb"][key]
        # if key in self.hand_sg_key:
        #     hand0_pos = self.sg["hands"][0]["get"][key]
        #     hand1_pos = self.sg["hands"][0]["get"][key]
        # pos_text_hand0 = ", ".join(f"{v:.3f}" for v in hand0_pos)
        # self.hand0_pos_text.config(state="normal")
        # self.hand0_pos_text.delete("1.0", tk.END)
        # self.hand0_pos_text.insert("1.0", pos_text_hand0)
        # self.hand0_pos_text.tag_add("center", "1.0", "end")
        # self.hand0_pos_text.config(state="disabled")

        # pos_text_hand1 = ", ".join(f"{v:.3f}" for v in hand1_pos)
        # self.hand1_pos_text.config(state="normal")
        # self.hand1_pos_text.delete("1.0", tk.END)
        # self.hand1_pos_text.insert("1.0", pos_text_hand1)
        # self.hand1_pos_text.tag_add("center", "1.0", "end")
        # self.hand1_pos_text.config(state="disabled")

        # ==================== ARM0 ====================
        cur_state = robot.current_state(FXObjType.OBJ_ARM0)
        self.left_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        if self.sg['arms'][0]['get']['tip_di'] == 1:
            self.left_state_1.config(text=f"Dragging")
        else:
            self.left_state_1.config(text=f"Drag off")
        if self.sg['arms'][0]['get']['low_speed_flag'] == 0:
            self.left_state_2.config(text=f"Moving")
        else:
            self.left_state_2.config(text=f"Stopped")

        arm_err = self.rt['arms'][0]['state']['err']
        self.left_state_3.config(text=f"{arm_err}")
        # Display error description using error_dict
        if arm_err != 0 and arm_err in error_dict:
            self.left_arm_error.config(text=f"Error {arm_err}: {error_dict[arm_err]}")
        else:
            self.left_arm_error.config(text="")

        if key in self.arm_rt_key:
            joint_pos_l = self.rt["arms"][0]["fb"][key]
        if key in self.arm_sg_key:
            joint_pos_l = self.sg["arms"][0]["get"][key]
        joint_text_l = ", ".join(f"{v:.3f}" for v in joint_pos_l)
        self.left_joint_text.config(state="normal")
        self.left_joint_text.delete("1.0", tk.END)
        self.left_joint_text.insert("1.0", joint_text_l)
        self.left_joint_text.tag_add("center", "1.0", "end")
        self.left_joint_text.config(state="disabled")

        arm0_joints = robot.forward_kinematics(0, self.rt["arms"][0]["fb"]['fb_pos'])
        arm0_xyzabc = robot.matrix2xyzabc(arm0_joints)
        arm0_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm0_xyzabc)
        self.left_pose_text.config(state="normal")
        self.left_pose_text.delete("1.0", tk.END)
        self.left_pose_text.insert("1.0", f"{arm0_xyzabc_text}")
        self.left_pose_text.tag_add("center", "1.0", "end")
        self.left_pose_text.config(state="disabled")

        # ==================== ARM1 ====================
        cur_state = robot.current_state(FXObjType.OBJ_ARM1)
        self.right_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        if self.sg['arms'][1]['get']['tip_di'] == 1:
            self.right_state_1.config(text=f"Dragging")
        else:
            self.right_state_1.config(text=f"Drag off")

        if self.sg['arms'][1]['get']['low_speed_flag'] == 0:
            self.right_state_2.config(text=f"Moving")
        else:
            self.right_state_2.config(text=f"Stopped")
        arm_err_r = self.rt['arms'][1]['state']['err']
        self.right_state_3.config(text=f"{arm_err_r}")
        if arm_err_r != 0 and arm_err_r in error_dict:
            self.right_arm_error.config(text=f"Error {arm_err_r}: {error_dict[arm_err_r]}")
        else:
            self.right_arm_error.config(text="")

        if key in self.arm_rt_key:
            joint_pos_r = self.rt["arms"][1]["fb"][key]
        if key in self.arm_sg_key:
            joint_pos_r = self.sg["arms"][1]["get"][key]
        joint_text_r = ", ".join(f"{v:.3f}" for v in joint_pos_r)

        self.r_joint_text.config(state="normal")
        self.r_joint_text.delete("1.0", tk.END)
        self.r_joint_text.insert("1.0", joint_text_r)
        self.r_joint_text.tag_add("center", "1.0", "end")
        self.r_joint_text.config(state="disabled")

        arm1_joints = robot.forward_kinematics(1, self.rt["arms"][1]["fb"]['fb_pos'])
        arm1_xyzabc = robot.matrix2xyzabc(arm1_joints)
        arm1_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm1_xyzabc)
        self.right_pose_text.config(state="normal")
        self.right_pose_text.delete("1.0", tk.END)
        self.right_pose_text.insert("1.0", f"{arm1_xyzabc_text}")
        self.right_pose_text.tag_add("center", "1.0", "end")
        self.right_pose_text.config(state="disabled")

        # ==================== BODY ====================
        cur_state = robot.current_state(FXObjType.OBJ_BODY)
        self.body_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        body_err = self.rt['body']['state']['err']
        self.body_error_code.config(text=f"{body_err}")
        if body_err != 0 and body_err in error_dict:
            self.body_error_detail.config(text=f"Error {body_err}: {error_dict[body_err]}")
        else:
            self.body_error_detail.config(text="")

        if key in self.body_rt_key:
            body_pos = self.rt["body"][key]
        if key in self.body_sg_key:
            body_pos = self.sg["body"]["get"][key]
        body_text = ", ".join(f"{v:.3f}" for v in body_pos)
        self.body_pos_text.config(state="normal")
        self.body_pos_text.delete("1.0", tk.END)
        self.body_pos_text.insert("1.0", body_text)
        self.body_pos_text.tag_add("center", "1.0", "end")
        self.body_pos_text.config(state="disabled")

        # ==================== HEAD ====================
        cur_state = robot.current_state(FXObjType.OBJ_HEAD)
        self.head_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        head_err = self.rt['head']['state']['err']
        self.head_error_code.config(text=f"{head_err}")
        if head_err != 0 and head_err in error_dict:
            self.head_error_detail.config(text=f"Error {head_err}: {error_dict[head_err]}")
        else:
            self.head_error_detail.config(text="")

        if key in self.head_rt_key:
            head_pos = self.rt["head"][key]
        if key in self.head_sg_key:
            head_pos = self.sg["head"]["get"][key]
        head_text = ", ".join(f"{v:.3f}" for v in head_pos)
        self.head_pos_text.config(state="normal")
        self.head_pos_text.delete("1.0", tk.END)
        self.head_pos_text.insert("1.0", head_text)
        self.head_pos_text.tag_add("center", "1.0", "end")
        self.head_pos_text.config(state="disabled")

        # ==================== LIFT ====================
        cur_state = robot.current_state(FXObjType.OBJ_LIFT)
        self.lift_state_main.config(text=state_map.get(cur_state, str(cur_state)))

        lift_err = self.rt['lift']['state']['err']
        self.lift_error_code.config(text=f"{lift_err}")
        if lift_err != 0 and lift_err in error_dict:
            self.lift_error_detail.config(text=f"Error {lift_err}: {error_dict[lift_err]}")
        else:
            self.lift_error_detail.config(text="")

        if key in self.lift_rt_key:
            lift_pos = self.rt["lift"][key]
        if key in self.lift_sg_key:
            lift_pos = self.sg["lift"]["get"][key]
        lift_text = ", ".join(f"{v:.3f}" for v in lift_pos)
        self.lift_pos_text.config(state="normal")
        self.lift_pos_text.delete("1.0", tk.END)
        self.lift_pos_text.insert("1.0", lift_text)
        self.lift_pos_text.tag_add("center", "1.0", "end")
        self.lift_pos_text.config(state="disabled")

        # ==================== USER FEEDBACK ====================
        try:
            ufb = self.rt.get("system_user_fbk")
            if ufb and hasattr(self, "user_fbk_disps"):
                for i in range(4):
                    vals = ", ".join(f"{v:.3f}" for v in ufb["chn"][i])
                    e = self.user_fbk_disps[i]
                    e.config(state="normal")
                    e.delete(0, tk.END)
                    e.insert(0, vals)
                    e.config(state="readonly")
        except Exception:
            pass

    def on_mode_selected(self, event=None):
        selected = self.mode_combo.get()
        self.display_mode = self.mode_names.index(selected)
        self.update_ui()

    def update_time(self):
        # Reschedule in a finally block so the clock keeps ticking even if a
        # label update raises (e.g. during teardown).
        try:
            current_time = time.strftime("%Y-%m-%d %H:%M:%S")
            self.time_label.config(text=current_time)
        except Exception as e:
            print(f"[update_time] exception: {e!r}")
        finally:
            try:
                self.root.after(1000, self.update_time)
            except Exception:
                pass

    def update_log(self):
        """Start a background thread that continuously reads system messages.

        The latest 5 messages are shown in vertically stacked labels. Messages
        tagged "[ERROR]" or "[WARNING]" by the controller are also mirrored
        into two separate dropdowns (latest 5 each).
        """
        if self._log_running:
            return
        self._log_running = True

        def _push_log(msg: str):
            """Insert a message at the front and refresh the 5 stacked labels."""
            self._log_history.insert(0, msg.strip())
            del self._log_history[5:]
            for i, lbl in enumerate(self.log_labels):
                text = self._log_history[i] if i < len(self._log_history) else ""
                lbl.config(text=text)

        def _show_idle():
            """Show a placeholder so the panel is visibly alive when idle."""
            if not self._log_history:
                self.log_labels[0].config(text="(no new messages)")
                for lbl in self.log_labels[1:]:
                    lbl.config(text="")

        def _push_error(msg: str):
            self._error_log_history.insert(0, msg.strip())
            del self._error_log_history[5:]
            # Show only the latest error in the dropdown text.
            self.error_log_var.set(self._error_log_history[0])

        def _push_warning(msg: str):
            self._warning_log_history.insert(0, msg.strip())
            del self._warning_log_history[5:]
            # Show only the latest warning in the dropdown text.
            self.warning_log_var.set(self._warning_log_history[0])

        def _poll_log():
            while self._log_running:
                try:
                    ret, msg = robot.fbk_get_system_msg()
                    if msg:
                        self.root.after(0, lambda m=msg: _push_log(m))
                        if "[ERRO]" in msg:
                            self.root.after(0, lambda m=msg: _push_error(m))
                        elif "[WARN]" in msg:
                            self.root.after(0, lambda m=msg: _push_warning(m))
                    else:
                        # ret==0, msg empty -> controller has nothing to report.
                        # Show an idle placeholder every few seconds so the panel
                        # is visibly alive instead of blank.
                        self.root.after(0, _show_idle)
                except Exception as e:
                    # Surface the error instead of swallowing it silently.
                    print(f"[update_log] exception: {e!r}")
                time.sleep(0.2)

        t = threading.Thread(target=_poll_log, daemon=True)
        t.start()

    def on_mousewheel(self, event):
        self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def on_close(self):
        if messagebox.askokcancel("Exit", "Are you sure you want to exit the application?"):
            self._stop_link_monitor()
            self._stop_update_data_loop()
            self._log_running = False
            if getattr(self, 'data_manager', None):
                self.data_manager.stop()
                self.data_manager = None
            if getattr(self, 'hand_data_manager', None) and self.hand_data_manager.is_running:
                self.hand_data_manager.stop()
            self.root.destroy()
            robot.unlink()

    def hand_p_d_torq_set(self,obj):
        try:
            if obj == 'Hand0':
                kp = float(self.hand0_kp_entry.get())
                kd = float(self.hand0_kd_entry.get())
                tor = float(self.hand0_tor_entry.get())
                ret = robot.runtime_set_hand_p(FXHandType.FX_HAND_LEFT, kp)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kp failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_d(FXHandType.FX_HAND_LEFT, kd)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kd failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_max_tor(FXHandType.FX_HAND_LEFT, tor)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set max torque failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Hand1':
                kp = float(self.hand1_kp_entry.get())
                kd = float(self.hand1_kd_entry.get())
                tor= float(self.hand1_tor_entry.get())
                ret = robot.runtime_set_hand_p(FXHandType.FX_HAND_RIGHT, kp)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kp failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_d(FXHandType.FX_HAND_RIGHT, kd)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set kd failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_hand_max_tor(FXHandType.FX_HAND_RIGHT, tor)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set max torque failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                raise ValueError(f"Unknown obj: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def hand_disable(self,obj):
        try:
            if obj=='Hand0':
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_LEFT, FXHandAction.FX_HAND_ACTION_DISABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand0 disable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj=="Hand1":
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_RIGHT, FXHandAction.FX_HAND_ACTION_DISABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand1 disable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def hand_enable(self, obj):
        try:
            if obj == 'Hand0':
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_LEFT, FXHandAction.FX_HAND_ACTION_ENABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand0 enable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == "Hand1":
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_RIGHT, FXHandAction.FX_HAND_ACTION_ENABLE)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand1 enable failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def hand_reset(self, obj):
        try:
            if obj == 'Hand0':
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_LEFT, FXHandAction.FX_HAND_ACTION_RESET)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand0 reset failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == "Hand1":
                ret = robot.runtime_set_hand_action(FXHandType.FX_HAND_RIGHT, FXHandAction.FX_HAND_ACTION_RESET)
                if ret != 0:
                    messagebox.showerror('Error', f"Set hand1 reset failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def hand_get_current_pos(self,obj):
        try:
            pose = None
            if obj == 'Hand0':
                pose = self.rt["hands"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 24:
                    pose_text = ", ".join(f"{v}" for v in pose)
                    self.hand0_cmd_entry.delete(0, tk.END)
                    self.hand0_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid pos for hand0')
                    return
            elif obj == 'Hand1':
                pose = self.rt["hands"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 24:
                    pose_text = ", ".join(f"{v}" for v in pose)
                    self.hand1_cmd_entry.delete(0, tk.END)
                    self.hand1_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid pos for hand1')

            else:
                messagebox.showerror('Error', f'Unknown object: {obj}')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get {obj} pos: {e}')

    def hand_add_pos(self, obj):
        num_points = 24
        if obj == 'Hand0':
            point_str = self.hand0_cmd_entry.get()
            points_list = self.hand_points1
            combo = self.hand0_combo
        elif obj == 'Hand1':
            point_str = self.hand1_cmd_entry.get()
            points_list = self.hand_points2
            combo = self.hand1_combo
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return
        is_valid, result = self.validate_point(point_str, num_points)
        if not is_valid:
            messagebox.showwarning("Wrong inputs", result)
            return
        if self.is_duplicate_command(result, points_list):
            messagebox.showwarning("Duplicate point", f"This point already exists in {obj} list")
            return
        points_list.insert(0, result)
        self.update_comboboxes()

    def hand_delete_pos(self, obj):
        if obj == 'Hand0':
            combo = self.hand0_combo
            points_list = self.hand_points1
        elif obj == 'Hand1':
            combo = self.hand1_combo
            points_list = self.hand_points2
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        selected_index = combo.current()
        if selected_index != -1 and selected_index < len(points_list):
            points_list.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", f"Please select a point to delete in {obj}")

    def hand_run_pos(self, obj):
        try:
            if obj == 'Hand0':
                selected = self.hand0_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 24)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [int(value.strip()) for value in values]
                        ret=robot.runtime_set_hand_pos(FXHandType.FX_HAND_LEFT, point_list)
                        if ret!= 0:
                            messagebox.showerror('Failed!', f"{obj} set run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", f"No point selected for {obj}")
            elif obj == 'Hand1':
                selected = self.hand1_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 24)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [int(value.strip()) for value in values]
                        ret = robot.runtime_set_hand_pos(FXHandType.FX_HAND_RIGHT, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", f"No point selected for {obj}")
            else:
                messagebox.showwarning("Warning", f"Unknown object: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def vel_acc_set(self, obj):
        try:
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM0, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM0, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_ARM1, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_ARM1, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_BODY, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_BODY, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_HEAD, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_HEAD, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
                ret = robot.runtime_set_vel_ratio(FXObjType.OBJ_LIFT, vel)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set vel failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
                ret = robot.runtime_set_acc_ratio(FXObjType.OBJ_LIFT, acc)
                if ret != 0:
                    messagebox.showerror('Failed!', f"{obj} set acc failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                raise ValueError(f"Unknown obj: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def reset_error(self, obj):
        try:
            obj_ = self._obj_name_to_type(obj)
            ret, system_errorcode = robot.reset_error(obj_, 1000)
            if ret != 0:
                messagebox.showerror('Error', f"Reset {obj} failed: {system_errorcode}")
        except Exception as e:
            messagebox.showerror('Error', f"Reset failed: {e}")

    def idle_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            ret =robot.switch_to_idle(obj_type, 1000)
            if ret !=0:
                messagebox.showerror('Error', f"Set idle failed. Error msg: {robot._get_operate_error_msg(ret)}")
        except Exception as e:
            messagebox.showerror('Error', f"Set idle failed: {e}")

    def cr_state(self, obj):
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Invalid obj: {obj}')
            return
        try:
            arm_idx = 0 if obj == 'Arm0' else 1
            ret=robot.switch_to_collab_release(arm_idx, 1000)
            if ret!=0:
                messagebox.showerror('Failed!', f'{obj} switch to collaborative release failed. Error msg: {robot._get_operate_error_msg(ret)}')
        except Exception as e:
            messagebox.showerror('Error', f"CR failed: {e}")

    def position_state(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
            elif obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
            elif obj == 'Head':
                vel = int(self.head_speed_entry.get())
                acc = int(self.head_accel_entry.get())
            elif obj == 'Lift':
                vel = int(self.lift_speed_entry.get())
                acc = int(self.lift_accel_entry.get())
            else:
                vel = acc = 20
            ret=robot.switch_to_position_mode(obj_type, 1000, vel, acc)
            if ret!=0:
                messagebox.showerror('Error', f"{obj} switch to position state failed. Error msg: \n {robot._get_operate_error_msg(ret)}")
        except Exception as e:
            messagebox.showerror('Error', f"Set position state failed: {e}")

    def get_current_pos(self, obj):
        try:
            pose = None
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry.delete(0, tk.END)
                    self.entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
                    return
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.entry1.delete(0, tk.END)
                    self.entry1.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
            elif obj == 'Body':
                pose = self.rt["body"]["fb_pos"]
                if pose and len(pose) == 6:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.body_cmd_entry.delete(0, tk.END)
                    self.body_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Body')
                    return
            elif obj == 'Head':
                pose = self.rt["head"]["fb_pos"]
                if pose and len(pose) == 3:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.head_cmd_entry.delete(0, tk.END)
                    self.head_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Head')
                    return
            elif obj == 'Lift':
                pose = self.rt["lift"]["fb_pos"]
                if pose and len(pose) == 2:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.lift_cmd_entry.delete(0, tk.END)
                    self.lift_cmd_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Lift')
                    return
            else:
                messagebox.showerror('Error', f'Unknown object: {obj}')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def add_pos(self, obj):
        if obj == 'Arm0':
            point_str = self.entry_var.get()
            num_points = 7
            points_list = self.points1
            combo = self.combo1
        elif obj == 'Arm1':
            point_str = self.entry1.get()
            num_points = 7
            points_list = self.points2
            combo = self.combo2
        elif obj == 'Body':
            point_str = self.body_cmd_entry.get()
            num_points = 6
            points_list = self.body_points
            combo = self.body_combo
        elif obj == 'Head':
            point_str = self.head_cmd_entry.get()
            num_points = 3
            points_list = self.head_points
            combo = self.head_combo
        elif obj == 'Lift':
            point_str = self.lift_cmd_entry.get()
            num_points = 2
            points_list = self.lift_points
            combo = self.lift_combo
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        is_valid, result = self.validate_point(point_str, num_points)
        if not is_valid:
            messagebox.showwarning("Wrong inputs", result)
            return
        if self.is_duplicate_command(result, points_list):
            messagebox.showwarning("Duplicate point", f"This point already exists in {obj} list")
            return

        points_list.insert(0, result)
        self.update_comboboxes()

    def delete_pos(self, obj):
        if obj == 'Arm0':
            combo = self.combo1
            points_list = self.points1
        elif obj == 'Arm1':
            combo = self.combo2
            points_list = self.points2
        elif obj == 'Body':
            combo = self.body_combo
            points_list = self.body_points
        elif obj == 'Head':
            combo = self.head_combo
            points_list = self.head_points
        elif obj == 'Lift':
            combo = self.lift_combo
            points_list = self.lift_points
        else:
            messagebox.showerror("Error", f"Unknown object: {obj}")
            return

        selected_index = combo.current()
        if selected_index != -1 and selected_index < len(points_list):
            points_list.pop(selected_index)
            self.update_comboboxes()
        else:
            messagebox.showwarning("Warning", f"Please select a point to delete in {obj}")

    def update_comboboxes(self):
        self.combo1['values'] = self.points1
        self.combo2['values'] = self.points2
        self.body_combo['values'] = self.body_points
        self.head_combo['values'] = self.head_points
        self.lift_combo['values'] = self.lift_points

        self._set_combo_selection(self.combo1, self.points1)
        self._set_combo_selection(self.combo2, self.points2)
        self._set_combo_selection(self.body_combo, self.body_points)
        self._set_combo_selection(self.head_combo, self.head_points)
        self._set_combo_selection(self.lift_combo, self.lift_points)

    def _set_combo_selection(self, combo, points_list):
        if points_list:
            combo.current(0)
        else:
            combo.set('')
        combo.update_idletasks()

    def run_pos(self, obj):
        try:
            if obj == 'Arm0':
                selected = self.combo1.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret=robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM0, point_list)
                        if ret!= 0:
                            messagebox.showerror('Failed!', f"{obj} set run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm0")
            elif obj == 'Arm1':
                selected = self.combo2.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 7)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_ARM1, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Arm1")
            elif obj == 'Body':
                selected = self.body_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 6)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_BODY, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Body")
            elif obj == 'Head':
                selected = self.head_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 3)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_HEAD, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Head")
            elif obj == 'Lift':
                selected = self.lift_combo.get()
                if selected:
                    is_valid, value_str = self.validate_point(selected, 2)
                    if is_valid:
                        values = value_str.split(',')
                        point_list = [float(value.strip()) for value in values]
                        ret = robot.runtime_set_joint_pos_cmd(FXObjType.OBJ_LIFT, point_list)
                        if ret != 0:
                            messagebox.showerror('Failed!', f"set {obj} run pose failed. Error msg: {robot._get_operate_error_msg(ret)}")
                            return
                        time.sleep(0.1)
                    else:
                        messagebox.showerror("Error", f"Invalid format: {selected}")
                        return
                else:
                    messagebox.showwarning("Warning", "No point selected for Lift")
            else:
                messagebox.showwarning("Warning", f"Unknown object: {obj}")
        except Exception as e:
            messagebox.showerror('Error', f"Operation failed: {e}")

    def validate_point(self, point_str, nums):
        try:
            point_str = point_str.strip()
            if not point_str:
                return False, "The input cannot be empty."
            values = point_str.split(',')
            if len(values) != nums:
                return False, f"Please enter {nums} comma-separated numbers"
            validated_values = []
            for value in values:
                value = value.strip()
                if not value:
                    return False, "All positions must contain numbers and cannot be empty."
                if not value.isdigit():
                    try:
                        float(value)
                    except ValueError:
                        return False, f"'{value}' is not a valid number"
                validated_values.append(value)
            if len(validated_values) != nums:
                return False, f"The list length must be {nums}"
            normalized_str = ','.join(validated_values)
            return True, normalized_str
        except Exception as e:
            return False, f"Incorrect input format: {str(e)}"

    def tools_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        set_tools_dialog = tk.Toplevel(self.root)
        set_tools_dialog.title("Tools dynamics and kinematics parameter setting")
        set_tools_dialog.geometry("800x200")  # Reduced height to avoid extra space
        set_tools_dialog.configure(bg="white")
        set_tools_dialog.transient(self.root)
        set_tools_dialog.resizable(True, True)
        set_tools_dialog.grab_set()

        # Main container frame anchored to the top
        main_frame = tk.Frame(set_tools_dialog, bg='white')
        main_frame.pack(fill="both", expand=True, anchor='n')

        # Arm0 row
        arm0_row1 = tk.Frame(main_frame, bg="white")
        arm0_row1.pack(fill="x", pady=5, anchor='n')
        tk.Label(arm0_row1, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=(5, 5))
        tk.Label(arm0_row1, text="Dynamics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm0_row1, textvariable=self.arm0_tool_dyn_entry, width=50).pack(side="left", padx=(5, 5))
        tk.Label(arm0_row1, text="Kinematics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm0_row1, textvariable=self.arm0_tool_kine_entry, width=30).pack(side="left", padx=(5, 5))

        # Arm1 row
        arm1_row1 = tk.Frame(main_frame, bg="white")
        arm1_row1.pack(fill="x", pady=5, anchor='n')
        tk.Label(arm1_row1, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=(5, 5))
        tk.Label(arm1_row1, text="Dynamics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm1_row1, textvariable=self.arm1_tool_dyn_entry, width=50).pack(side="left", padx=(5, 5))
        tk.Label(arm1_row1, text="Kinematics:", width=10).pack(side="left", padx=(5, 0))
        tk.Entry(arm1_row1, textvariable=self.arm1_tool_kine_entry, width=30).pack(side="left", padx=(5, 5))

        # Button row
        btn_row1 = tk.Frame(main_frame, bg="white")
        btn_row1.pack(pady=10, anchor='center')
        tk.Button(btn_row1, text="Set tools", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.tools_set).pack(side="left", padx=10)

    def tools_set(self):
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:

            k0 = self.arm0_tool_kine_entry.get().strip()
            if not k0:
                messagebox.showerror("Error", "arm0 tools kinematics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(k0, 6)
            if not is_valid:
                messagebox.showerror("Error", f"arm0 tools Invalid kinematics format: {result}")
                return
            k0_list = [float(x) for x in result.split(',')]

            d0 = self.arm0_tool_dyn_entry.get().strip()
            if not d0:
                messagebox.showerror("Error", "arm0 tools dynamics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(d0, 10)
            if not is_valid:
                messagebox.showerror("Error", f"arm0 tools Invalid dynamics format: {result}")
                return
            d0_list = [float(x) for x in result.split(',')]

            k1 = self.arm1_tool_kine_entry.get().strip()
            if not k1:
                messagebox.showerror("Error", "arm1 tools kinematics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(k1, 6)
            if not is_valid:
                messagebox.showerror("Error", f"arm1 tools Invalid kinematics format: {result}")
                return
            k1_list = [float(x) for x in result.split(',')]

            d1 = self.arm1_tool_dyn_entry.get().strip()
            if not d1:
                messagebox.showerror("Error", "arm1 tools dynamics parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(d1, 10)
            if not is_valid:
                messagebox.showerror("Error", f"arm1 tools Invalid dynamics format: {result}")
                return
            d1_list = [float(x) for x in result.split(',')]

            arm0_mat=robot.xyzabc2matrix(k0_list)
            ret=robot.set_tool(0,arm0_mat)
            if ret != 0:
                messagebox.showerror('Failed!', f"set tool matrix of arm0 failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_tool_kd(FXObjType.OBJ_ARM0, k0_list, d0_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"set tools failed for arm0. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            
            arm1_mat=robot.xyzabc2matrix(k1_list)
            ret=robot.set_tool(1,arm1_mat)
            if ret != 0:
                messagebox.showerror('Failed!', f"set tool matrix of arm1 failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_tool_kd(FXObjType.OBJ_ARM1, k1_list, d1_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"set tools failed for arm1. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def eef_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        drag_dialog = tk.Toplevel(self.root)
        drag_dialog.title("End-Effector Communication")
        drag_dialog.geometry("1300x400")
        drag_dialog.resizable(True, True)
        drag_dialog.transient(self.root)
        drag_dialog.grab_set()

        drag_dialog.update_idletasks()
        x = (drag_dialog.winfo_screenwidth() - drag_dialog.winfo_width()) // 2
        y = (drag_dialog.winfo_screenheight() - drag_dialog.winfo_height()) // 2
        drag_dialog.geometry(f"+{x}+{y}")

        parent = tk.Frame(drag_dialog, bg="white", padx=10, pady=10)
        parent.pack(fill="both", expand=True)

        self.eef_frame_1 = tk.Frame(parent, bg="white")
        self.eef_frame_1.pack(fill="x")
        self.eef_text_1 = tk.Button(self.eef_frame_1, text="Arm0 send", command=lambda: self.send_data_eef('Arm0'))
        self.eef_text_1.grid(row=0, column=0, padx=5, pady=5)

        self.com_text_1 = tk.Label(self.eef_frame_1, text="Channel", bg="white", width=5)
        self.com_text_1.grid(row=0, column=1, padx=5)

        self.com_select_combobox_1 = ttk.Combobox(
            self.eef_frame_1,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"
        )
        self.com_select_combobox_1.current(0)
        self.com_select_combobox_1.grid(row=0, column=2, padx=5)

        self.com_entry_1 = tk.Entry(self.eef_frame_1, width=120)
        self.com_entry_1.insert(0, "01 00 00 00 FF FF FF FF FF FF FF FC")
        self.com_entry_1.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_delet_1 = tk.Button(self.eef_frame_1, text="Delete", command=lambda: self.delete_eef_command('Arm0'))
        self.eef_delet_1.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo1 = ttk.Combobox(self.eef_frame_1, state="readonly", width=120)
        self.eef_combo1.grid(row=0, column=4, padx=5)

        self.eef_bt_1 = tk.Button(self.eef_frame_1, text="Arm0 receive", command=lambda: self.receive_data_eef('Arm0'))
        self.eef_bt_1.grid(row=0, column=5, padx=5)

        self.eef_frame_1_2 = tk.Frame(parent, bg="white")
        self.eef_frame_1_2.pack(fill="x")

        self.eef1_2_b1 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b1.grid(row=0, column=0, padx=5)

        self.eef1_2_b2 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b2.grid(row=0, column=1, padx=5)

        self.eef1_2_b3 = tk.Label(self.eef_frame_1_2, text="", bg="white", width=7)
        self.eef1_2_b3.grid(row=0, column=2, padx=5)

        self.eef_add_1 = tk.Button(self.eef_frame_1_2, text='Arm0 add', command=lambda: self.add_eef_command('Arm0'))
        self.eef_add_1.grid(row=0, column=3, padx=5)

        self.eef_entry = tk.Entry(self.eef_frame_1_2, width=120)
        self.eef_entry.insert(0, "01 06 00 00 00 01 48 0A")
        self.eef_entry.grid(row=0, column=4, padx=5, sticky="ew")

        self.eef_add_2 = tk.Button(self.eef_frame_1_2, text='Arm1 add', command=lambda: self.add_eef_command('Arm1'))
        self.eef_add_2.grid(row=0, column=5, padx=5)

        self.eef_frame_2 = tk.Frame(parent, bg="white")
        self.eef_frame_2.pack(fill="x")
        self.eef_bt_2 = tk.Button(self.eef_frame_2, text="Arm1 send", command=lambda: self.send_data_eef('Arm1'))
        self.eef_bt_2.grid(row=0, column=0, padx=5)

        self.com_text_2 = tk.Label(self.eef_frame_2, text="Channel", bg="white", width=5)
        self.com_text_2.grid(row=0, column=1, padx=5)

        self.com_select_combobox_2 = ttk.Combobox(
            self.eef_frame_2,
            values=["CAN", "COM1", "COM2"],
            width=5,
            state="readonly"
        )
        self.com_select_combobox_2.current(0)
        self.com_select_combobox_2.grid(row=0, column=2, padx=5)

        self.eef_delet_2 = tk.Button(self.eef_frame_2, text="Delete", command=lambda: self.delete_eef_command('Arm1'))
        self.eef_delet_2.grid(row=0, column=3, padx=5, pady=5)

        self.eef_combo2 = ttk.Combobox(self.eef_frame_2, state="readonly", width=120)
        self.eef_combo2.grid(row=0, column=4, padx=5)

        self.eef_bt_4 = tk.Button(self.eef_frame_2, text="Arm1 receive", command=lambda: self.receive_data_eef('Arm1'))
        self.eef_bt_4.grid(row=0, column=5, padx=5, pady=5)

        self.eef_frame_3 = tk.Frame(parent, bg="white")
        self.eef_frame_3.pack(fill="x")

        recv_label1 = tk.Label(self.eef_frame_3, text="Arm0 received:")
        recv_label1.grid(row=0, column=0, padx=5)

        spacer = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer.grid(row=0, column=1, padx=5)

        self.recv_text1 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text1.grid(row=1, column=0, padx=5)
        self.recv_text1.insert(tk.END,
                               'Usage tips:\nFirst select the port: CAN/COM1/COM2,\nClick Arm0 receive button,\nEnter data to send, click Arm0 send button,\nReceived end-effector data is refreshed at 1kHz')

        spacer1 = tk.Label(self.eef_frame_3, text="   ", bg='white')
        spacer1.grid(row=1, column=1, padx=5)

        recv_label2 = tk.Label(self.eef_frame_3, text="Arm1 received:")
        recv_label2.grid(row=0, column=2, padx=5)

        self.recv_text2 = scrolledtext.ScrolledText(self.eef_frame_3, width=70, height=8, wrap=tk.WORD)
        self.recv_text2.grid(row=1, column=2, padx=5)
        self.recv_text2.insert(tk.END,
                               'Usage tips:\nFirst select the port: CAN/COM1/COM2,\nClick Arm1 receive button,\nEnter data to send, click Arm1 send button,\nReceived end-effector data is refreshed at 1kHz')

        status_display_frame_7 = tk.Frame(parent, bg="white", padx=10, pady=5)
        status_display_frame_7.pack(fill="x", pady=5)

    def is_duplicate_command(self, point_list, target_list):
        for existing_point_str in target_list:
            if existing_point_str == point_list:
                return True
        return False

    def add_eef_command(self, obj):
        command_str = self.eef_entry.get()
        if obj == 'Arm0':
            if self.is_duplicate_command(command_str, self.command1):
                messagebox.showwarning("Duplicate instruction", "This instruction already exists in left arm list.")
                return
            else:
                self.command1.insert(0, command_str)
        elif obj == 'Arm1':
            if self.is_duplicate_command(command_str, self.command2):
                messagebox.showwarning("Duplicate instruction", "This instruction already exists in right arm list.")
                return
            else:
                self.command2.insert(0, command_str)
        self.update_combo_eef()

    def update_combo_eef(self):
        self.eef_combo1['values'] = self.command1
        self.eef_combo2['values'] = self.command2
        if self.command1:
            self.eef_combo1.current(0)
        else:
            self.eef_combo1.set('')
        if self.command2:
            self.eef_combo2.current(0)
        else:
            self.eef_combo2.set('')

    def send_data_eef(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            com = 0
            com_str = ''
            sample_data = None
            if obj == 'Arm0':
                sample_data = self.eef_combo1.get()
                com_str = self.com_select_combobox_1.get()
                terminal = FXObjType.OBJ_ARM0
            elif obj == 'Arm1':
                sample_data = self.eef_combo2.get()
                com_str = self.com_select_combobox_2.get()
                terminal = FXObjType.OBJ_ARM1
            else:
                return

            if com_str == 'CAN':
                com = 1
            elif com_str == 'COM1':
                com = 2
            elif com_str == 'COM2':
                com = 3

            ret=robot.terminal_clear(terminal)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj} can not clear {com_str} buffer. Error msg: {robot._get_operate_error_msg(ret)}')
                return
            time.sleep(0.01)
            ret, send_time=robot.terminal_set(terminal, com, sample_data)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj} set data to {com_str} failed. Error msg:  {robot._get_operate_error_msg(ret)}')
                return

            for _ in range(200):
                chn, data, time_rec = robot.terminal_get(terminal)
                if chn > 0:
                    hex_str = data.hex().upper()
                    formatted_hex = ' '.join(hex_str[i:i + 2] for i in range(0, len(hex_str), 2))
                    if obj == 'Arm0':
                        self.recv_text1.delete('1.0', tk.END)
                        self.recv_text1.insert(tk.END, formatted_hex)
                    else:
                        self.recv_text2.delete('1.0', tk.END)
                        self.recv_text2.insert(tk.END, formatted_hex)
                    break
                time.sleep(0.005)
            else:
                if obj == 'Arm0':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, "No response")
                else:
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, "No response")
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def delete_eef_command(self, obj):
        if obj == 'Arm0':
            selected_index = self.eef_combo1.current()
            if selected_index != -1 and selected_index < len(self.command1):
                self.command1.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete.")
        elif obj == 'Arm1':
            selected_index = self.eef_combo2.current()
            if selected_index != -1 and selected_index < len(self.command2):
                self.command2.pop(selected_index)
                self.update_combo_eef()
            else:
                messagebox.showwarning("Warning", "Please select a communication command to delete.")

    def receive_data_eef(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            terminal = FXObjType.OBJ_ARM0 if obj == 'Arm0' else FXObjType.OBJ_ARM1
            chn, data = robot.terminal_get(terminal)
            if chn > 0:
                text = data.decode(errors='replace')
                if obj == 'Arm0':
                    self.recv_text1.delete('1.0', tk.END)
                    self.recv_text1.insert(tk.END, text)
                else:
                    self.recv_text2.delete('1.0', tk.END)
                    self.recv_text2.insert(tk.END, text)
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def imu_settings(self):
        settings_window = tk.Toplevel(self.root)
        settings_window.title("IMU calculations")
        settings_window.geometry("880x760")
        settings_window.configure(bg="#f5f6f8")
        settings_window.transient(self.root)
        settings_window.resizable(True, True)
        settings_window.grab_set()

        # Keep a reference so the periodic IMU refresh can stop on close.
        self._imu_win = settings_window
        self._imu_refresh_id = None

        # ---- Header ----
        header = tk.Frame(settings_window, bg="#2c3e50", height=40)
        header.pack(fill="x")
        header.pack_propagate(False)
        tk.Label(header, text="IMU Calculations & Monitoring",
                 font=("Arial", 13, "bold"), fg="white", bg="#2c3e50").pack(side="left", padx=15)

        # ---- Real-time IMU data panel ----
        imu_frame = tk.LabelFrame(settings_window, text="  Real-time IMU Data (refresh 200 ms)  ",
                                  font=("Arial", 11, "bold"), fg="#2c3e50", bg="#ffffff",
                                  relief=tk.GROOVE, borderwidth=2)
        imu_frame.pack(fill="x", padx=12, pady=(10, 6))

        # Each IMU is 9 floats: assume Accel(x,y,z) / Gyro(x,y,z) / Mag(x,y,z).
        imu_groups = [("Acc", ("X", "Y", "Z")),
                      ("W-acc",  ("X", "Y", "Z")),
                      ("Angle",   ("X", "Y", "Z"))]

        self._imu_labels = {"robot": [], "agv": []}

        def build_imu_column(parent, title, color):
            col = tk.Frame(parent, bg="#ffffff")
            col.pack(side="left", fill="both", expand=True, padx=8, pady=6)
            tk.Label(col, text=title, font=("Arial", 10, "bold"),
                     fg="white", bg=color).pack(fill="x", ipady=3)
            labels = []
            for gname, axes in imu_groups:
                gframe = tk.Frame(col, bg="#ffffff")
                gframe.pack(fill="x", pady=(6, 2))
                tk.Label(gframe, text=gname, width=6, font=("Arial", 9, "bold"),
                         bg="#eef2f7", fg="#2c3e50").grid(row=0, column=0, sticky="w")
                row_labels = []
                for j, ax in enumerate(axes):
                    tk.Label(gframe, text=ax, width=4, font=("Arial", 9),
                             bg="#ffffff", fg="#7f8c8d").grid(row=0, column=1 + j * 2, padx=(8, 0))
                    lab = tk.Label(gframe, text="0.000", width=9, font=("Consolas", 9),
                                   bg="#ffffff", fg="#2c3e50", anchor="e")
                    lab.grid(row=0, column=2 + j * 2, padx=(0, 4))
                    row_labels.append(lab)
                labels.append(row_labels)
            return labels

        self._imu_labels["robot"] = build_imu_column(imu_frame, "Robot IMU", "#2980b9")
        self._imu_labels["agv"]   = build_imu_column(imu_frame, "AGV IMU",    "#27ae60")

        # ---- Notebook with calculation tab ----
        notebook = ttk.Notebook(settings_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=12, pady=(6, 6))
        floating_base_frame = ttk.Frame(notebook, padding="10")
        notebook.add(floating_base_frame, text="Floating base parameter calculation")

        self.create_floating_base_tab(floating_base_frame)

        # ---- Bottom buttons ----
        button_frame = tk.Frame(settings_window, bg="#f5f6f8")
        button_frame.pack(fill="x", padx=12, pady=(0, 10))
        ttk.Button(button_frame, text="Save settings",
                   command=lambda: self.save_all_settings(notebook)).pack(side="left", padx=5)
        ttk.Button(button_frame, text="Close",
                   command=self._close_imu_settings).pack(side="left", padx=5)

        # Start periodic refresh and do one immediate update.
        self._refresh_imu_data()
        settings_window.protocol("WM_DELETE_WINDOW", self._close_imu_settings)

    def _close_imu_settings(self):
        """Stop the IMU refresh loop and close the window."""
        if self._imu_refresh_id is not None:
            try:
                self._imu_win.after_cancel(self._imu_refresh_id)
            except Exception:
                pass
            self._imu_refresh_id = None
        if self._imu_win is not None:
            try:
                self._imu_win.destroy()
            except Exception:
                pass
            self._imu_win = None

    def _refresh_imu_data(self):
        """Refresh robot_imu / agv_imu from self.rt every 200 ms while the window is open."""
        win = self._imu_win
        if win is None or not win.winfo_exists():
            self._imu_refresh_id = None
            return
        try:
            rt = getattr(self, "rt", None)
            if rt:
                # robot_imu
                robot_imu = rt.get("robot_imu")
                if robot_imu is not None and len(robot_imu) >= 9:
                    self._set_imu_labels("robot", robot_imu)
                else:
                    self._set_imu_labels("robot", [0.0] * 9)
                # agv_imu
                agv_imu = rt.get("agv_imu")
                if agv_imu is not None and len(agv_imu) >= 9:
                    self._set_imu_labels("agv", agv_imu)
                else:
                    self._set_imu_labels("agv", [0.0] * 9)
        except Exception:
            pass
        self._imu_refresh_id = win.after(200, self._refresh_imu_data)

    def _set_imu_labels(self, which, values):
        """values is 9 floats grouped as [ax,ay,az, gx,gy,gz, mx,my,mz]."""
        labels = self._imu_labels.get(which)
        if not labels:
            return
        groups = [values[0:3], values[3:6], values[6:9]]
        for gi, row_labels in enumerate(labels):
            for j, lab in enumerate(row_labels):
                v = groups[gi][j] if gi < len(groups) and j < len(groups[gi]) else 0.0
                lab.config(text=f"{v:.3f}")

    def create_floating_base_tab(self, parent):
        self.row2_selection = [0, 0, 0]
        self.row3_selection = [0, 0, 0]

        self.row2_var = tk.StringVar()
        self.row3_var = tk.StringVar()

        self.row2_var.trace('w', lambda *args: self.on_selection_change(2))
        self.row3_var.trace('w', lambda *args: self.on_selection_change(3))

        ttk.Label(parent, text="Floating base parameter calculation",
                  font=("Arial", 13, "bold"), foreground="#2c3e50").pack(pady=(6, 12))

        row1_frame = ttk.Frame(parent)
        row1_frame.pack(fill="x", pady=4)
        ttk.Label(row1_frame, text="The coordinate directions of the base (x-axis and y-axis)",
                  font=("Arial", 9)).pack(side="left", padx=5)
        ttk.Label(row1_frame,
                  text="UMI coordinate orientation (option to align base with UMI coordinate orientation)",
                  font=("Arial", 9)).pack(side="right", padx=5)

        axis_frame = tk.Frame(parent, bg="#ffffff", relief=tk.GROOVE, borderwidth=1)
        axis_frame.pack(fill="x", pady=8)
        tk.Label(axis_frame, text="X-axis", width=8, font=("Arial", 9, "bold"),
                 bg="#eef2f7", fg="#2c3e50").grid(row=0, column=0, padx=8, pady=8, sticky="w")
        tk.Label(axis_frame, text="Y-axis", width=8, font=("Arial", 9, "bold"),
                 bg="#eef2f7", fg="#2c3e50").grid(row=1, column=0, padx=8, pady=8, sticky="w")

        options = ["x", "-x", "y", "-y", "z", "-z"]
        self.row2_buttons = []
        for i, option in enumerate(options):
            btn = ttk.Radiobutton(axis_frame, text=option, value=option,
                                  variable=self.row2_var,
                                  command=lambda: self.on_selection_change(2))
            btn.grid(row=0, column=i + 1, padx=8, pady=6)
            self.row2_buttons.append(btn)

        self.row3_buttons = []
        for i, option in enumerate(options):
            btn = ttk.Radiobutton(axis_frame, text=option, value=option,
                                  variable=self.row3_var,
                                  command=lambda: self.on_selection_change(3))
            btn.grid(row=1, column=i + 1, padx=8, pady=6)
            self.row3_buttons.append(btn)

        self.result_frame = tk.LabelFrame(parent, text="  Calculation results  ",
                                          font=("Arial", 10, "bold"), fg="#2c3e50", bg="#ffffff",
                                          relief=tk.GROOVE, borderwidth=2)
        self.result_frame.pack(fill="both", expand=True, pady=(8, 4))

        self.result_text = tk.Text(self.result_frame, height=8, wrap=tk.WORD,
                                   font=("Consolas", 10), bg="#fafbfc", relief=tk.FLAT)
        scrollbar = ttk.Scrollbar(self.result_frame, orient=tk.VERTICAL, command=self.result_text.yview)
        self.result_text.config(yscrollcommand=scrollbar.set)

        self.result_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=6, pady=6)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y, pady=6)

    def on_selection_change(self, changed_row):
        self.update_selection_lists()
        self.apply_mutual_exclusion(changed_row)
        if any(self.row2_selection) and any(self.row3_selection):
            result = self.get_abc_calculation()
            self.display_result(result)
        else:
            self.result_text.delete(1.0, tk.END)
            self.result_text.insert(tk.END,
                                    "Please complete the selections in both rows to view the calculation results.")

    def update_selection_lists(self):

        self.row2_selection = [0, 0, 0]
        self.row3_selection = [0, 0, 0]

        row2_val = self.row2_var.get()
        if row2_val == "x":
            self.row2_selection[0] = 1
        if row2_val == "-x":
            self.row2_selection[0] = -1
        if row2_val == "y":
            self.row2_selection[1] = 1
        if row2_val == "-y":
            self.row2_selection[1] = -1
        if row2_val == "z":
            self.row2_selection[2] = 1
        if row2_val == "-z":
            self.row2_selection[2] = -1

        row3_val = self.row3_var.get()
        if row3_val == "x":
            self.row3_selection[0] = 1
        if row3_val == "-x":
            self.row3_selection[0] = -1
        if row3_val == "y":
            self.row3_selection[1] = 1
        if row3_val == "-y":
            self.row3_selection[1] = -1
        if row3_val == "z":
            self.row3_selection[2] = 1
        if row3_val == "-z":
            self.row3_selection[2] = -1

    def apply_mutual_exclusion(self, changed_row):
        row2_val = self.row2_var.get()
        row3_val = self.row3_var.get()

        for btn in self.row2_buttons + self.row3_buttons:
            btn.state(["!disabled"])
        if row2_val:
            if row2_val in ["x", "-x"]:
                self.disable_axis_options(self.row3_buttons, ["x", "-x"])
            elif row2_val in ["y", "-y"]:
                self.disable_axis_options(self.row3_buttons, ["y", "-y"])
            elif row2_val in ["z", "-z"]:
                self.disable_axis_options(self.row3_buttons, ["z", "-z"])
        if row3_val:
            if row3_val in ["x", "-x"]:
                self.disable_axis_options(self.row2_buttons, ["x", "-x"])
            elif row3_val in ["y", "-y"]:
                self.disable_axis_options(self.row2_buttons, ["y", "-y"])
            elif row3_val in ["z", "-z"]:
                self.disable_axis_options(self.row2_buttons, ["z", "-z"])

    def disable_axis_options(self, buttons, options_to_disable):
        for btn in buttons:
            if btn['value'] in options_to_disable:
                btn.state(["disabled"])

    def main_function(self,vx, vy):
        def Matrix2ABC(m, abc):
            r = math.sqrt(m[0][0] * m[0][0] + m[1][0] * m[1][0])
            abc[1] = math.atan2(-m[2][0], r) * 57.295779513082320876798154814105
            if abs(r) <= DBL_EPSILON:
                abc[2] = 0
                if abc[1] > 0:
                    abc[0] = math.atan2(m[0][1], m[1][1]) * 57.295779513082320876798154814105
                else:
                    abc[0] = -math.atan2(m[0][1], m[1][1]) * 57.295779513082320876798154814105
            else:
                abc[2] = math.atan2(m[1][0], m[0][0]) * 57.295779513082320876798154814105
                abc[0] = math.atan2(m[2][1], m[2][2]) * 57.295779513082320876798154814105
            return True

        def FX_VectCross( a, b):
            result = [0.0] * 3
            result[0] = a[1] * b[2] - a[2] * b[1]
            result[1] = a[2] * b[0] - a[0] * b[2]
            result[2] = a[0] * b[1] - a[1] * b[0]
            return result

        def NormVect(a):
            return math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])

        m_S = ""
        if NormVect(vx) < 0.01 or NormVect(vy) < 0.01:
            return m_S, [0, 0, 0]
        vz = FX_VectCross(vx, vy)
        vz_norm = NormVect(vz)
        if vz_norm < 0.99 or vz_norm > 1.01:
            return m_S, [0, 0, 0]
        m_mat = [
            [vx[0], vy[0], vz[0]],
            [vx[1], vy[1], vz[1]],
            [vx[2], vy[2], vz[2]]
        ]
        m_S += "Matrix form (column vectors are coordinate direction vectors):\n"
        m_S += f"{m_mat[0][0]:.2f}\t{m_mat[0][1]:.2f}\t{m_mat[0][2]:.2f}\n"
        m_S += f"{m_mat[1][0]:.2f}\t{m_mat[1][1]:.2f}\t{m_mat[1][2]:.2f}\n"
        m_S += f"{m_mat[2][0]:.2f}\t{m_mat[2][1]:.2f}\t{m_mat[2][2]:.2f}\n\n"
        m_abc = [0.0] * 3
        Matrix2ABC(m_mat, m_abc)
        m_S += f"ABC angles：[{m_abc[0]:.5f}, {m_abc[1]:.5f}, {m_abc[2]:.5f}]\n"
        return m_S

    def get_abc_calculation(self):
        result = f"The base coordinate direction is as follows during the rotation of the gyroscope IMU:\n"
        result += "=" * 20 + "\n"
        try:
            abc = self.main_function(self.row2_selection, self.row3_selection)
            result += abc
            result += "\n"
        except Exception as e:
            result += f"Calculation error: {str(e)}\n"

        result += "=" * 20 + "\n\n"
        result += ("Please update the three angles A, B, and C to the [R.A0.BASIC] group in robot.ini respectively:\n"
                   "GYROSETA, GYROSETB, GYROSETC\n"
                   "Please note that the left and right arms should be calculated sequentially, with [R.A0.BASIC] representing the left arm and [R.A1.BASIC] representing the right arm.")
        return result

    def display_result(self, result):
        self.result_text.delete(1.0, tk.END)
        self.result_text.insert(tk.END, result)

    def show_more_features(self):
        menu = tk.Menu(self.root, tearoff=0)
        menu.add_command(label="Data Visualization", command=self.data_visualization_dialog)
        menu.add_separator()
        menu.add_command(label="CAN/485", command=self.eef_dialog)
        menu.add_separator()
        menu.add_command(label="Hand Data Communication", command=self.hand_data_dialog)
        menu.add_separator()
        menu.add_command(label="Tool Dynamics Identification", command=self.tool_dynamics_dialog)
        menu.add_separator()
        menu.add_command(label="Tools Setting", command=self.tools_dialog)
        menu.add_separator()
        menu.add_command(label="Motion Planning", command=self.planning_dialog)
        menu.add_separator()
        menu.add_command(label="Jogging", command=self.step_motion_dialog)
        menu.add_separator()
        menu.add_command(label="IMU Calculation", command=self.imu_settings)
        menu.add_separator()
        menu.add_command(label="Sensors & Encoders", command=self.sensor_decoder_dialog)
        menu.add_separator()
        menu.add_command(label="SDO", command=self.sdo_dialog)
        menu.add_separator()
        menu.add_command(label="Versions", command=self.servo_sensor_version_dialog)
        menu.add_separator()
        menu.add_command(label="System Upgrade", command=self.system_update_dialog)
        menu.add_separator()
        menu.add_command(label="Robot Settings",command=self.config_settings_dialog)
        # menu.add_separator()
        # menu.add_command(label="Docs", )#command=self.open_doc()
        try:
            menu.tk_popup(
                self.more_features_btn.winfo_rootx(),
                self.more_features_btn.winfo_rooty() + self.more_features_btn.winfo_height()
            )
        finally:
            menu.grab_release()

    # ==================== Data Visualization ====================
    def data_visualization_dialog(self):
        """Realtime signal viewer: plots a user-picked subset of RT data.

        Data comes from self.data_manager.latest_rt (the same snapshot the main
        window already polls), sampled by a thread that lives only as long as
        this window. Nothing is written back to PYTHON_SDK.
        """
        if _matplotlib_error is not None:
            messagebox.showerror(
                'Error',
                "matplotlib is not available, so Data Visualization cannot start.\n"
                f"Import error: {_matplotlib_error}")
            return
        if not self.connected or not self.data_manager:
            messagebox.showerror('Error', "Please connect robot first!")
            return

        # One window at a time: if it is already up, just raise it instead of
        # starting a second sampling thread.
        if self._viz_win is not None and self._viz_win.winfo_exists():
            self._viz_win.lift()
            return

        win = tk.Toplevel(self.root)
        win.title("Data Visualization")
        win.geometry("1250x780")
        win.configure(bg="white")
        win.transient(self.root)
        win.resizable(True, True)
        self._viz_win = win

        # ---- per-dialog state ----
        self._viz_alive = True
        self._viz_running = False          # sampling thread enabled
        self._viz_draw_id = None           # pending after() id for redraw
        self._viz_thread = None
        self._viz_rate = float(_VIZ_RATE_DEFAULT)   # Hz
        self._viz_window = 10.0            # seconds kept in the buffer
        self._viz_maxlen = int(self._viz_rate * self._viz_window)
        self._viz_buf = {}                 # key -> deque[(t, value)]
        self._viz_selected = list(_VIZ_DEFAULT_KEYS)
        self._viz_color_slots = list(_VIZ_DEFAULT_KEYS)  # selection order -> colour
        self._viz_checked = {}             # tree item id -> signal key
        self._viz_overrun = 0
        self._viz_samples = 0
        self._viz_t0 = 0.0
        self._viz_view_end = 0.0           # right edge of the view, frozen on pause
        self._viz_lines = {}               # key -> Line2D

        # ---- selection tree (left) + plot (right) ----
        body = tk.Frame(win, bg="white")
        body.pack(fill="both", expand=True, padx=10, pady=(10, 5))

        left = tk.Frame(body, bg="white")
        left.pack(side="left", fill="y")
        tree_head = tk.Frame(left, bg="white")
        tree_head.pack(fill="x")
        tk.Label(tree_head, text="Signals (click to tick):", bg="white").pack(side="left")
        tk.Button(tree_head, text="Clear all", command=self._viz_clear_ticks
                  ).pack(side="right", padx=(5, 0))
        tree_wrap = tk.Frame(left, bg="white")
        tree_wrap.pack(fill="both", expand=True)
        self._viz_tree = ttk.Treeview(tree_wrap, show="tree", height=28,
                                      selectmode="browse")
        tree_sb = ttk.Scrollbar(tree_wrap, orient="vertical",
                                command=self._viz_tree.yview)
        self._viz_tree.configure(yscrollcommand=tree_sb.set)
        self._viz_tree.pack(side="left", fill="both", expand=True)
        tree_sb.pack(side="left", fill="y")
        self._viz_tree.bind("<Button-1>", self._viz_tree_click)
        self._build_viz_tree()

        right = tk.Frame(body, bg="white")
        right.pack(side="left", fill="both", expand=True, padx=(10, 0))
        figure = Figure(figsize=(7.5, 5.2), dpi=100)
        self._viz_ax = figure.add_subplot(111)
        self._viz_ax.set_xlabel("time (s)")
        self._viz_ax.set_ylabel("value")
        self._viz_ax.grid(True, alpha=0.3)
        self._viz_figure = figure
        self._viz_canvas = FigureCanvasTkAgg(figure, master=right)
        self._viz_canvas.get_tk_widget().pack(fill="both", expand=True)
        toolbar = NavigationToolbar2Tk(self._viz_canvas, right)
        toolbar.update()
        toolbar.pack(fill="x")

        # ---- controls ----
        ctl = tk.Frame(win, bg="white")
        ctl.pack(fill="x", padx=10, pady=(0, 10))

        self._viz_start_btn = tk.Button(ctl, text="Start", bg="#A2CD5A", width=8,
                                        command=self._viz_start)
        self._viz_start_btn.pack(side="left", padx=(0, 5))
        self._viz_pause_btn = tk.Button(ctl, text="Pause", bg="#4AA7EA", fg="white",
                                        width=8, state="disabled",
                                        command=self._viz_pause)
        self._viz_pause_btn.pack(side="left", padx=(0, 5))
        tk.Button(ctl, text="Clear", width=8,
                  command=self._viz_clear).pack(side="left", padx=(0, 10))

        tk.Label(ctl, text="Rate:", bg="white").pack(side="left")
        self._viz_rate_var = tk.IntVar(value=_VIZ_RATE_DEFAULT)
        rate_scale = tk.Scale(ctl, from_=_VIZ_RATE_MIN, to=_VIZ_RATE_MAX,
                              orient="horizontal", variable=self._viz_rate_var,
                              length=190, showvalue=False, bg="white",
                              highlightthickness=0, command=self._viz_rate_changed)
        rate_scale.pack(side="left", padx=(2, 2))
        self._viz_rate_label = tk.Label(ctl, text=f"{_VIZ_RATE_DEFAULT} Hz",
                                        bg="white", width=8, anchor="w")
        self._viz_rate_label.pack(side="left", padx=(0, 10))

        tk.Label(ctl, text="Window:", bg="white").pack(side="left")
        self._viz_window_var = tk.StringVar(value="10 s")
        ttk.Combobox(ctl, textvariable=self._viz_window_var, state="readonly", width=6,
                     values=["5 s", "10 s", "30 s", "60 s"]).pack(side="left", padx=(2, 10))

        self._viz_auto_var = tk.IntVar(value=1)
        tk.Checkbutton(ctl, text="Auto Y", variable=self._viz_auto_var,
                       bg="white").pack(side="left", padx=(0, 10))

        self._viz_status = tk.Label(ctl, text="idle", bg="white", fg="gray")
        self._viz_status.pack(side="left")

        win.protocol("WM_DELETE_WINDOW", lambda: self._viz_close(win))
        self._viz_redraw()          # draw the empty axes + default legend
        self._viz_start()           # sampling on by default

    # ---- selection tree -------------------------------------------------
    def _build_viz_tree(self):
        """Populate the tree from the signal keys currently present in RT data."""
        tree = self._viz_tree
        for item in tree.get_children():
            tree.delete(item)
        self._viz_checked = {}

        rt = self.data_manager.latest_rt if self.data_manager else None
        keys = sorted(flatten_rt(rt).keys())
        if not keys:
            tree.insert("", "end", text="(no RT data yet)")
            return

        # Group by the first path segment, then by the parent path.
        groups = {}
        for key in keys:
            groups.setdefault(key.split("/")[0], []).append(key)

        for group in sorted(groups):
            node = tree.insert("", "end", text=group, open=(group == "arms"))
            subgroups = {}
            for key in groups[group]:
                parent = "/".join(key.split("/")[:-1]) or group
                subgroups.setdefault(parent, []).append(key)
            for parent in sorted(subgroups):
                if parent == group:
                    target = node
                else:
                    # "arms/0/fb/fb_pos" -> show as "0 / fb / fb_pos" under arms
                    target = tree.insert(node, "end", text=parent[len(group) + 1:])
                for key in sorted(subgroups[parent]):
                    checked = key in self._viz_selected
                    item = tree.insert(target, "end",
                                       text=(_VIZ_TICK if checked else _VIZ_UNTICK) + viz_signal_label(key))
                    self._viz_checked[item] = key
                    if checked:
                        # Open every branch on the path to a ticked signal, so
                        # the default selection is visible without hunting.
                        self._viz_expand_path(target)

    def _viz_expand_path(self, item):
        while item:
            self._viz_tree.item(item, open=True)
            item = self._viz_tree.parent(item)

    def _viz_tree_click(self, event):
        """Toggle the tick on the clicked leaf; rows without one are left alone.

        Returning "break" here would consume the click for every row, including
        the group rows, whose expand/collapse arrow is driven by Tk's own
        binding. Only a consumed leaf click stops propagation.
        """
        if not self._viz_alive:
            return
        item = self._viz_tree.identify_row(event.y)
        if not item or item not in self._viz_checked:
            return
        key = self._viz_checked[item]
        text = self._viz_tree.item(item, "text")
        if text.startswith(_VIZ_TICK):
            if key in self._viz_selected:
                self._viz_selected.remove(key)
            self._viz_tree.item(item, text=_VIZ_UNTICK + text[len(_VIZ_TICK):])
        else:
            if key not in self._viz_selected:
                self._viz_selected.append(key)
            self._viz_tree.item(item, text=_VIZ_TICK + text[len(_VIZ_UNTICK):])
        self._viz_apply_selection()
        return "break"

    def _viz_clear_ticks(self):
        """Untick every signal, so the user does not have to click them off one by one."""
        if not self._viz_alive or not self._viz_selected:
            return
        self._viz_selected = []
        for item in list(self._viz_checked):
            text = self._viz_tree.item(item, "text")
            if text.startswith(_VIZ_TICK):
                self._viz_tree.item(item, text=_VIZ_UNTICK + text[len(_VIZ_TICK):])
        self._viz_apply_selection()

    def _viz_apply_selection(self):
        """Drop buffers/lines for unticked signals; pick up newly ticked ones."""
        for key in list(self._viz_buf):
            if key not in self._viz_selected:
                self._viz_buf.pop(key, None)
        self._viz_color_slots = list(self._viz_selected)
        self._viz_repaint_colors()
        self._viz_redraw()

    # ---- sampling -------------------------------------------------------
    def _viz_start(self):
        if not self._viz_alive or self._viz_running:
            return
        self._viz_read_controls()
        self._viz_running = True
        self._viz_thread = threading.Thread(target=self._viz_sample_loop, daemon=True)
        self._viz_thread.start()
        self._viz_start_btn.config(state="disabled")
        self._viz_pause_btn.config(state="normal")
        self._viz_status.config(text="sampling...", fg="green")
        self._viz_schedule_draw()

    def _viz_pause(self):
        self._viz_running = False
        self._viz_start_btn.config(state="normal")
        self._viz_pause_btn.config(state="disabled")
        if self._viz_alive:
            self._viz_status.config(text="paused", fg="gray")

    def _viz_rate_changed(self, _value=None):
        """Rate slider callback: update the label, and the buffers if sampling."""
        rate = self._viz_rate_var.get()
        if self._viz_rate_label.winfo_exists():
            self._viz_rate_label.config(text=f"{rate} Hz")
        if self._viz_alive and self._viz_running:
            self._viz_read_controls()

    def _viz_read_controls(self):
        """Apply the Rate slider / Window combobox; resizes the history buffers.

        Called on every rate change, so the buffer length always matches the
        rate: at 1000 Hz a 10 s window needs 10000 slots per signal.
        """
        try:
            rate = float(self._viz_rate_var.get())
        except Exception:
            rate = float(_VIZ_RATE_DEFAULT)
        try:
            window = float(self._viz_window_var.get().split()[0])
        except Exception:
            window = 10.0
        maxlen = max(2, int(rate * window))
        if maxlen != self._viz_maxlen:
            self._viz_maxlen = maxlen
            for key, buf in self._viz_buf.items():
                # deque(maxlen=) is fixed at construction; rebuild with the data
                self._viz_buf[key] = collections.deque(buf, maxlen=maxlen)
        self._viz_rate = rate
        self._viz_window = window

    def _viz_sample_loop(self):
        """Sample latest_rt at a fixed rate until the dialog closes.

        Runs off the Tk thread. It only touches the plain-python buffers, never
        a widget; the redraw is pulled by _viz_redraw from the Tk loop.

        A deadline clock is used instead of "sleep(period - work)": at 1000 Hz a
        single 1 ms overrun would otherwise reset the cadence every cycle and the
        achieved rate would sag well below the requested one. The rate is re-read
        every cycle so the slider takes effect immediately.

        Note the SDK feeds RT data at its own 1 ms tick, so rates above that
        resample the same frame and can show a staircase, not error.
        """
        deadline = time.perf_counter()
        while self._viz_alive and self._viz_running:
            dm = self.data_manager
            if dm is not None:
                flat = flatten_rt(dm.latest_rt)
                if flat:
                    cycle_start = time.perf_counter()
                    if self._viz_t0 == 0.0:
                        self._viz_t0 = cycle_start
                    t = cycle_start - self._viz_t0
                    for key in self._viz_selected:
                        if key not in flat:
                            continue
                        buf = self._viz_buf.get(key)
                        if buf is None:
                            buf = collections.deque(maxlen=self._viz_maxlen)
                            self._viz_buf[key] = buf
                        buf.append((t, flat[key]))
                    self._viz_samples += 1
            # Advance to the next slot on the deadline, not from "now": the work
            # above is already accounted for and lateness does not accumulate.
            period = 1.0 / max(self._viz_rate, 1.0)
            deadline += period
            sleep_time = deadline - time.perf_counter()
            if sleep_time > 0:
                time.sleep(sleep_time)
            else:
                self._viz_overrun += 1
                # Too far behind to catch up by sleeping less (e.g. the rate was
                # just raised): rebase rather than spin through a backlog.
                if sleep_time < -5 * period:
                    deadline = time.perf_counter()

    # ---- drawing --------------------------------------------------------
    def _viz_schedule_draw(self):
        """Schedule the next redraw from the Tk loop; no-op once closed."""
        if not self._viz_alive:
            return
        self._viz_draw_id = self._viz_win.after(_VIZ_DRAW_INTERVAL_MS, self._viz_redraw)

    def _viz_redraw(self):
        """Repaint the plot from the buffers (runs on the Tk thread)."""
        if not self._viz_alive:
            return
        win = self._viz_win
        if win is None or not win.winfo_exists():
            return

        ax = self._viz_ax
        try:
            # set_data() on persistent lines, never ax.clear(): clearing would
            # throw away the toolbar's zoom/pan state every frame.
            for key in self._viz_selected:
                buf = self._viz_buf.get(key)
                line = self._viz_lines.get(key)
                if buf is None or not buf:
                    continue
                if line is None:
                    # Colour by slot in the selection, not by signal family:
                    # every visible curve must be a different colour, including
                    # two signals from the same arm.
                    color = f"C{self._viz_color_slot(key) % 10}"
                    line, = ax.plot([], [], linewidth=1.0, color=color,
                                    label=viz_signal_label(key))
                    self._viz_lines[key] = line
                xs = [p[0] for p in buf]
                ys = [p[1] for p in buf]
                line.set_data(xs, ys)

            for key in list(self._viz_lines):
                if key not in self._viz_selected:
                    self._viz_lines.pop(key).remove()

            # While paused the window holds its position, so the plot reads as
            # frozen instead of sliding across data that is no longer arriving.
            if self._viz_t0:
                if self._viz_running:
                    # The newest sample, not the wall clock: a slow rate must not
                    # leave the view scrolled off ahead of the data.
                    last_t = 0.0
                    for buf in self._viz_buf.values():
                        if buf and buf[-1][0] > last_t:
                            last_t = buf[-1][0]
                    self._viz_view_end = max(last_t, self._viz_window)
                ax.set_xlim(self._viz_view_end - self._viz_window, self._viz_view_end)
            if self._viz_auto_var.get():
                ax.relim()
                ax.autoscale_view(scalex=False, scaley=True)
            # The legend is rebuilt in full every frame: a partial update would
            # leave ghost entries behind for signals the user just unticked.
            if self._viz_lines and self._viz_legend_wanted():
                ax.legend(loc="upper right", fontsize=8, ncol=2)
            elif not self._viz_lines and ax.get_legend() is not None:
                ax.get_legend().remove()
            self._viz_canvas.draw_idle()
        except Exception as e:
            print(f"[viz] redraw error: {e!r}")

        self._viz_update_status()
        self._viz_schedule_draw()

    def _viz_legend_wanted(self):
        """Legend only once it stops being a solid wall of text."""
        return len(self._viz_lines) <= _VIZ_LEGEND_MAX

    def _viz_color_slot(self, key):
        """Stable position of a signal in the selection, used to pick a colour.

        Assigning slots from the live selection keeps every curve a distinct
        colour. Removing one signal can shift the next one onto its colour, so
        the existing lines are repainted to match — see _viz_apply_selection.
        """
        try:
            return self._viz_color_slots.index(key)
        except ValueError:
            return len(self._viz_color_slots) + len(self._viz_lines)

    def _viz_update_status(self):
        if not self._viz_alive or not self._viz_status.winfo_exists():
            return
        elapsed = (time.perf_counter() - self._viz_t0) if self._viz_t0 else 0.0
        actual = self._viz_samples / elapsed if elapsed > 1e-6 else 0.0
        self._viz_status.config(
            text=f"{len(self._viz_selected)} signals | {self._viz_samples} samples | "
                 f"{actual:.1f} Hz | overrun {self._viz_overrun}")

    def _viz_clear(self):
        """Empty every buffer, rewind the clock and rebuild the lines."""
        self._viz_buf = {}
        self._viz_samples = 0
        self._viz_overrun = 0
        self._viz_t0 = 0.0
        self._viz_view_end = 0.0
        # Drop the Line2D objects too, otherwise the axes would keep showing the
        # old arrays after the buffers are gone.
        for key in list(self._viz_lines):
            self._viz_lines.pop(key).remove()
        self._viz_redraw()

    def _viz_close(self, win):
        """Stop the thread, cancel the redraw, then tear the window down."""
        self._viz_alive = False
        self._viz_running = False
        thread = self._viz_thread
        if thread is not None and thread.is_alive():
            thread.join(timeout=1.0)
        self._viz_thread = None
        if self._viz_draw_id is not None:
            try:
                self.root.after_cancel(self._viz_draw_id)
            except Exception:
                pass
            self._viz_draw_id = None
        self._viz_lines = {}
        self._viz_buf = {}
        self._viz_checked = {}
        self._viz_color_slots = []
        try:
            win.destroy()
        except Exception:
            pass
        self._viz_win = None

    def _viz_repaint_colors(self):
        """Re-apply colour slots to the lines already on the axes."""
        for key, line in self._viz_lines.items():
            line.set_color(f"C{self._viz_color_slot(key) % 10}")

    # ==================== Tool dynamics identification ====================
    def tool_dynamics_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        win = tk.Toplevel(self.root)
        win.title("Tool Dynamics Identification")
        win.geometry("1000x640")
        win.configure(bg="white")
        win.transient(self.root)
        win.resizable(True, True)
        win.grab_set()

        # per-dialog task state
        self._tdid_task = None
        self._tdid_para = None
        self._tdid_path = None
        self._tdid_poll_id = None
        self._tdid_start_time = 0.0

        # ---- configuration row ----
        cfg = tk.Frame(win, bg="white")
        cfg.pack(fill="x", padx=10, pady=(10, 5))

        tk.Label(cfg, text="Arm:", bg="white").grid(row=0, column=0, padx=(0, 5))
        self.tdid_arm_var = tk.StringVar(value="Arm0")
        ttk.Combobox(cfg, textvariable=self.tdid_arm_var, values=["Arm0", "Arm1"],
                     state="readonly", width=8).grid(row=0, column=1, padx=(0, 15))

        tk.Label(cfg, text="Load:", bg="white").grid(row=0, column=2, padx=(0, 5))
        self.tdid_load_var = tk.StringVar(value="NoLoad")
        ttk.Combobox(cfg, textvariable=self.tdid_load_var, values=["NoLoad", "Load"],
                     state="readonly", width=8).grid(row=0, column=3, padx=(0, 15))

        tk.Label(cfg, text="Traj file:", bg="white").grid(row=0, column=4, padx=(0, 5))
        self.tdid_path_var = tk.StringVar()
        tk.Entry(cfg, textvariable=self.tdid_path_var, width=70).grid(row=0, column=5, padx=(0, 5))
        tk.Button(cfg, text="Browse...", command=self._tdid_browse).grid(row=0, column=6)

        # ---- sample / stop / identify row ----
        btn = tk.Frame(win, bg="white")
        btn.pack(fill="x", padx=10, pady=5)
        
        self.tdid_start_btn = tk.Button(btn, text="Start collecting", bg="#A2CD5A",
                                        command=self._tdid_sample)
        self.tdid_start_btn.pack(side="left", padx=(0, 10))
        self.tdid_stop_btn = tk.Button(btn, text="Break", bg="#EC2A23", fg="white",
                                       state="disabled", command=self._tdid_stop)
        self.tdid_stop_btn.pack(side="left", padx=(0, 10))
        self.tdid_identify_btn = tk.Button(btn, text="Identify", bg="#4AA7EA",
                                           command=self._tdid_identify)
        self.tdid_identify_btn.pack(side="left")

        # ---- output ----
        tk.Label(win, text="Output:", bg="white").pack(anchor="w", padx=10)
        self.tdid_output = scrolledtext.ScrolledText(win, width=95, height=14, wrap=tk.WORD,
                                                     font=("Consolas", 10))
        self.tdid_output.pack(fill="both", expand=True, padx=10, pady=5)

        # ---- set tool dynamics ----
        setf = tk.Frame(win, bg="white")
        setf.pack(fill="x", padx=10, pady=(5, 10))
        tk.Label(setf, text="Tool dynamics  m, cx, cy, cz, Ixx, Ixy, Ixz, Iyy, Iyz, Izz :",
                 bg="white").pack(anchor="w")
        drow = tk.Frame(setf, bg="white")
        drow.pack(fill="x", pady=2)
        self.tdid_dyn_var = tk.StringVar(value="0,0,0,0,0,0,0,0,0,0")
        tk.Entry(drow, textvariable=self.tdid_dyn_var, width=62).pack(side="left", padx=(0, 5))
        tk.Button(drow, text="Use last result", command=self._tdid_use_result).pack(side="left", padx=(0, 5))
        tk.Button(drow, text="Set Tool dynamics", bg="#A2CD5A", command=self._tdid_set_dynamics).pack(side="left")

        win.protocol("WM_DELETE_WINDOW", lambda: self._tdid_on_close(win))

    def _tdid_browse(self):
        p = filedialog.askopenfilename(
            title="Select trajectory file",
            filetypes=[("FMV trajectory", "*.fmv"), ("All files", "*.*")])
        if p:
            self.tdid_path_var.set(p)

    def _tdid_log(self, msg):
        self.tdid_output.insert(tk.END, msg + "\n")
        self.tdid_output.see(tk.END)

    def _tdid_sample(self):
        if self._tdid_task is not None:
            messagebox.showwarning("Running", "A sampling task is already running.")
            return
        arm = 0 if self.tdid_arm_var.get() == "Arm0" else 1
        loadcode = 0 if self.tdid_load_var.get() == "NoLoad" else 1
        path = self.tdid_path_var.get().strip()
        if not path:
            messagebox.showwarning("No file", "Please select a trajectory (.fmv) file.")
            return
        if not messagebox.askyesno("Confirm", "The robot will move along the trajectory. Continue?"):
            return

        self._tdid_para = None
        self._tdid_path = path
        self.tdid_output.delete("1.0", tk.END)
        self._tdid_log("Start sampling: arm={}, load={}, file={}".format(arm, loadcode, path))
        try:
            handle = robot.start_sampling(arm, loadcode, path)
        except Exception as e:
            messagebox.showerror("Error", "start_sampling failed: {}".format(e))
            return
        if not handle:
            self._tdid_log("Failed to start sampling task (another task may be running).")
            return

        self._tdid_task = handle
        self._tdid_start_time = time.time()
        self.tdid_start_btn.config(state="disabled")
        self.tdid_stop_btn.config(state="normal")
        self._tdid_poll()

    def _tdid_poll(self):
        if self._tdid_task is None:
            return
        try:
            ret = robot.get_sampling_status(self._tdid_task)
        except Exception as e:
            self._tdid_log("Status query failed: {}".format(e))
            self._tdid_finish()
            return

        if not isinstance(ret, tuple):
            self._tdid_log("get_sampling_status returned {}.".format(ret))
            self._tdid_finish()
            return

        status, progress, error_code = ret
        self._tdid_log("status={}, progress={}%, error_code={}".format(status, progress, error_code))

        if status == ToolDynTaskStatus.TASK_DONE:
            self._tdid_log("Sampling done. Click Identify to compute load parameters.")
            self._tdid_cleanup_task()
            self._tdid_reset_buttons()
            return
        if status == ToolDynTaskStatus.TASK_ERROR:
            self._tdid_log("Sampling failed (error_code={}).".format(error_code))
            self._tdid_finish()
            return
        if status == ToolDynTaskStatus.TASK_STOPPED:
            self._tdid_log("Task stopped.")
            self._tdid_finish()
            return
        if time.time() - self._tdid_start_time > 300:
            self._tdid_log("Timeout (300s), stopping...")
            self._tdid_finish()
            return

        self._tdid_poll_id = self.root.after(1000, self._tdid_poll)

    def _tdid_cleanup_task(self):
        """Stop + destroy the sampling task and cancel polling (no button changes)."""
        task = self._tdid_task
        self._tdid_task = None
        if self._tdid_poll_id is not None:
            try:
                self.root.after_cancel(self._tdid_poll_id)
            except Exception:
                pass
            self._tdid_poll_id = None
        if task is not None:
            try:
                robot.stop_sampling(task)
            except Exception:
                pass
            try:
                robot.destroy_sampling(task)
            except Exception:
                pass

    def _tdid_identify(self):
        """Run load identification on the sampled data and show the result."""
        if self._tdid_task is not None:
            messagebox.showwarning("Running", "Sampling is still running. Stop it first.")
            return
        self._tdid_path=self.tdid_path_var.get().strip()
        if not self._tdid_path:
            messagebox.showwarning("No file", "Run sampling first to set the trajectory path.")
            return
        try:
            arm = 0 if self.tdid_arm_var.get() == "Arm0" else 1
            ret, error_code, para = robot.run_load_identification(arm, self._tdid_path)
        except Exception as e:
            self._tdid_log("run_load_identification failed: {}".format(e))
            return
        if ret != 0 or error_code != 0:
            self._tdid_log("Load identification failed (error_code={}).".format(error_code))
            return
        self._tdid_para = para
        self._tdid_log("Identification succeeded.")
        self._tdid_log("  m = {:.6f}".format(para.m))
        self._tdid_log("  r = [{:.6f}, {:.6f}, {:.6f}]".format(para.r[0], para.r[1], para.r[2]))
        inertia = self._tdid_inertia(para)  # -> Ixx, Ixy, Ixz, Iyy, Iyz, Izz
        self._tdid_log("  I = [Ixx={:.6f}, Ixy={:.6f}, Ixz={:.6f}, Iyy={:.6f}, Iyz={:.6f}, Izz={:.6f}]".format(*inertia))

    def _tdid_reset_buttons(self):
        self.tdid_start_btn.config(state="normal")
        self.tdid_stop_btn.config(state="disabled")

    def _tdid_finish(self):
        self._tdid_cleanup_task()
        self._tdid_reset_buttons()

    def _tdid_stop(self):
        if self._tdid_task is not None:
            self._tdid_log("User requested stop...")
            self._tdid_finish()

    def _tdid_on_close(self, win):
        if self._tdid_task is not None:
            self._tdid_finish()
        win.destroy()

    def _tdid_inertia(self, p):
        """Reorder the SDK inertia vector to the canonical order.

        LoadDynamicPara.I is stored as [Ixx, Iyy, Izz, Ixy, Ixz, Iyz]; return it as
        [Ixx, Ixy, Ixz, Iyy, Iyz, Izz], which is the order shown in the UI and
        expected by runtime_set_tool_d.
        """
        return [p.I[i] for i in (0, 3, 4, 1, 5, 2)]

    def _tdid_use_result(self):
        p = self._tdid_para
        if p is None:
            messagebox.showinfo("No result", "Run an identification first.")
            return
        vals = [p.m, p.r[0], p.r[1], p.r[2]] + self._tdid_inertia(p)
        self.tdid_dyn_var.set(",".join("{:.6f}".format(v) for v in vals))

    def _tdid_set_dynamics(self):
        if not self.connected:
            messagebox.showerror('Error', "Robot not connected")
            return
        try:
            vals = [float(x.strip()) for x in self.tdid_dyn_var.get().split(",")]
        except ValueError:
            messagebox.showerror("Error", "Tool dynamics must be 10 comma-separated numbers.")
            return
        if len(vals) != 10:
            messagebox.showerror("Error", "Tool dynamics must have exactly 10 values.")
            return
        obj = FXObjType.OBJ_ARM0 if self.tdid_arm_var.get() == "Arm0" else FXObjType.OBJ_ARM1
        try:
            ret = robot.runtime_set_tool_d(obj, vals)
        except Exception as e:
            messagebox.showerror("Error", "runtime_set_tool_d failed: {}".format(e))
            return
        if ret != 0:
            messagebox.showerror("Failed",
                                 "runtime_set_tool_d failed. Error msg:  {}".format(robot._get_operate_error_msg(ret)))
        else:
            self._tdid_log("Tool dynamics set on {}.".format(self.tdid_arm_var.get()))

    # ==================== Hand data communication ====================
    HAND_DATA_LEN = 512

    def hand_data_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        win = tk.Toplevel(self.root)
        win.title("Hand Data Communication")
        win.geometry("1000x800")
        win.configure(bg="white")
        win.transient(self.root)
        win.resizable(True, True)
        win.grab_set()

        # hand data manager (start/stop at top)
        self.hand_data_manager = getattr(self, "hand_data_manager", None) or HandDataManager(robot)
        cfg_top = tk.Frame(win, bg="white")
        cfg_top.pack(fill="x", padx=10, pady=(10, 5))
        self.hand_mgr_btn = tk.Button(cfg_top, text="Start Hand Data Manager", bg="#A2CD5A",
                                      command=self._hand_mgr_toggle)
        self.hand_mgr_btn.pack(side="left")
        self.hand_mgr_status = tk.Label(cfg_top, text="stopped", bg="white", fg="gray")
        self.hand_mgr_status.pack(side="left", padx=(10, 0))

        # hand selection (Arm0 / Arm1)
        cfg = tk.Frame(win, bg="white")
        cfg.pack(fill="x", padx=10, pady=(10, 5))
        tk.Label(cfg, text="Hand:", bg="white").grid(row=0, column=0, padx=(0, 5))
        self.hand_obj_var = tk.StringVar(value="Arm0")
        ttk.Combobox(cfg, textvariable=self.hand_obj_var, values=["Arm0", "Arm1"],
                     state="readonly", width=10).grid(row=0, column=1)

        # ---- send ----
        sendf = tk.Frame(win, bg="white")
        sendf.pack(fill="x", padx=10, pady=5)
        srow = tk.Frame(sendf, bg="white")
        srow.pack(fill="x", pady=2)
        tk.Label(srow, text="Send (hex, {} bytes):".format(self.HAND_DATA_LEN), bg="white").pack(side="left",
                                                                                                 padx=(0, 8))
        tk.Button(srow, text="Send", bg="#A2CD5A", command=self._hand_send).pack(side="left", padx=(0, 8))
        tk.Button(srow, text="All zero", command=self._hand_send_clear).pack(side="left")
        self.hand_send_text = scrolledtext.ScrolledText(sendf, width=82, height=14, wrap=tk.NONE,
                                                        font=("Consolas", 10))
        self.hand_send_text.pack(fill="x", pady=2)
        self._hand_send_clear()  # initialize to all-zero hex

        # ---- receive ----
        recvf = tk.Frame(win, bg="white")
        recvf.pack(fill="both", expand=True, padx=10, pady=5)
        rrow = tk.Frame(recvf, bg="white")
        rrow.pack(fill="x", pady=2)
        tk.Button(rrow, text="Refresh", bg="#A2CD5A", command=self._hand_recv_once).pack(side="left", padx=(0, 10))
        self.hand_auto_var = tk.IntVar(value=0)
        tk.Checkbutton(rrow, text="Auto refresh (from manager)", variable=self.hand_auto_var,
                       command=self._hand_auto_toggle, bg="white").pack(side="left", padx=(0, 10))
        self.hand_serial_label = tk.Label(rrow, text="serial: -", bg="white")
        self.hand_serial_label.pack(side="left")
        tk.Label(recvf, text="Received (hex, {} bytes):".format(self.HAND_DATA_LEN), bg="white").pack(anchor="w")
        self.hand_recv_text = scrolledtext.ScrolledText(recvf, width=82, height=14, wrap=tk.NONE,
                                                        font=("Consolas", 10))
        self.hand_recv_text.pack(fill="both", expand=True, pady=2)

        self._hand_recv_id = None
        win.protocol("WM_DELETE_WINDOW", lambda: self._hand_close(win))

    def _hand_obj(self):
        return FXObjType.OBJ_ARM0 if self.hand_obj_var.get() == "Arm0" else FXObjType.OBJ_ARM1

    @staticmethod
    def _bytes_to_hex_text(data):
        hx = data.hex().upper()
        bl = [hx[i:i + 2] for i in range(0, len(hx), 2)]
        return "\n".join(" ".join(bl[i:i + 50]) for i in range(0, len(bl), 50))

    @staticmethod
    def _parse_hex_padded(hex_str, n=512):
        hs = hex_str.replace(' ', '').replace('\n', '').replace('\r', '').replace('\t', '')
        raw = bytes.fromhex(hs) if hs else b''
        if len(raw) < n:
            raw = raw + bytes(n - len(raw))
        return raw[:n]

    def _hand_send_clear(self):
        self.hand_send_text.delete("1.0", tk.END)
        self.hand_send_text.insert("1.0", self._bytes_to_hex_text(bytes(self.HAND_DATA_LEN)))

    def _hand_send(self):
        hex_str = self.hand_send_text.get("1.0", "end-1c")
        try:
            sent = self._parse_hex_padded(hex_str, self.HAND_DATA_LEN)
        except ValueError:
            messagebox.showerror("Error", "Invalid hex string")
            return
        ret = robot.hand_set_data(self._hand_obj(), hex_str)
        if ret != 0:
            messagebox.showerror("Failed", "hand_set_data failed. Error msg:  {}".format(robot._get_operate_error_msg(ret)))
            return
        # show the full 512-byte data that was actually sent
        self.hand_send_text.delete("1.0", tk.END)
        self.hand_send_text.insert("1.0", self._bytes_to_hex_text(sent))

    def _hand_show_recv(self, serial, data):
        self.hand_serial_label.config(text="serial: {}".format(serial))
        self.hand_recv_text.delete("1.0", tk.END)
        self.hand_recv_text.insert("1.0", self._bytes_to_hex_text(data))

    def _hand_recv_once(self):
        try:
            ret, serial, data = robot.hand_get_data(self._hand_obj())
        except Exception as e:
            messagebox.showerror("Error", "hand_get_data failed: {}".format(e))
            return
        if ret != 0:
            messagebox.showerror("Failed", "hand_get_data failed. Error msg:  {}".format(robot._get_operate_error_msg(ret)))
            return
        self._hand_show_recv(serial, data)

    def _hand_auto_toggle(self):
        if self.hand_auto_var.get():
            if not self.hand_data_manager.is_running:
                self._hand_mgr_toggle()
            self._hand_recv_loop()
        elif self._hand_recv_id is not None:
            try:
                self.root.after_cancel(self._hand_recv_id)
            except Exception:
                pass
            self._hand_recv_id = None

    def _hand_mgr_toggle(self):
        if self.hand_data_manager.is_running:
            self.hand_data_manager.stop()
            self.hand_auto_var.set(0)
            if self._hand_recv_id is not None:
                try:
                    self.root.after_cancel(self._hand_recv_id)
                except Exception:
                    pass
                self._hand_recv_id = None
            self.hand_mgr_btn.config(text="Start Hand Data Manager", bg="#A2CD5A")
            self.hand_mgr_status.config(text="stopped", fg="gray")
        else:
            self.hand_data_manager.start()
            self.hand_mgr_btn.config(text="Stop Hand Data Manager", bg="#F44336")
            self.hand_mgr_status.config(text="running", fg="green")

    def _hand_recv_loop(self):
        if not self.hand_auto_var.get() or not self.hand_data_manager.is_running:
            return
        # Never let a transient decode error escape: this runs on the Tk event
        # loop, so an unhandled exception would tear down mainloop and kill the
        # whole app. The loop must always be rescheduled or the panel freezes.
        try:
            latest_in = self.hand_data_manager.latest_hand_in
            if latest_in and "error" not in latest_in:
                datas = latest_in.get("data") or [None, None]
                idx = 0 if self.hand_obj_var.get() == "Arm0" else 1
                data = datas[idx] if len(datas) > idx else None
                if data is not None:
                    self._hand_show_recv(latest_in.get("frame_serial", "-"), data)
        except Exception as e:
            print(f"[hand_recv] exception: {e!r}")
        finally:
            if self.hand_auto_var.get() and self.hand_data_manager.is_running:
                self._hand_recv_id = self.root.after(100, self._hand_recv_loop)
            else:
                self._hand_recv_id = None

    def _hand_close(self, win):
        self.hand_auto_var.set(0)
        if self._hand_recv_id is not None:
            try:
                self.root.after_cancel(self._hand_recv_id)
            except Exception:
                pass
            self._hand_recv_id = None
        if getattr(self, "hand_data_manager", None) and self.hand_data_manager.is_running:
            self.hand_data_manager.stop()
        self.hand_mgr_btn.config(text="Start Hand Data Manager", bg="#A2CD5A")
        self.hand_mgr_status.config(text="stopped", fg="gray")
        win.destroy()
        if self._hand_recv_id is not None:
            try:
                self.root.after_cancel(self._hand_recv_id)
            except Exception:
                pass
            self._hand_recv_id = None
        win.destroy()

    def sensor_decoder_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return
        sensor_encoder_window = tk.Toplevel(self.root)
        sensor_encoder_window.title("Sensor and encoder functions")
        sensor_encoder_window.geometry("800x400")
        sensor_encoder_window.configure(bg="white")
        sensor_encoder_window.transient(self.root)
        sensor_encoder_window.resizable(True, True)
        sensor_encoder_window.grab_set()

        self.sensor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_2.pack(fill="x", padx=5, pady=(15, 10))
        self.sensor_main_tex = tk.Label(self.sensor_frame_2, text="Sensor offset reset", bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.sensor_main_tex.pack(fill='x', padx=(5, 20))

        self.sensor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.sensor_frame_1.pack(fill="x")
        self.axis_text_ = tk.Label(self.sensor_frame_1, text="Arm0", bg="#D8F4F3")
        self.axis_text_.grid(row=0, column=0, padx=(5, 5))
        # resetoffset
        self.get_offset_btn_1 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm0'))
        self.get_offset_btn_1.grid(row=0, column=1, padx=(0, 20))

        self.axis_text__ = tk.Label(self.sensor_frame_1, text="Arm1", bg="#F4E4D8")
        self.axis_text__.grid(row=0, column=2, padx=(5, 5))

        self.get_offset_btn_2 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Arm1'))
        self.get_offset_btn_2.grid(row=0, column=3, padx=(0, 20))

        self.axis_text___ = tk.Label(self.sensor_frame_1, text="Body", bg="#B0C4DE")
        self.axis_text___.grid(row=0, column=4, padx=(5, 5))

        self.get_offset_btn_3 = tk.Button(self.sensor_frame_1, text="ResetOffset",
                                          command=lambda: self.clear_sensor_offset('Body'))
        self.get_offset_btn_3.grid(row=0, column=5, padx=5)

        '''encoder'''
        self.encoder_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.encoder_frame_1.pack(fill="x", padx=5, pady=(25, 10))
        self.encoder_frame_1 = tk.Label(self.encoder_frame_1, text="Motor encoder zeroing and error clearing",
                                        bg="#2196F3",
                                        fg="white", font=("Arial", 10, "bold"))
        self.encoder_frame_1.pack(fill='x')
        '''left arm'''
        self.motor_frame_1 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_1.pack(fill="x")
        self.motor_text_1 = tk.Label(self.motor_frame_1, text="Arm0", bg="#D8F4F3", width=8)
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_1 = tk.Button(self.motor_frame_1, text="Motor encoder zeroing",
                                     command=lambda: self.clear_motor_as_zero('Arm0', self.motor_btn_1))
        self.motor_btn_1.grid(row=0, column=1, padx=5, pady=5)

        self.disable_soft_btn_1 = tk.Button(self.motor_frame_1, text="Disable SoftLimit",
                                            command=lambda: self.disable_soft_limit('Arm0', 0xFF))
        self.disable_soft_btn_1.grid(row=0, column=2, padx=5)

        self.motor_btn_3 = tk.Button(self.motor_frame_1, text="Encoder clearing error", bg="#7ED2B4", state="disabled")
        # command=lambda: self.clear_motor_error('Arm0'))
        self.motor_btn_3.grid(row=0, column=3, padx=5)

        '''right arm'''
        self.motor_frame_2 = tk.Frame(sensor_encoder_window, bg="white")
        self.motor_frame_2.pack(fill="x")

        self.motor_text_1 = tk.Label(self.motor_frame_2, text="Arm1", bg="#F4E4D8", width=8)
        self.motor_text_1.grid(row=0, column=0, padx=(5, 5))

        self.motor_btn_11 = tk.Button(self.motor_frame_2, text="Motor encoder zeroing",
                                      command=lambda: self.clear_motor_as_zero('Arm1', self.motor_btn_11))
        self.motor_btn_11.grid(row=0, column=1, padx=5)

        self.disable_soft_btn_2 = tk.Button(self.motor_frame_2, text="Disable SoftLimit",
                                            command=lambda: self.disable_soft_limit('Arm1', 0xFF))
        self.disable_soft_btn_2.grid(row=0, column=2, padx=5)

        self.motor_btn_31 = tk.Button(self.motor_frame_2, text="Encoder clearing error", bg="#7ED2B4", state="disabled")
        # command=lambda: self.clear_motor_error('Arm1'))
        self.motor_btn_31.grid(row=0, column=3, padx=5)

        '''body'''
        motor_frame_3 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_3.pack(fill="x")
        motor_text_3 = tk.Label(motor_frame_3, text="Body", bg="#B0C4DE", width=8)
        motor_text_3.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_body = tk.Button(motor_frame_3, text="Motor encoder zeroing",
                                        command=lambda: self.clear_motor_as_zero('Body', self.motor_btn_body))
        self.motor_btn_body.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_body = tk.Button(motor_frame_3, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit('Body', 0xFF))
        self.disable_soft_btn_body.grid(row=0, column=3, padx=5)

        '''Head'''
        motor_frame_4 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_4.pack(fill="x")
        motor_text_4 = tk.Label(motor_frame_4, text="Head", bg="#D8BFD8", width=8)
        motor_text_4.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_head = tk.Button(motor_frame_4, text="Motor encoder zeroing",
                                        command=lambda: self.clear_motor_as_zero('Head', self.motor_btn_head))
        self.motor_btn_head.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_head = tk.Button(motor_frame_4, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit('Head', 0xFF))
        self.disable_soft_btn_head.grid(row=0, column=3, padx=5)

        '''Lift'''
        motor_frame_5 = tk.Frame(sensor_encoder_window, bg="white")
        motor_frame_5.pack(fill="x")
        motor_text_5 = tk.Label(motor_frame_5, text="Lift", bg="#FFFACD", width=8)
        motor_text_5.grid(row=0, column=0, padx=(5, 5))
        self.motor_btn_lift = tk.Button(motor_frame_5, text="Motor encoder zeroing",
                                        command=lambda: self.clear_motor_as_zero('Lift', self.motor_btn_lift))
        self.motor_btn_lift.grid(row=0, column=1, padx=5, pady=5)
        self.disable_soft_btn_lift = tk.Button(motor_frame_5, text="Disable SoftLimit",
                                               command=lambda: self.disable_soft_limit('Lift', 0x03))
        self.disable_soft_btn_lift.grid(row=0, column=3, padx=5)

    # ==================== EtherCAT SDO read / write ====================
    # data type name -> (read method name, write method name on GentoRobot, min, max)
    SDO_TYPES = {
        "int8":   ("config_sdo_read_int8",   "config_sdo_write_int8",   -2**7,        2**7 - 1),
        "uint8":  ("config_sdo_read_uint8",  "config_sdo_write_uint8",   0,           2**8 - 1),
        "int16":  ("config_sdo_read_int16",  "config_sdo_write_int16",  -2**15,       2**15 - 1),
        "uint16": ("config_sdo_read_uint16", "config_sdo_write_uint16",  0,           2**16 - 1),
        "int32":  ("config_sdo_read_int32",  "config_sdo_write_int32",  -2**31,       2**31 - 1),
        "uint32": ("config_sdo_read_uint32", "config_sdo_write_uint32",  0,           2**32 - 1),
    }

    def sdo_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        win = tk.Toplevel(self.root)
        win.title("EtherCAT SDO")
        win.geometry("900x300")
        win.configure(bg="white")
        win.transient(self.root)
        win.resizable(False, False)
        win.grab_set()

        self._sdo_range_label = {}

        def type_range_text(dtype):
            return self._sdo_range_text(dtype)

        # ---------- Read row ----------
        read_frame = tk.Frame(win, bg="white")
        read_frame.pack(fill="x", padx=15, pady=(15, 5))
        tk.Label(read_frame, text="Read SDO", bg="#2196F3", fg="white",
                 font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=9, sticky="w", pady=(0, 4))
        tk.Label(read_frame, text="master:", bg="white").grid(row=1, column=0, padx=(0, 3))
        sdo_read_master = tk.Spinbox(read_frame, from_=0, to=2, width=5)
        sdo_read_master.grid(row=1, column=1, padx=(0, 10))
        tk.Label(read_frame, text="slave:", bg="white").grid(row=1, column=2, padx=(0, 3))
        sdo_read_slave = tk.Spinbox(read_frame, from_=0, to=15, width=5)
        sdo_read_slave.grid(row=1, column=3, padx=(0, 10))
        tk.Label(read_frame, text="index (hex):", bg="white").grid(row=1, column=4, padx=(0, 3))
        sdo_read_index = tk.Entry(read_frame, width=10)
        sdo_read_index.insert(0, "0x0000")
        sdo_read_index.grid(row=1, column=5, padx=(0, 10))
        tk.Label(read_frame, text="subindex (hex):", bg="white").grid(row=1, column=6, padx=(0, 3))
        sdo_read_subindex = tk.Entry(read_frame, width=6)
        sdo_read_subindex.insert(0, "0x00")
        sdo_read_subindex.grid(row=1, column=7, padx=(0, 10))
        sdo_read_type_var = tk.StringVar(value="uint32")
        ttk.Combobox(read_frame, textvariable=sdo_read_type_var, values=list(self.SDO_TYPES.keys()),
                     state="readonly", width=8).grid(row=1, column=8, padx=(0, 5))
        sdo_read_range_lbl = tk.Label(read_frame, text=type_range_text(sdo_read_type_var.get()),
                                      bg="white", fg="gray", font=("Arial", 8))
        sdo_read_range_lbl.grid(row=1, column=9, padx=(0, 10))
        self._sdo_range_label['read'] = (sdo_read_type_var, sdo_read_range_lbl)
        tk.Button(read_frame, text="Read", bg="#A2CD5A", width=8,
                  command=lambda: self._sdo_do_read(sdo_read_master, sdo_read_slave, sdo_read_index,
                                                    sdo_read_subindex, sdo_read_type_var)
                  ).grid(row=1, column=10)
        sdo_read_result_var = tk.StringVar(value="value: -")
        tk.Label(read_frame, textvariable=sdo_read_result_var, bg="white", fg="black",
                 font=("Consolas", 10), anchor='w').grid(row=2, column=0, columnspan=11, sticky="w", pady=(4, 0))
        self._sdo_read_result_var = sdo_read_result_var

        # ---------- Write row ----------
        write_frame = tk.Frame(win, bg="white")
        write_frame.pack(fill="x", padx=15, pady=(15, 10))
        tk.Label(write_frame, text="Write SDO", bg="#EC7014", fg="white",
                 font=("Arial", 10, "bold")).grid(row=0, column=0, columnspan=9, sticky="w", pady=(0, 4))
        tk.Label(write_frame, text="master:", bg="white").grid(row=1, column=0, padx=(0, 3))
        sdo_write_master = tk.Spinbox(write_frame, from_=0, to=2, width=5)
        sdo_write_master.grid(row=1, column=1, padx=(0, 10))
        tk.Label(write_frame, text="slave:", bg="white").grid(row=1, column=2, padx=(0, 3))
        sdo_write_slave = tk.Spinbox(write_frame, from_=0, to=15, width=5)
        sdo_write_slave.grid(row=1, column=3, padx=(0, 10))
        tk.Label(write_frame, text="index (hex):", bg="white").grid(row=1, column=4, padx=(0, 3))
        sdo_write_index = tk.Entry(write_frame, width=10)
        sdo_write_index.insert(0, "0x0000")
        sdo_write_index.grid(row=1, column=5, padx=(0, 10))
        tk.Label(write_frame, text="subindex (hex):", bg="white").grid(row=1, column=6, padx=(0, 3))
        sdo_write_subindex = tk.Entry(write_frame, width=6)
        sdo_write_subindex.insert(0, "0x00")
        sdo_write_subindex.grid(row=1, column=7, padx=(0, 10))
        sdo_write_type_var = tk.StringVar(value="uint32")
        ttk.Combobox(write_frame, textvariable=sdo_write_type_var, values=list(self.SDO_TYPES.keys()),
                     state="readonly", width=8).grid(row=1, column=8, padx=(0, 5))
        sdo_write_range_lbl = tk.Label(write_frame, text=type_range_text(sdo_write_type_var.get()),
                                       bg="white", fg="gray", font=("Arial", 8))
        sdo_write_range_lbl.grid(row=1, column=9, padx=(0, 10))
        self._sdo_range_label['write'] = (sdo_write_type_var, sdo_write_range_lbl)
        tk.Label(write_frame, text="value:", bg="white").grid(row=2, column=0, padx=(0, 3), pady=(4, 0))
        sdo_write_value = tk.Entry(write_frame, width=14)
        sdo_write_value.grid(row=2, column=1, columnspan=2, sticky="w", pady=(4, 0))
        tk.Button(write_frame, text="Write", bg="#EC2A23", fg="white", width=8,
                  command=lambda: self._sdo_do_write(sdo_write_master, sdo_write_slave, sdo_write_index,
                                                     sdo_write_subindex, sdo_write_type_var, sdo_write_value)
                  ).grid(row=2, column=3, sticky="w", pady=(4, 0))
        sdo_write_result_var = tk.StringVar(value="")
        tk.Label(write_frame, textvariable=sdo_write_result_var, bg="white", fg="black",
                 font=("Consolas", 10), anchor='w').grid(row=3, column=0, columnspan=11, sticky="w", pady=(4, 0))
        self._sdo_write_result_var = sdo_write_result_var

        # Refresh the range hints when the type selection changes.
        sdo_read_type_var.trace_add("write", lambda *_: self._sdo_type_combo_changed('read'))
        sdo_write_type_var.trace_add("write", lambda *_: self._sdo_type_combo_changed('write'))

    @staticmethod
    def _sdo_range_text(dtype):
        """Hex range hint for a SDO data type, e.g. 'range: [0x8000, 0x7FFF]'."""
        _, _, lo, hi = App.SDO_TYPES[dtype]
        bits = {'int8': 8, 'uint8': 8, 'int16': 16, 'uint16': 16, 'int32': 32, 'uint32': 32}[dtype]
        mask = (1 << bits) - 1
        # Signed types show their two's-complement hex bounds.
        lo_hex = f"0x{lo & mask:X}"
        hi_hex = f"0x{hi & mask:X}" if lo < 0 else f"0x{hi:X}"
        return f"range: [{lo_hex}, {hi_hex}]"

    def _sdo_type_combo_changed(self, which):
        if not hasattr(self, '_sdo_range_label'):
            return
        entry = self._sdo_range_label.get(which)
        if not entry:
            return
        var, lbl = entry
        lbl.config(text=self._sdo_range_text(var.get()))

    def _sdo_parse_master_slave(self, master_w, slave_w):
        try:
            master = int(master_w.get())
        except ValueError:
            raise ValueError("master must be an integer in [0, 2]")
        try:
            slave = int(slave_w.get())
        except ValueError:
            raise ValueError("slave must be an integer in [0, 15]")
        if not 0 <= master <= 2:
            raise ValueError("master out of range [0, 2]")
        if not 0 <= slave <= 15:
            raise ValueError("slave out of range [0, 15]")
        return master, slave

    def _sdo_do_read(self, master_w, slave_w, index_w, subindex_w, type_var):
        try:
            if not self.connected:
                messagebox.showerror('Error', 'Robot not connected')
                return
            master, slave = self._sdo_parse_master_slave(master_w, slave_w)
            # index/subindex are hex strings ('0x6040' or '6040'); the SDK
            # wrapper parses and range-checks them.
            index = index_w.get().strip()
            subindex = subindex_w.get().strip()
            dtype = type_var.get()
            read_name, _, _, _ = self.SDO_TYPES[dtype]
            ret, value = getattr(robot, read_name)(master, slave, index, subindex)
            if ret != 0:
                self._sdo_read_result_var.set(f"value: read failed - {robot._get_operate_error_msg(ret)}")
            else:
                bits = int(dtype.replace('int', '').replace('uint', ''))
                self._sdo_read_result_var.set(f"value: {value} (0x{value & (2**bits - 1):X}) [{dtype}]")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def _sdo_do_write(self, master_w, slave_w, index_w, subindex_w, type_var, value_w):
        try:
            if not self.connected:
                messagebox.showerror('Error', 'Robot not connected')
                return
            master, slave = self._sdo_parse_master_slave(master_w, slave_w)
            # index/subindex are hex strings ('0x6040' or '6040'); the SDK
            # wrapper parses and range-checks them.
            index = index_w.get().strip()
            subindex = subindex_w.get().strip()
            dtype = type_var.get()
            _, write_name, lo, hi = self.SDO_TYPES[dtype]
            raw = value_w.get().strip()
            try:
                value = int(raw, 0)
            except ValueError:
                raise ValueError(f"value '{raw}' is not a valid integer (decimal or 0x.. hex)")
            if not lo <= value <= hi:
                raise ValueError(f"value out of range for {dtype}: {self._sdo_range_text(dtype)[8:-1]}")
            if not messagebox.askyesno("Confirm", f"Write {value} to master {master}, slave {slave}, "
                                                  f"index {index}, subindex {subindex} ({dtype})?"):
                return
            ret = getattr(robot, write_name)(master, slave, index, subindex, value)
            if ret != 0:
                self._sdo_write_result_var.set(f"write failed - {robot._get_operate_error_msg(ret)}")
            else:
                self._sdo_write_result_var.set(f"write ok: {value} ({dtype})")
        except Exception as e:
            messagebox.showerror("Error", str(e))

    def servo_sensor_version_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return
        servo_sensor_version_window = tk.Toplevel(self.root)
        servo_sensor_version_window.title("Version")
        servo_sensor_version_window.geometry("1000x800")
        servo_sensor_version_window.configure(bg="white")
        servo_sensor_version_window.transient(self.root)
        servo_sensor_version_window.resizable(True, True)
        servo_sensor_version_window.grab_set()

        # ---------- System version section ----------
        sys_frame = tk.Frame(servo_sensor_version_window, bg="white")
        sys_frame.pack(fill="x", padx=5, pady=(15, 5))
        sys_label_title = tk.Label(sys_frame, text="System & SDK version", bg="#2196F3",
                                   fg="white", font=("Arial", 10, "bold"))
        sys_label_title.pack(fill='x', padx=(5, 20))

        row_sys = tk.Frame(sys_frame, bg="white")
        row_sys.pack(fill="x", padx=(5, 0), pady=2)
        label_sys = tk.Label(row_sys, text="System:", width=10, anchor='w', bg="white")
        label_sys.pack(side="left")
        self.system_version_label = tk.Label(row_sys, text="", bg="white", fg="black",
                                             font=("Arial", 9), anchor='w')
        self.system_version_label.pack(side="left", fill="x", expand=True, padx=5)

        label_sdk = tk.Label(row_sys, text="SDK:", width=10, anchor='w', bg="white")
        label_sdk.pack(side="left")
        self.sdk_version_label = tk.Label(row_sys, text="", bg="white", fg="black",
                                          font=("Arial", 9), anchor='w')
        self.sdk_version_label.pack(side="left", fill="x", expand=True, padx=5)

        # ---------- Servo version section ----------
        ss_frame1 = tk.Frame(servo_sensor_version_window, bg="white")
        ss_frame1.pack(fill="x", padx=5, pady=(15, 10))
        servo_text = tk.Label(ss_frame1, text="Servo version", bg="#2196F3",
                              fg="white", font=("Arial", 10, "bold"))
        servo_text.pack(fill='x', padx=(5, 20))

        devices_servo = ["Arm0", "Arm1", "Body", "Head"]
        self.servo_version_labels = {}
        self.servo_config_version_labels = {}
        for device in devices_servo:
            row_frame = tk.Frame(ss_frame1, bg="white")
            row_frame.pack(fill="x", padx=20, pady=2)
            label = tk.Label(row_frame, text=f"{device}:", width=12, anchor='w', bg="white")
            label.pack(side="left")
            val_label = tk.Label(row_frame, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            val_label.pack(side="left", fill="x", expand=True, padx=5)
            self.servo_version_labels[device] = val_label

            row_cfg = tk.Frame(ss_frame1, bg="white")
            row_cfg.pack(fill="x", padx=20, pady=(0, 2))
            label_cfg = tk.Label(row_cfg, text=f"{device} config:", width=12, anchor='w', bg="white")
            label_cfg.pack(side="left")
            cfg_label = tk.Label(row_cfg, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            cfg_label.pack(side="left", fill="x", expand=True, padx=5)
            self.servo_config_version_labels[device] = cfg_label

        # ---------- Sensor version & Serial section ----------
        ss_frame2 = tk.Frame(servo_sensor_version_window, bg="white")
        ss_frame2.pack(fill="x", padx=5, pady=(15, 10))
        sensor_text = tk.Label(ss_frame2, text="Sensor version & Serial", bg="#2196F3",
                               fg="white", font=("Arial", 10, "bold"))
        sensor_text.pack(fill='x', padx=(5, 20))

        devices_sensor = ["Arm0", "Arm1", "Body"]
        self.sensor_version_labels = {}
        self.sensor_serial_labels = {}
        for device in devices_sensor:
            row_ver = tk.Frame(ss_frame2, bg="white")
            row_ver.pack(fill="x", padx=20, pady=(2, 0))
            label_ver = tk.Label(row_ver, text=f"{device} sensor:", width=12, anchor='w', bg="white")
            label_ver.pack(side="left")
            ver_label = tk.Label(row_ver, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            ver_label.pack(side="left", fill="x", expand=True, padx=5)
            self.sensor_version_labels[device] = ver_label

            row_ser = tk.Frame(ss_frame2, bg="white")
            row_ser.pack(fill="x", padx=20, pady=(0, 2))
            label_ser = tk.Label(row_ser, text=f"{device} serial:", width=12, anchor='w', bg="white")
            label_ser.pack(side="left")
            ser_label = tk.Label(row_ser, text="", bg="white", fg="black",
                                 font=("Arial", 9), anchor='w')
            ser_label.pack(side="left", fill="x", expand=True, padx=5)
            self.sensor_serial_labels[device] = ser_label

        # ---------- Hand type section (after sensor serials) ----------
        self.hand_type_labels = {}
        devices_hand = ["Hand0", "Hand1"]
        obj_types_hand = [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1]
        for device, obj_type in zip(devices_hand, obj_types_hand):
            row_ht = tk.Frame(ss_frame2, bg="white")
            row_ht.pack(fill="x", padx=20, pady=(0, 2))
            label_ht = tk.Label(row_ht, text=f"{device} type:", width=12, anchor='w', bg="white")
            label_ht.pack(side="left")
            ht_label = tk.Label(row_ht, text="", bg="white", fg="black",
                                font=("Arial", 9), anchor='w')
            ht_label.pack(side="left", fill="x", expand=True, padx=5)
            self.hand_type_labels[device] = ht_label

        # ---------- Physical state section (new) ----------
        phys_frame = tk.Frame(servo_sensor_version_window, bg="white")
        phys_frame.pack(fill="x", padx=5, pady=(15, 10))
        phys_title = tk.Label(phys_frame, text="Physical State", bg="#4CAF50",
                              fg="white", font=("Arial", 10, "bold"))
        phys_title.pack(fill='x', padx=(5, 20))

        devices_phys = ["Arm0", "Arm1", "Body", "Head", "Lift"]
        self.physical_state_labels = {}
        for device in devices_phys:
            row = tk.Frame(phys_frame, bg="white")
            row.pack(fill="x", padx=20, pady=2)
            label = tk.Label(row, text=f"{device}:", width=10, anchor='w', bg="white")
            label.pack(side="left")
            state_label = tk.Label(row, text="", bg="white", fg="black",
                                   font=("Arial", 9), anchor='w')
            state_label.pack(side="left", fill="x", expand=True, padx=5)
            self.physical_state_labels[device] = state_label

        self.system_version_label.config(text=getattr(self, 'sys_version', 'Not connected'))
        self.sdk_version_label.config(text=getattr(self, 'sdk_version', 'Not connected'))

        for device in self.physical_state_labels:
            self.physical_state_labels[device].config(
                text=getattr(self, 'physical_states', {}).get(device, 'Not connected'))

        for device in self.servo_version_labels:
            self.servo_version_labels[device].config(
                text=getattr(self, 'servo_versions', {}).get(device, 'Not connected'))
        for device in self.servo_config_version_labels:
            self.servo_config_version_labels[device].config(
                text=getattr(self, 'servo_cfg_versions', {}).get(device, 'Not connected'))
        for device in self.sensor_version_labels:
            self.sensor_version_labels[device].config(
                text=getattr(self, 'sensor_versions', {}).get(device, 'Not connected'))
        for device in self.sensor_serial_labels:
            self.sensor_serial_labels[device].config(
                text=getattr(self, 'sensor_serials', {}).get(device, 'Not connected'))

        # Hand types, read live from the SDK each time the dialog opens.
        for device, obj_type in zip(["Hand0", "Hand1"], [FXObjType.OBJ_ARM0, FXObjType.OBJ_ARM1]):
            if not self.connected:
                text = "Not connected"
            else:
                try:
                    ret, hand_type = robot.hand_get_type(obj_type)
                    if ret == 0:
                        text = hand_type
                    else:
                        text = f"Error msg: {robot._get_operate_error_msg(ret)}"
                except Exception as e:
                    text = f"Error: {e}"
            self.hand_type_labels[device].config(text=text)

    def planning_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return
        if not messagebox.askyesno("Yes", "Motion planning must under position state, and speed and accleration in main UI must be 100"):
            return

        planning_window = tk.Toplevel(self.root)
        planning_window.title("Motion Planning")
        planning_window.geometry("1200x800")
        planning_window.configure(bg="white")
        planning_window.transient(self.root)
        planning_window.resizable(True, True)
        planning_window.grab_set()

        # Canvas + Scrollbar
        canvas = tk.Canvas(planning_window, bg="white", highlightthickness=0)
        scrollbar = tk.Scrollbar(planning_window, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=scrollbar.set)
        scrollbar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        inner_frame = tk.Frame(canvas, bg="white")
        canvas.create_window((0, 0), window=inner_frame, anchor="nw", width=canvas.winfo_width())

        def configure_inner_frame(event):
            canvas.configure(scrollregion=canvas.bbox("all"))

        inner_frame.bind("<Configure>", configure_inner_frame)

        def configure_canvas(event):
            canvas.itemconfig(1, width=event.width)

        canvas.bind("<Configure>", configure_canvas)

        def on_mousewheel(event):
            canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        canvas.bind_all("<MouseWheel>", on_mousewheel)  # Windows
        canvas.bind_all("<Button-4>", lambda e: canvas.yview_scroll(-1, "units"))
        canvas.bind_all("<Button-5>", lambda e: canvas.yview_scroll(1, "units"))

        def unbind_mousewheel(event=None):
            canvas.unbind_all("<MouseWheel>")
            canvas.unbind_all("<Button-4>")
            canvas.unbind_all("<Button-5>")

        planning_window.bind("<Destroy>", unbind_mousewheel)

        joint_frame_1 = tk.Frame(inner_frame, bg="white")
        joint_frame_1.pack(fill="x", padx=5, pady=(5, 5))
        joint_title_text = tk.Label(joint_frame_1, text="Joint Space", bg="#2196F3",
                                    fg="white", font=("Arial", 10, "bold"))
        joint_title_text.pack(fill='x', padx=(5, 20))

        # JOINTS TO JOINTS
        func1_frame = ttk.LabelFrame(inner_frame, text="JOINTS TO JOINTS", padding=10,
                                     relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        func1_frame.pack(fill="x", padx=10, pady=(0, 5))

        arm0_row1 = tk.Frame(func1_frame, bg="white")
        arm0_row1.pack(fill="x", pady=2)
        tk.Label(arm0_row1, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_row1, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_start_arm0_entry = tk.Entry(arm0_row1, width=50)
        self.joints_start_arm0_entry.pack(side="left", padx=2)
        self.joints_start_arm0_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm0_row1, text="GetCur",
                  command=lambda: self.pln_get_cur_joints('Arm0')).pack(side="left",
                                                                        padx=2)
        tk.Label(arm0_row1, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_end_arm0_entry = tk.Entry(arm0_row1, width=50)
        self.joints_end_arm0_entry.pack(side="left", padx=2)
        self.joints_end_arm0_entry.insert(0, "17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134")

        arm1_row1 = tk.Frame(func1_frame, bg="white")
        arm1_row1.pack(fill="x", pady=2)
        tk.Label(arm1_row1, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_row1, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_start_arm1_entry = tk.Entry(arm1_row1, width=50)
        self.joints_start_arm1_entry.pack(side="left", padx=2)
        self.joints_start_arm1_entry.insert(0, "0,0,0,0,0,0,0")
        tk.Button(arm1_row1, text="GetCur",
                  command=lambda: self.pln_get_cur_joints('Arm1')).pack(side="left",
                                                                        padx=2)
        tk.Label(arm1_row1, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.joints_end_arm1_entry = tk.Entry(arm1_row1, width=50)
        self.joints_end_arm1_entry.pack(side="left", padx=2)
        self.joints_end_arm1_entry.insert(0, "-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134")

        params_row1 = tk.Frame(func1_frame, bg="white")
        params_row1.pack(fill="x", pady=5)
        tk.Label(params_row1, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                     padx=10)
        tk.Label(params_row1, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_freq_entry = tk.Entry(params_row1, width=6)
        self.joints_freq_entry.pack(side="left", padx=2)
        self.joints_freq_entry.insert(0, "50")
        tk.Label(params_row1, text="(1000%freq==0)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                    padx=(0, 5))

        tk.Label(params_row1, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_vel_entry = tk.Entry(params_row1, width=6)
        self.joints_vel_entry.pack(side="left", padx=2)
        self.joints_vel_entry.insert(0, "0.1")
        tk.Label(params_row1, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=(0, 5))

        tk.Label(params_row1, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.joints_acc_entry = tk.Entry(params_row1, width=6)
        self.joints_acc_entry.pack(side="left", padx=2)
        self.joints_acc_entry.insert(0, "0.1")
        tk.Label(params_row1, text="(0.01~1)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        btn_row1 = tk.Frame(func1_frame, bg="white")
        btn_row1.pack(pady=5, anchor="center")
        tk.Button(btn_row1, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_joint_inputs).pack(side="left", padx=10)
        tk.Button(btn_row1, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_joint_to_joint).pack(side="left", padx=10)
        tk.Button(btn_row1, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)


        # ========== Cartesian Space =========
        cartesian_frame = tk.Frame(inner_frame, bg="white")
        cartesian_frame.pack(fill="x", padx=5, pady=(5, 5))
        cartesian_title = tk.Label(cartesian_frame, text="Cartesian Space", bg="#2196F3",
                                   fg="white", font=("Arial", 10, "bold"))
        cartesian_title.pack(fill='x', padx=(5, 20))


        # JOINTS TO JOINTS (linear motion)
        func2_frame = ttk.LabelFrame(cartesian_frame, text="Joints to joints (linear)", padding=10,
                                     relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        func2_frame.pack(fill="x", padx=10, pady=(10, 5))

        arm0_row2 = tk.Frame(func2_frame, bg="white")
        arm0_row2.pack(fill="x", pady=2)
        tk.Label(arm0_row2, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_row2, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_start_arm0_entry = tk.Entry(arm0_row2, width=50)
        self.linear_start_arm0_entry.pack(side="left", padx=2)
        self.linear_start_arm0_entry.insert(0, "17.470, -43.308, 11.804, -79.761, -10.700, -2.874, 9.134")
        tk.Button(arm0_row2, text="GetCur", command=lambda: self.pln_get_cur_joints_linear('Arm0')).pack(side="left",
                                                                                                         padx=2)
        tk.Label(arm0_row2, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_end_arm0_entry = tk.Entry(arm0_row2, width=50)
        self.linear_end_arm0_entry.pack(side="left", padx=2)
        self.linear_end_arm0_entry.insert(0, "19.597, -32.480, 10.050, -58.939, -8.863, -33.821, 4.772")

        arm1_row2 = tk.Frame(func2_frame, bg="white")
        arm1_row2.pack(fill="x", pady=2)
        tk.Label(arm1_row2, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_row2, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_start_arm1_entry = tk.Entry(arm1_row2, width=50)
        self.linear_start_arm1_entry.pack(side="left", padx=2)
        self.linear_start_arm1_entry.insert(0, "-17.470, -43.308, -11.804, -79.761, 10.700, -2.874, -9.134")
        tk.Button(arm1_row2, text="GetCur", command=lambda: self.pln_get_cur_joints_linear('Arm1')).pack(side="left",
                                                                                                         padx=2)
        tk.Label(arm1_row2, text="End joints", bg="white", width=10).pack(side="left", padx=2)
        self.linear_end_arm1_entry = tk.Entry(arm1_row2, width=50)
        self.linear_end_arm1_entry.pack(side="left", padx=2)
        self.linear_end_arm1_entry.insert(0, "-19.597,-32.480,-10.050,-58.939,8.863,-33.821,-4.772")

        # Freq, Vel, Acc）
        params_row2 = tk.Frame(func2_frame, bg="white")
        params_row2.pack(fill="x", pady=5)
        tk.Label(params_row2, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                     padx=10)
        tk.Label(params_row2, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_freq_entry = tk.Entry(params_row2, width=6)
        self.linear_freq_entry.pack(side="left", padx=2)
        self.linear_freq_entry.insert(0, "50")
        tk.Label(params_row2, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                      padx=(0, 5))

        tk.Label(params_row2, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_vel_entry = tk.Entry(params_row2, width=6)
        self.linear_vel_entry.pack(side="left", padx=2)
        self.linear_vel_entry.insert(0, "100")
        tk.Label(params_row2, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=(0, 5))

        tk.Label(params_row2, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.linear_acc_entry = tk.Entry(params_row2, width=6)
        self.linear_acc_entry.pack(side="left", padx=2)
        self.linear_acc_entry.insert(0, "100")
        tk.Label(params_row2, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        btn_row2 = tk.Frame(func2_frame, bg="white")
        btn_row2.pack(pady=5, anchor="center")
        tk.Button(btn_row2, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_linear_inputs).pack(side="left", padx=10)
        tk.Button(btn_row2, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_joint_to_joints_linear).pack(side="left", padx=10)
        tk.Button(btn_row2, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)


        # ===Linear
        linear_frame = ttk.LabelFrame(cartesian_frame, text="Linear", padding=10,
                                      relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        linear_frame.pack(fill="x", padx=5, pady=(5, 5))
        # ARM0
        arm0_cart_row = tk.Frame(linear_frame, bg="white")
        arm0_cart_row.pack(fill="x", pady=2)
        tk.Label(arm0_cart_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_cart_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_start_arm0_entry = tk.Entry(arm0_cart_row, width=50)
        self.cart_start_arm0_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:
            self.cart_start_arm0_entry.insert(0, "447.829, 203.577, 336.036, -169.144, 55.011, -146.752")
        else:
            self.cart_start_arm0_entry.insert(0, "509.734, 233.609, 365.948, -169.144, 55.011, -146.752")
        tk.Button(arm0_cart_row, text="GetCur",
                  command=lambda: self.pln_get_cur_xyzabc('Arm0')).pack(side="left", padx=2)
        tk.Label(arm0_cart_row, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_end_arm0_entry = tk.Entry(arm0_cart_row, width=50)
        self.cart_end_arm0_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:
            self.cart_end_arm0_entry.insert(0, "447.829, 203.577, 236.036, -169.144, 55.011, -146.752")
        else:
            self.cart_end_arm0_entry.insert(0, "509.734, 233.609, 265.948, -169.144, 55.011, -146.752")

        arm0_cart_row1 = tk.Frame(linear_frame, bg="white")
        arm0_cart_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm0_cart_row1, text="Ref joints", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.linear_ref_arm0_entry = tk.Entry(arm0_cart_row1, width=50)
        self.linear_ref_arm0_entry.pack(side="left", padx=2)
        self.linear_ref_arm0_entry.insert(0, "19.597, -32.480, 10.050, -58.939, -8.863, -33.821, 4.772")
        tk.Button(arm0_cart_row1, text="GetCur", command=lambda: self.pln_get_cur_joints_as_linear_ref('Arm0')).pack(
            side="left", padx=2)

        # ARM1
        arm1_cart_row = tk.Frame(linear_frame, bg="white")
        arm1_cart_row.pack(fill="x", pady=2)
        tk.Label(arm1_cart_row, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_cart_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_start_arm1_entry = tk.Entry(arm1_cart_row, width=50)
        self.cart_start_arm1_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:
            self.cart_start_arm1_entry.insert(0, "447.829, -203.577, 336.036, 169.144, 55.011, 146.752")
        else:
            self.cart_start_arm1_entry.insert(0, "509.734, -233.609, 365.948, 169.144, 55.011, 146.752")
        tk.Button(arm1_cart_row, text="GetCur",
                  command=lambda: self.pln_get_cur_xyzabc('Arm1')).pack(side="left", padx=2)
        tk.Label(arm1_cart_row, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.cart_end_arm1_entry = tk.Entry(arm1_cart_row, width=50)
        self.cart_end_arm1_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:
            self.cart_end_arm1_entry.insert(0, "447.829, -203.577, 236.036, 169.144, 55.011, 146.752")
        else:
            self.cart_end_arm1_entry.insert(0, "509.734, -233.609, 265.948, 169.144, 55.011, 146.752")

        arm1_cart_row1 = tk.Frame(linear_frame, bg="white")
        arm1_cart_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm1_cart_row1, text="Ref joints", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.linear_ref_arm1_entry = tk.Entry(arm1_cart_row1, width=50)
        self.linear_ref_arm1_entry.pack(side="left", padx=2)
        self.linear_ref_arm1_entry.insert(0, "-19.597, -32.480, -10.050, -58.939, 8.863, -33.821, -4.772")
        tk.Button(arm1_cart_row1, text="GetCur", command=lambda: self.pln_get_cur_joints_as_linear_ref('Arm1')).pack(
            side="left", padx=2)

        # freq vel acc
        cart_params_row = tk.Frame(linear_frame, bg="white")
        cart_params_row.pack(fill="x", pady=5)
        tk.Label(cart_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                         padx=10)
        tk.Label(cart_params_row, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_freq_entry = tk.Entry(cart_params_row, width=6)
        self.cart_freq_entry.pack(side="left", padx=2)
        self.cart_freq_entry.insert(0, "50")
        tk.Label(cart_params_row, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                          padx=(0, 5))

        tk.Label(cart_params_row, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_vel_entry = tk.Entry(cart_params_row, width=6)
        self.cart_vel_entry.pack(side="left", padx=2)
        self.cart_vel_entry.insert(0, "100")
        tk.Label(cart_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                  padx=(0, 5))

        tk.Label(cart_params_row, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.cart_acc_entry = tk.Entry(cart_params_row, width=6)
        self.cart_acc_entry.pack(side="left", padx=2)
        self.cart_acc_entry.insert(0, "100")
        tk.Label(cart_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        cart_btn_row = tk.Frame(linear_frame, bg="white")
        cart_btn_row.pack(pady=5, anchor="center")
        tk.Button(cart_btn_row, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_linear_cart_inputs).pack(side="left", padx=10)
        tk.Button(cart_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_cartesian_linear).pack(side="left", padx=10)
        tk.Button(cart_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

        # Linear (multi-segment)
        multi_seg_frame = ttk.LabelFrame(cartesian_frame, text="Linear (multi-segment)", padding=10,
                                         relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        multi_seg_frame.pack(fill="x", padx=5, pady=(10, 5))

        # ---------- Arm0 ----------
        arm0_multi_row = tk.Frame(multi_seg_frame, bg="white")
        arm0_multi_row.pack(fill="x", pady=2)
        tk.Label(arm0_multi_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_multi_row, text="start joints", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.multi_start_joints_arm0_entry = tk.Entry(arm0_multi_row, width=50)
        self.multi_start_joints_arm0_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:
            self.multi_start_joints_arm0_entry.insert(0, "17.832, -35.817, 11.527, -75.747, -9.230, -14.070, 7.530")
        else:
            self.multi_start_joints_arm0_entry.insert(0, "17.970, -35.197, 11.414, -73.344, -9.154, -17.035, 7.086")
        tk.Button(arm0_multi_row, text="GetCur",
                  command=lambda: self.pln_get_cur_joints_as_ref('Arm0')).pack(side="left",
                                                                                                          padx=2)

        tk.Label(arm0_multi_row, text="Add XYZABC", bg="white", width=10).pack(side="left", padx=(10, 0))
        self.multi_add_xyzabc_arm0_entry = tk.Entry(arm0_multi_row, width=50)
        self.multi_add_xyzabc_arm0_entry.pack(side="left", padx=2)
        self.multi_add_xyzabc_arm0_entry.insert(0, "0,0,0,0,0,0")
        tk.Button(arm0_multi_row, text="Add",
                  command=lambda: self.add_multi_seg_point('Arm0')).pack(side="left", padx=2)

        arm0_multi_row1 = tk.Frame(multi_seg_frame, bg="white")
        arm0_multi_row1.pack(fill="x", pady=(2, 10))
        tk.Label(arm0_multi_row1, text="All points", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.multi_points_arm0_combo = ttk.Combobox(arm0_multi_row1, width=50, state="readonly")
        self.multi_points_arm0_combo.pack(side="left", padx=2)
        tk.Button(arm0_multi_row1, text="Delete",
                  command=lambda: self.del_multi_seg_point('Arm0')).pack(side="left", padx=2)

        if robot.get_robot_type() == robot_type_map[3]:  
            default_points0=[
            "447.833, 203.571, 236.037, -169.143, 55.012, -146.752", 
            "447.833, 203.571, 336.037, -169.143, 55.012, -146.752", 
            "447.833, 303.571, 336.037, -169.143, 55.012, -146.752", 
            "447.833, 303.571, 236.037, -169.143, 55.012, -146.752"
            ]
        else:
            default_points0 = [
                "509.731, 233.614, 265.949, -169.144, 55.011, -146.752",
                "509.731, 233.614, 65.949, -169.144, 55.011, -146.752",
                "509.731, 33.614, 65.949, -169.144, 55.011, -146.752",
                "509.731, 33.614, 265.949, -169.144, 55.011, -146.752"
            ]

        self.multi_points_arm0_list = default_points0.copy()
        self.multi_points_arm0_combo['values'] = tuple(self.multi_points_arm0_list)
        if self.multi_points_arm0_list:
            self.multi_points_arm0_combo.current(0)

            # ---------- Arm1 ----------
        arm1_multi_row = tk.Frame(multi_seg_frame, bg="white")
        arm1_multi_row.pack(fill="x", pady=2)
        tk.Label(arm1_multi_row, text="Arm1", bg="#F4E4D8", width=5).pack(side="left", padx=2)
        tk.Label(arm1_multi_row, text="start joints", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.multi_start_joints_arm1_entry = tk.Entry(arm1_multi_row, width=50)
        self.multi_start_joints_arm1_entry.pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:
            self.multi_start_joints_arm1_entry.insert(0, "-17.832, -35.817, -11.527, -75.747, 9.230, -14.070, -7.530")
        else:
            self.multi_start_joints_arm1_entry.insert(0, "-17.970, -35.197, -11.414, -73.344, 9.154, -17.035, -7.086")
        tk.Button(arm1_multi_row, text="GetCur",
                  command=lambda: self.pln_get_cur_joints_as_ref('Arm1')).pack(side="left",
                                                                                                          padx=2)

        tk.Label(arm1_multi_row, text="Add XYZABC", bg="white", width=10).pack(side="left", padx=(10, 0))
        self.multi_add_xyzabc_arm1_entry = tk.Entry(arm1_multi_row, width=50)
        self.multi_add_xyzabc_arm1_entry.pack(side="left", padx=2)
        self.multi_add_xyzabc_arm1_entry.insert(0, "0,0,0,0,0,0")
        tk.Button(arm1_multi_row, text="Add",
                  command=lambda: self.add_multi_seg_point('Arm1')).pack(side="left", padx=2)
        arm1_multi_row1 = tk.Frame(multi_seg_frame, bg="white")
        arm1_multi_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm1_multi_row1, text="All points", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.multi_points_arm1_combo = ttk.Combobox(arm1_multi_row1, width=50, state="readonly")
        self.multi_points_arm1_combo.pack(side="left", padx=2)
        tk.Button(arm1_multi_row1, text="Delete",
                  command=lambda: self.del_multi_seg_point('Arm1')).pack(side="left", padx=2)
        if robot.get_robot_type() == robot_type_map[3]:   
            default_points1=[
            "447.833, -203.571, 236.037, 169.143, 55.012, 146.752", 
            "447.833, -203.571, 336.03700000000003, 169.143, 55.012, 146.752", 
            "447.833, -103.571, 336.03700000000003, 169.143, 55.012, 146.752", 
            "447.833, -103.571, 236.03700000000003, 169.143, 55.012, 146.752"
            ]
        else:
            default_points1 = [
                "509.731, -233.614, 265.949, 169.144, 55.011, 146.752",
                "509.731, -233.614, 65.949, 169.144, 55.011, 146.752",
                "509.731, -33.614, 65.949, 169.144, 55.011, 146.752",
                "509.731, -33.614, 265.949, 169.144, 55.011, 146.752"
            ]
        self.multi_points_arm1_list = default_points1.copy()
        self.multi_points_arm1_combo['values'] = tuple(self.multi_points_arm1_list)
        if self.multi_points_arm1_list:
            self.multi_points_arm1_combo.current(0)

        multi_params_row = tk.Frame(multi_seg_frame, bg="white")
        multi_params_row.pack(fill="x", pady=5)
        tk.Label(multi_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                          padx=10)
        tk.Label(multi_params_row, text="Freq:", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_cart_freq_entry = tk.Entry(multi_params_row, width=6)
        self.multi_cart_freq_entry.pack(side="left", padx=2)
        self.multi_cart_freq_entry.insert(0, "50")
        tk.Label(multi_params_row, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                           padx=(0, 5))
        tk.Label(multi_params_row, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_cart_vel_entry = tk.Entry(multi_params_row, width=6)
        self.multi_cart_vel_entry.pack(side="left", padx=2)
        self.multi_cart_vel_entry.insert(0, "100")
        tk.Label(multi_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                   padx=(0, 5))

        tk.Label(multi_params_row, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_cart_acc_entry = tk.Entry(multi_params_row, width=6)
        self.multi_cart_acc_entry.pack(side="left", padx=2)
        self.multi_cart_acc_entry.insert(0, "100")
        tk.Label(multi_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)
        tk.Label(multi_params_row, text="Allow Range", bg="white", font=("Arial", 9)).pack(side="left",  padx=(5, 2))
        self.multi_allow_range_entry = tk.Entry(multi_params_row, width=3)
        self.multi_allow_range_entry.pack(side="left", padx=2)
        self.multi_allow_range_entry.insert(0, "5")
        tk.Label(multi_params_row, text="ZSP Type", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_zsp_type_entry = tk.Entry(multi_params_row, width=3)
        self.multi_zsp_type_entry.pack(side="left", padx=2)
        self.multi_zsp_type_entry.insert(0, "1")
        tk.Label(multi_params_row, text="ZSP Params", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.multi_zsp_params_entry = tk.Entry(multi_params_row, width=20)
        self.multi_zsp_params_entry.pack(side="left", padx=2)
        self.multi_zsp_params_entry.insert(0, "0, 0, -1, 0, 0, 0")

        multi_btn_row = tk.Frame(multi_seg_frame, bg="white")
        multi_btn_row.pack(pady=5, anchor="center")
        tk.Button(multi_btn_row, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_multi_segment_inputs).pack(side="left", padx=10)
        tk.Button(multi_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_multi_segment_linear).pack(side="left", padx=10)
        tk.Button(multi_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

        # ====Co-arms
        syn_frame = ttk.LabelFrame(cartesian_frame, text="Arms Synchronous Linear", padding=10,
                                relief=tk.GROOVE, borderwidth=2, style="MyCustom.TLabelframe")
        syn_frame.pack(fill="x", padx=5, pady=(5, 5))
        # ARM0
        arm0_syn_row = tk.Frame(syn_frame, bg="white")
        arm0_syn_row.pack(fill="x", pady=2)
        tk.Label(arm0_syn_row, text="Arm0", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm0_syn_row, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.syn_joints_arm0_entry = tk.Entry(arm0_syn_row, width=50)
        self.syn_joints_arm0_entry.pack(side="left", padx=2)
        self.syn_joints_arm0_entry.insert(0, "0.876, -25.548, -0.000, -87.472, -18.026, -7.201, -18.925")
        tk.Button(arm0_syn_row, text="GetCur", command=lambda: self.pln_syn_get_cur_joints('Arm0')).pack(side="left", padx=2)
        tk.Label(arm0_syn_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.syn_start_arm0_entry = tk.Entry(arm0_syn_row, width=50)
        self.syn_start_arm0_entry.pack(side="left", padx=2)
        self.syn_start_arm0_entry.insert(0, "509.733, 33.610, 265.953, -169.144, 55.012, -146.752")
        tk.Button(arm0_syn_row, text="GetCur",
                  command=lambda: self.pln_syn_get_cur_xyzabc('Arm0')).pack(side="left", padx=2)

        arm0_syn_row1 = tk.Frame(syn_frame, bg="white")
        arm0_syn_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm0_syn_row1, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.syn_end_arm0_entry = tk.Entry(arm0_syn_row1, width=50)
        self.syn_end_arm0_entry.pack(side="left", padx=(2,50))
        self.syn_end_arm0_entry.insert(0, "509.733, 233.610, 265.953, -169.144, 55.012, -146.752")
        tk.Label(arm0_syn_row1, text="ZSP Type", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn0_zsp_type_entry = tk.Entry(arm0_syn_row1, width=3)
        self.syn0_zsp_type_entry.pack(side="left", padx=2)
        self.syn0_zsp_type_entry.insert(0, "1")
        tk.Label(arm0_syn_row1, text="ZSP Params", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn0_zsp_params_entry = tk.Entry(arm0_syn_row1, width=20)
        self.syn0_zsp_params_entry.pack(side="left", padx=2)
        self.syn0_zsp_params_entry.insert(0, "0, 0, -1, 0, 0, 0")

        # ARM1
        arm1_syn_row = tk.Frame(syn_frame, bg="white")
        arm1_syn_row.pack(fill="x", pady=2)
        tk.Label(arm1_syn_row, text="Arm1", bg="#D8F4F3", width=5).pack(side="left", padx=2)
        tk.Label(arm1_syn_row, text="Start joints", bg="white", width=10).pack(side="left", padx=2)
        self.syn_joints_arm1_entry = tk.Entry(arm1_syn_row, width=50)
        self.syn_joints_arm1_entry.pack(side="left", padx=2)
        self.syn_joints_arm1_entry.insert(0, "-0.876, -25.548, 0.000, -87.472, 18.026, -7.201, 18.925")
        tk.Button(arm1_syn_row, text="GetCur", command=lambda: self.pln_syn_get_cur_joints('Arm1')).pack(side="left",
                                                                                                         padx=2)
        tk.Label(arm1_syn_row, text="Start XYZABC", bg="white", width=10).pack(side="left", padx=(5, 0))
        self.syn_start_arm1_entry = tk.Entry(arm1_syn_row, width=50)
        self.syn_start_arm1_entry.pack(side="left", padx=2)
        self.syn_start_arm1_entry.insert(0, "509.733, -33.610, 265.953, 169.144, 55.012, 146.752")
        tk.Button(arm1_syn_row, text="GetCur",
                  command=lambda: self.pln_syn_get_cur_xyzabc('Arm1')).pack(side="left", padx=2)

        arm1_syn_row1 = tk.Frame(syn_frame, bg="white")
        arm1_syn_row1.pack(fill="x", pady=(2,10))
        tk.Label(arm1_syn_row1, text="End XYZABC", bg="white", width=10).pack(side="left", padx=(50, 0))
        self.syn_end_arm1_entry = tk.Entry(arm1_syn_row1, width=50)
        self.syn_end_arm1_entry.pack(side="left", padx=(2,50))
        self.syn_end_arm1_entry.insert(0, "509.733, -233.610, 265.953, 169.144, 55.012, 146.752")

        tk.Label(arm1_syn_row1, text="ZSP Type", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn1_zsp_type_entry = tk.Entry(arm1_syn_row1, width=3)
        self.syn1_zsp_type_entry.pack(side="left", padx=2)
        self.syn1_zsp_type_entry.insert(0, "1")
        tk.Label(arm1_syn_row1, text="ZSP Params", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn1_zsp_params_entry = tk.Entry(arm1_syn_row1, width=20)
        self.syn1_zsp_params_entry.pack(side="left", padx=2)
        self.syn1_zsp_params_entry.insert(0, "0, 0, -1, 0, 0, 0")

        # freq vel acc
        syn_params_row = tk.Frame(syn_frame, bg="white")
        syn_params_row.pack(fill="x", pady=5)
        tk.Label(syn_params_row, text="Common Parameters:", bg="white", font=("Arial", 9, "bold")).pack(side="left",
                                                                                                         padx=10)
        tk.Label(syn_params_row, text="Freq", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn_freq_entry = tk.Entry(syn_params_row, width=6)
        self.syn_freq_entry.pack(side="left", padx=2)
        self.syn_freq_entry.insert(0, "50")
        tk.Label(cart_params_row, text="((1000%freq==0))", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                          padx=(0, 5))
        tk.Label(syn_params_row, text="Vel", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn_vel_entry = tk.Entry(syn_params_row, width=6)
        self.syn_vel_entry.pack(side="left", padx=2)
        self.syn_vel_entry.insert(0, "100")
        tk.Label(syn_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left",
                                                                                                  padx=(0, 5))
        tk.Label(syn_params_row, text="Acc", bg="white", font=("Arial", 9)).pack(side="left", padx=(5, 2))
        self.syn_acc_entry = tk.Entry(syn_params_row, width=6)
        self.syn_acc_entry.pack(side="left", padx=2)
        self.syn_acc_entry.insert(0, "100")
        tk.Label(syn_params_row, text="(1-1000)", bg="white", fg="gray", font=("Arial", 7)).pack(side="left", padx=2)

        syn_btn_row = tk.Frame(syn_frame, bg="white")
        syn_btn_row.pack(pady=5, anchor="center")
        tk.Button(syn_btn_row, text="Clear params", width=10, font=("Arial", 11, "bold"), bg="#E6E6FA",
                  command=self.clear_syn_inputs).pack(side="left", padx=10)
        tk.Button(syn_btn_row, text="Run", width=10, font=("Arial", 11, "bold"), bg="#A2CD5A",
                  command=self.pln_run_syn).pack(side="left", padx=10)
        tk.Button(syn_btn_row, text="Break", width=10, font=("Arial", 11, "bold"), bg="#FFF68F",
                  command=self.stop_motion).pack(side="left", padx=10)

    def pln_get_cur_joints(self, obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.joints_start_arm0_entry.delete(0, tk.END)
                    self.joints_start_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
                    return
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.joints_start_arm1_entry.delete(0, tk.END)
                    self.joints_start_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_get_cur_joints_as_ref(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.multi_start_joints_arm0_entry.delete(0, tk.END)
                    self.multi_start_joints_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.multi_start_joints_arm1_entry.delete(0, tk.END)
                    self.multi_start_joints_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_get_cur_joints_as_linear_ref(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_ref_arm0_entry.delete(0, tk.END)
                    self.linear_ref_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_ref_arm1_entry.delete(0, tk.END)
                    self.linear_ref_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_get_cur_joints_linear(self, obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_start_arm0_entry.delete(0, tk.END)
                    self.linear_start_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.linear_start_arm1_entry.delete(0, tk.END)
                    self.linear_start_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def pln_syn_get_cur_joints(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.syn_joints_arm0_entry.delete(0, tk.END)
                    self.syn_joints_arm0_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm0')
            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                if pose and len(pose) == 7:
                    pose_text = ", ".join(f"{v:.3f}" for v in pose)
                    self.syn_joints_arm1_entry.delete(0, tk.END)
                    self.syn_joints_arm1_entry.insert(0, pose_text)
                else:
                    messagebox.showerror('Error', 'Invalid joint data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get joint positions: {e}')

    def clear_joint_inputs(self):
        for entry in [self.joints_start_arm0_entry, self.joints_end_arm0_entry,
                      self.joints_start_arm1_entry, self.joints_end_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                    entry.delete(0, tk.END)
                    entry.insert(0, "0.0")

    def pln_run_joint_to_joint(self):
        start_str0 = self.joints_start_arm0_entry.get().strip()
        end_str0 = self.joints_end_arm0_entry.get().strip()
        start_str1 = self.joints_start_arm1_entry.get().strip()
        end_str1 = self.joints_end_arm1_entry.get().strip()

        def parse_joints(s):
            parts = s.split(',')
            if len(parts) != 7:
                raise ValueError(f"need 7values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]

        try:
            start0 = parse_joints(start_str0)
            end0 = parse_joints(end_str0)
            start1 = parse_joints(start_str1)
            end1 = parse_joints(end_str1)
        except ValueError as e:
            messagebox.showerror("joints error", f"parse joints failed: {e}")
            return
        try:
            vel = float(self.joints_vel_entry.get().strip())
            acc = float(self.joints_acc_entry.get().strip())
            freq = int(self.joints_freq_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if not (0.01 <= vel <= 1):
            messagebox.showerror("value error", "vel range [0.01,1]")
            return
        if not (0.01 <= acc <= 1):
            messagebox.showerror("value error", "acc range [0.01,1]")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        is_zero0 = all(abs(v) < 1e-6 for v in start0) and all(abs(v) < 1e-6 for v in end0)
        is_zero1 = all(abs(v) < 1e-6 for v in start1) and all(abs(v) < 1e-6 for v in end1)

        if is_zero0 and is_zero1:
            messagebox.showerror("value error", "start and end in same pose, can not run planning")
            return

        if not is_zero0 and is_zero1:
            ret = robot.plan_joints(0, start0, end0, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array, point_num)
                if ret1!= 0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed. Error msg:  {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed. Error msg:  {robot._get_operate_error_msg(ret)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if not is_zero1 and is_zero0:
            ret = robot.plan_joints(1, start1, end1, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1= robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array, point_num)
                if ret1 != 0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed. Error msg: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return

            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if not is_zero0 and not is_zero1:
            points0 = robot.plan_joints(0, start0, end0, vel, acc, freq)
            if isinstance(points0, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points0[0],points0[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed. Error msg: {robot._get_operate_error_msg(points0)}")
                return

            points1 = robot.plan_joints(1, start1, end1, vel, acc, freq)
            if isinstance(points1, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points1[0],points1[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm1 send Planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed. Error msg: {robot._get_operate_error_msg(points1)}")
                return

            mask=FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask!=mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def clear_linear_inputs(self):
        for entry in [self.linear_start_arm0_entry, self.linear_end_arm0_entry,
                      self.linear_start_arm1_entry, self.linear_end_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

    def pln_run_joint_to_joints_linear(self):
        start_str0 = self.linear_start_arm0_entry.get().strip()
        end_str0 = self.linear_end_arm0_entry.get().strip()
        start_str1 = self.linear_start_arm1_entry.get().strip()
        end_str1 = self.linear_end_arm1_entry.get().strip()

        def parse_joints(s):
            parts = s.split(',')
            if len(parts) != 7:
                raise ValueError(f"need 7 values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]

        try:
            start0 = parse_joints(start_str0)
            end0 = parse_joints(end_str0)

            start1 = parse_joints(start_str1)
            end1 = parse_joints(end_str1)
        except ValueError as e:
            messagebox.showerror("joints error", f"parse joints failed: {e}")
            return
        try:
            vel = float(self.linear_vel_entry.get().strip())
            acc = float(self.linear_acc_entry.get().strip())
            freq = int(self.linear_freq_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if not (1 <= vel <= 1000):
            messagebox.showerror("value error", "vel range [1,1000]")
            return
        if not (1 <= acc <= 1000):
            messagebox.showerror("value error", "acc range [1,1000]")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        is_zero0 = all(abs(v) < 1e-6 for v in start0) and all(abs(v) < 1e-6 for v in end0)
        is_zero1 = all(abs(v) < 1e-6 for v in start1) and all(abs(v) < 1e-6 for v in end1)

        if is_zero0 and is_zero1:
            messagebox.showerror("value error", "start and end in same pose, can not run planning")
            return

        if not is_zero0 and is_zero1:
            ret = robot.plan_linear_keep_joints(0, start0, end0, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array, point_num)
                if ret1!= 0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed. Error msg: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return

            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if not is_zero1 and is_zero0:
            ret = robot.plan_linear_keep_joints(1, start1, end1, vel, acc, freq)
            if isinstance(ret, tuple):
                raw_array, point_num = ret
                ret1=robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array, point_num)
                if ret1!= 0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed. Error msg: {robot._get_operate_error_msg(ret1)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 Planning failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return


            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if not is_zero0 and not is_zero1:
            points0 = robot.plan_linear_keep_joints(0, start0, end0, vel, acc, freq)
            if isinstance(points0, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points0[0], points0[1])
                if ret!= 0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm0 planning failed. Error msg: {robot._get_operate_error_msg(points0)}")
                return

            points1 = robot.plan_linear_keep_joints(1, start1, end1, vel, acc, freq)
            if isinstance(points1, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points1[0], points1[1])
                if ret!= 0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error", f"Arm1 planning failed. Error msg: {robot._get_operate_error_msg(points1)}")
                return

            mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def pln_get_cur_xyzabc(self, obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                arm0_joints = robot.forward_kinematics(0, pose)
                arm0_xyzabc = robot.matrix2xyzabc(arm0_joints)

                if arm0_xyzabc and len(arm0_xyzabc) == 6:
                    arm0_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm0_xyzabc)
                    self.cart_start_arm0_entry.delete(0, tk.END)
                    self.cart_start_arm0_entry.insert(0, arm0_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm0')

            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                arm1_joints = robot.forward_kinematics(1, pose)
                arm1_xyzabc = robot.matrix2xyzabc(arm1_joints)
                if arm1_xyzabc and len(arm1_xyzabc) == 6:
                    arm1_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm1_xyzabc)
                    self.cart_start_arm1_entry.delete(0, tk.END)
                    self.cart_start_arm1_entry.insert(0, arm1_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get xyzabc positions: {e}')

    def pln_syn_get_cur_xyzabc(self,obj):
        try:
            if obj == 'Arm0':
                pose = self.rt["arms"][0]["fb"]["fb_pos"]
                arm0_joints = robot.forward_kinematics(0, pose)
                arm0_xyzabc = robot.matrix2xyzabc(arm0_joints)

                if arm0_xyzabc and len(arm0_xyzabc) == 6:
                    arm0_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm0_xyzabc)
                    self.syn_start_arm0_entry.delete(0, tk.END)
                    self.syn_start_arm0_entry.insert(0, arm0_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm0')

            elif obj == 'Arm1':
                pose = self.rt["arms"][1]["fb"]["fb_pos"]
                arm1_joints = robot.forward_kinematics(1, pose)
                arm1_xyzabc = robot.matrix2xyzabc(arm1_joints)
                if arm1_xyzabc and len(arm1_xyzabc) == 6:
                    arm1_xyzabc_text = ", ".join(f"{v:.3f}" for v in arm1_xyzabc)
                    self.syn_start_arm1_entry.delete(0, tk.END)
                    self.syn_start_arm1_entry.insert(0, arm1_xyzabc_text)
                else:
                    messagebox.showerror('Error', 'Invalid xyzabc data for Arm1')
        except (KeyError, IndexError, TypeError) as e:
            messagebox.showerror('Error', f'Failed to get xyzabc positions: {e}')

    def clear_linear_cart_inputs(self):
        for entry in [self.cart_start_arm0_entry, self.cart_end_arm0_entry, self.linear_ref_arm0_entry,
                      self.cart_start_arm1_entry, self.cart_end_arm1_entry, self.linear_ref_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

    def pln_run_cartesian_linear(self):
        start_str0 = self.cart_start_arm0_entry.get().strip()
        end_str0 = self.cart_end_arm0_entry.get().strip()
        start_str1 = self.cart_start_arm1_entry.get().strip()
        end_str1 = self.cart_end_arm1_entry.get().strip()
        ref_joints0=self.linear_ref_arm0_entry.get().strip()
        ref_joints1 = self.linear_ref_arm1_entry.get().strip()

        def parse_joints(s,num):
            parts = s.split(',')
            if len(parts) != num:
                raise ValueError(f"need {num} values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]

        try:
            start0 = parse_joints(start_str0,6)
            end0 = parse_joints(end_str0,6)

            start1 = parse_joints(start_str1,6)
            end1 = parse_joints(end_str1,6)

            ref0=parse_joints(ref_joints0,7)
            ref1=parse_joints(ref_joints1,7)

        except ValueError as e:
            messagebox.showerror("xyzabc/ref_joints error", f"parse xyzabc/ref_joints failed: {e}")
            return
        try:
            vel = float(self.cart_vel_entry.get().strip())
            acc = float(self.cart_acc_entry.get().strip())
            freq = int(self.cart_freq_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if vel <= 0:
            messagebox.showerror("value error", "vel > 0")
            return
        if acc <= 0:
            messagebox.showerror("value error", "acc > 0")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        if all(v == 0 for v in ref0):
            messagebox.showwarning('error',"reference joints can not be all zero")
            return
        if all(v == 0 for v in ref1):
            messagebox.showwarning('error',"reference joints can not be all zero")
            return

        is_zero0 = all(abs(v) < 1e-6 for v in start0) and all(abs(v) < 1e-6 for v in end0)
        is_zero1 = all(abs(v) < 1e-6 for v in start1) and all(abs(v) < 1e-6 for v in end1)

        if is_zero0 and is_zero1:
            messagebox.showerror("value error", "start and end in same pose, can not run planning")
            return

        if not is_zero0 and is_zero1:
            points = robot.plan_linear(0, start0, end0,ref0, vel, acc, freq)
            if isinstance(points, tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points[0],points[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm0 planning failed. Error msg: {robot._get_operate_error_msg(points)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0. Return mask: {ret_mask}")
                return

        if not is_zero1 and is_zero0:
            points = robot.plan_linear(1, start1, end1,ref1, vel, acc, freq)
            if isinstance(points,tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points[0],points[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm1 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm1 planning failed. Error msg: {robot._get_operate_error_msg(points)}")
                return


            mask = FXObjMask.OBJ_ARM1_FLAG
            ret_mask=robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm1. Return mask: {ret_mask}")
                return

        if not is_zero0 and not is_zero1:
            points0 = robot.plan_linear(0, start0, end0, ref0,vel, acc, freq)
            if isinstance(points0,tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM0, points0[0],points0[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send Planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm0 planning failed. Error msg: {robot._get_operate_error_msg(points0)}")
                return

            points1 = robot.plan_linear(1, start1, end1,ref1,vel, acc, freq)
            if isinstance(points1,tuple):
                ret=robot.config_set_traj(FXObjType.OBJ_ARM1, points1[0],points1[1])
                if ret!=0:
                    messagebox.showerror("Failed!", f"Arm0 send Planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
            else:
                messagebox.showerror("Error",
                                     f"Arm1 planning failed. Error msg: {robot._get_operate_error_msg(points1)}")
                return


            mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            ret_mask = robot.runtime_run_traj(mask)
            if ret_mask != mask:
                messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
                return

    def is_duplicate_xyzabc(self, point_list, target_list):
        new_tuple = tuple(point_list)
        for existing_str in target_list:
            try:
                existing = [float(x) for x in existing_str.split(',') if x.strip()]
                if tuple(existing) == new_tuple:
                    return True
            except ValueError:
                continue
        return False

    def add_multi_seg_point(self, arm):
        if arm == 'Arm0':
            entry = self.multi_add_xyzabc_arm0_entry
            combo = self.multi_points_arm0_combo
            points_list = self.multi_points_arm0_list
        else:
            entry = self.multi_add_xyzabc_arm1_entry
            combo = self.multi_points_arm1_combo
            points_list = self.multi_points_arm1_list
        point_str = entry.get().strip()
        if not point_str:
            return
        try:
            point_nums = [float(x) for x in point_str.split(',') if x.strip()]
        except ValueError:
            return
        if self.is_duplicate_xyzabc(point_nums, points_list):
            messagebox.showwarning('error',f"Point {point_str} already exists, not added.")
            return

        point_nums = [float(x) for x in point_str.split(',') if x.strip()]
        if all(v == 0 for v in point_nums):
            messagebox.showwarning('error',"zero points is not allowed")
            return

        points_list.append(point_str)
        combo['values'] = tuple(points_list)
        combo.set(point_str)

    def del_multi_seg_point(self, arm):
        if arm == 'Arm0':
            combo = self.multi_points_arm0_combo
            points_list = self.multi_points_arm0_list
        else:
            combo = self.multi_points_arm1_combo
            points_list = self.multi_points_arm1_list
        selected = combo.get()
        if not selected:
            messagebox.showwarning('error', "please select a point to delete")
            return
        if selected in points_list:
            points_list.remove(selected)
        combo['values'] = tuple(points_list)
        if points_list:
            combo.current(0)
        else:
            combo.set('')

    def clear_multi_segment_inputs(self):
        for entry in [self.multi_start_joints_arm0_entry, self.multi_start_joints_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

        for entry in [self.multi_add_xyzabc_arm0_entry, self.multi_add_xyzabc_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

        self.multi_points_arm0_list = []
        self.multi_points_arm0_combo['values'] = []
        if hasattr(self, 'multi_points_arm0_combo'):
            self.multi_points_arm0_combo.set('')

        self.multi_points_arm1_list = []
        self.multi_points_arm1_combo['values'] = []
        if hasattr(self, 'multi_points_arm1_combo'):
            self.multi_points_arm1_combo.set('')

    def pln_run_multi_segment_linear(self):
        try:
            start_joints_arm0 = [float(x) for x in self.multi_start_joints_arm0_entry.get().strip().split(',') if
                                 x.strip()]
            start_joints_arm1 = [float(x) for x in self.multi_start_joints_arm1_entry.get().strip().split(',') if
                                 x.strip()]

            points_arm0 = [[float(x) for x in ps.split(',') if x.strip()] for ps in self.multi_points_arm0_list if
                           ps.strip()]
            points_arm1 = [[float(x) for x in ps.split(',') if x.strip()] for ps in self.multi_points_arm1_list if
                           ps.strip()]
            vel = float(self.multi_cart_vel_entry.get().strip())
            acc = float(self.multi_cart_acc_entry.get().strip())
            freq = int(self.multi_cart_freq_entry.get().strip())
            allow_range = float(self.multi_allow_range_entry.get().strip())
            zsp_type = int(self.multi_zsp_type_entry.get().strip())
            zsp_params = [float(x) for x in self.multi_zsp_params_entry.get().strip().split(',') if x.strip()]
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if vel <= 0:
            messagebox.showerror("value error", "vel > 0")
            return
        if acc <= 0:
            messagebox.showerror("value error", "acc > 0")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return

        # An arm is skipped if its points are empty or all zero (same logic as Linear)
        def is_arm_blank(points):
            if not points:
                return True
            return all(all(abs(v) < 1e-6 for v in p) for p in points)

        blank0 = is_arm_blank(points_arm0)
        blank1 = is_arm_blank(points_arm1)

        if blank0 and blank1:
            messagebox.showerror("Error", "all parameters are zero")
            return

        if not blank0:
            if not self._plan_multi_seg_arm(0, start_joints_arm0, points_arm0, allow_range, zsp_type, zsp_params,
                                            vel, acc, freq):
                return

        if not blank1:
            if not self._plan_multi_seg_arm(1, start_joints_arm1, points_arm1, allow_range, zsp_type, zsp_params,
                                            vel, acc, freq):
                return

        if not blank0 and not blank1:
            mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
            err_msg = "Run planning trajectory failed for arm0 & arm1"
        elif not blank0:
            mask = FXObjMask.OBJ_ARM0_FLAG
            err_msg = "run planning trajectory failed for arm0"
        else:
            mask = FXObjMask.OBJ_ARM1_FLAG
            err_msg = "run planning trajectory failed for arm1"
        ret_mask = robot.runtime_run_traj(mask)
        if ret_mask != mask:
            messagebox.showerror('Failed!', f"{err_msg}. Return mask: {ret_mask}")
            return

    def _plan_multi_seg_arm(self, arm_idx, start_joints, points, allow_range, zsp_type, zsp_params, vel, acc, freq):
        """Plan and send the multi-segment linear trajectory for one arm.
        Returns True on success, False (with a messagebox) on failure."""
        arm_name = f"Arm{arm_idx}"
        obj_type = FXObjType.OBJ_ARM0 if arm_idx == 0 else FXObjType.OBJ_ARM1

        if len(points) < 2:
            messagebox.showerror("Error", f"{arm_name} needs at least 2 points")
            return False
        if all(v == 0 for v in start_joints):
            messagebox.showwarning('value error', f"{arm_name} reference joints can not be all zero")
            return False

        ret = robot.plan_linear_multi_points_set_start(arm_idx, start_joints, points[0], points[1], allow_range,
                                                       zsp_type, zsp_params, vel, acc, freq)
        if ret != 0:
            messagebox.showerror("Error", f"{arm_name} planning failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return False

        for next_one in points[2:]:
            ret = robot.plan_linear_multi_points_set_next(arm_idx, next_one, allow_range, zsp_type, zsp_params, vel, acc)
            if ret != 0:
                messagebox.showerror("Error", f"{arm_name} planning failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return False

        pts = robot.plan_linear_multi_points_get_points()
        if isinstance(pts, tuple):
            ret = robot.config_set_traj(obj_type, pts[0], pts[1])
            if ret != 0:
                messagebox.showerror("Failed!", f"{arm_name} send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return False
        else:
            messagebox.showerror("Error", f"{arm_name} planning failed. Error msg: {robot._get_operate_error_msg(pts)}")
            return False
        return True

    def clear_syn_inputs(self):
        for entry in [self.syn_joints_arm0_entry, self.syn_joints_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                entry.delete(0, tk.END)
                entry.insert(0, "0.0")

        for entry in [self.syn_start_arm0_entry, self.syn_start_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                    entry.delete(0, tk.END)
                    entry.insert(0, "0.0")

        for entry in [self.syn_end_arm0_entry, self.syn_end_arm1_entry]:
            original = entry.get()
            if ',' in original:
                parts = original.split(',')
                zero_parts = []
                for part in parts:
                    zero_parts.append("0.0")
                entry.delete(0, tk.END)
                entry.insert(0, ', '.join(zero_parts))
            else:
                    entry.delete(0, tk.END)
                    entry.insert(0, "0.0")

    def pln_run_syn(self):
        start_str0 = self.syn_start_arm0_entry.get().strip()
        start_str1 = self.syn_start_arm1_entry.get().strip()
        end_str0 = self.syn_end_arm0_entry.get().strip()
        end_str1 = self.syn_end_arm1_entry.get().strip()
        start_joints_str0 = self.syn_joints_arm0_entry.get().strip()
        start_joints_str1 = self.syn_joints_arm1_entry.get().strip()
        zsp_params_str0 = self.syn0_zsp_params_entry.get().strip()
        zsp_params_str1 = self.syn1_zsp_params_entry.get().strip()

        def parse_joints(s, num):
            parts = s.split(',')
            if len(parts) != num:
                raise ValueError(f"need {num} values, actual:{len(parts)}")
            return [float(p.strip()) for p in parts]
        try:
            start0 = parse_joints(start_str0, 6)
            end0 = parse_joints(end_str0, 6)
            start1 = parse_joints(start_str1, 6)
            end1 = parse_joints(end_str1, 6)
            start_joints0 = parse_joints(start_joints_str0, 7)
            start_joints1 = parse_joints(start_joints_str1, 7)
            zsp_params0=parse_joints(zsp_params_str0,6)
            zsp_params1 = parse_joints(zsp_params_str1, 6)
        except ValueError as e:
            messagebox.showerror("value error", f"parse paramerters failed: {e}")
            return

        try:
            vel = float(self.syn_vel_entry.get().strip())
            acc = float(self.syn_acc_entry.get().strip())
            freq = int(self.syn_freq_entry.get().strip())
            zsp_type0 = int(self.syn0_zsp_type_entry.get().strip())
            zsp_type1 = int(self.syn1_zsp_type_entry.get().strip())
        except ValueError:
            messagebox.showerror("value error", "all parameters must be number")
            return
        if vel <= 0:
            messagebox.showerror("value error", "vel > 0")
            return
        if acc <= 0:
            messagebox.showerror("value error", "acc > 0")
            return
        if freq <= 0 or 1000 % freq != 0:
            messagebox.showerror("value error", "1000%freq==0 and fraq>0")
            return
        if all(v == 0 for v in start_joints0) and all(v == 0 for v in start_joints1):
            messagebox.showwarning('value error', "Sart joints can not be all zero")
            return
        if all(v == 0 for v in start0) and all(v == 0 for v in start1):
            messagebox.showwarning('value error', "Sart XYZABC can not be all zero")
            return
        if all(v == 0 for v in end0) and all(v == 0 for v in end1):
            messagebox.showwarning('value error', "End XYZABC can not be all zero")
            return

        arms_structure_params = ArmsSynchronousPlanningParams()
        arms_structure_params.World_Co_Flag = 0
        arms_structure_params.Sync_Type = 0
        arms_structure_params.Freq = freq
        arms_structure_params.Vel = vel
        arms_structure_params.Acc = acc
        arms_structure_params.Arm0_ZSP_Type = zsp_type0
        arms_structure_params.Arm1_ZSP_Type =zsp_type1

        for i in range(7):
            arms_structure_params.Arm0_Ref_Joints[i] = start_joints0[i]
            arms_structure_params.Arm1_Ref_Joints[i] = start_joints1[i]

            if i < 6:
                arms_structure_params.Arm0_Start_XYZABC[i] = start0[i]
                arms_structure_params.Arm1_Start_XYZABC[i] = start1[i]

                arms_structure_params.Arm0_End_XYZABC[i] = end0[i]
                arms_structure_params.Arm1_End_XYZABC[i] = end1[i]

                arms_structure_params.Arm0_ZSP_Para[i] = zsp_params0[i]
                arms_structure_params.Arm1_ZSP_Para[i] = zsp_params1[i]

        ret = robot.plan_linear_synchronous(arms_structure_params)
        if not isinstance(ret, tuple):
            messagebox.showerror("Error", f"Arm0 planning failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

        raw_array0, raw_array1, point_num = ret
        ret=robot.config_set_traj(FXObjType.OBJ_ARM0, raw_array0, point_num)
        if ret!=0:
            messagebox.showerror("Failed!", f"Arm0 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        ret=robot.config_set_traj(FXObjType.OBJ_ARM1, raw_array1, point_num)
        if ret!=0:
            messagebox.showerror("Failed!", f"Arm1 send planning points failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

        mask = FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG
        ret_mask = robot.runtime_run_traj(mask)
        if ret_mask != mask:
            messagebox.showerror('Failed!', f"Run planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
            return

    def stop_motion(self):
        ret_mask = robot.runtime_stop_traj(FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG)
        if ret_mask != (FXObjMask.OBJ_ARM0_FLAG | FXObjMask.OBJ_ARM1_FLAG):
            messagebox.showerror('Failed!', f"brake planning trajectory failed for arm0 & arm1. Return mask: {ret_mask}")
            return

    def disable_soft_limit(self, obj, axis_mask: int):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            ret=robot.config_disable_soft_limit(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj} disable soft limit failed. Error msg: {robot._get_operate_error_msg(ret)} ')
        except Exception as e:
            messagebox.showerror('Error', f'Disable soft limit error: {e}')

    def clear_sensor_offset(self, obj):
        try:
            obj_type = self._obj_name_to_type(obj)
            ret=robot.config_clear_sensor_offset(obj_type)
            if ret!= 0:
                messagebox.showerror('Failed', f'{obj} clear sensor offset failed. Error msg: {robot._get_operate_error_msg(ret)} ')
        except Exception as e:
            messagebox.showerror('Error', f'Clear sensor offset error: {e}')

    def clear_motor_as_zero(self, obj, btn):
        """Motor encoder zeroing (reset encoder offset) for specified arm."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        btn.config(state="disabled")
        try:
            obj_type = self._obj_name_to_type(obj)
            axis_mask = 0x7F  # All 7 axes
            ret=robot.config_reset_enc_offset(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed!', f"{obj} reset encoder offset failed. Error msg: {robot._get_operate_error_msg(ret)} ")
                return
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def clear_motor_error(self, obj):
        """Clear encoder error for specified obj."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            if obj == 'A':
                obj_type = FXObjType.OBJ_ARM0
            elif obj == 'B':
                obj_type = FXObjType.OBJ_ARM1
            else:
                raise ValueError("obj must be 'A' or 'B'")
            axis_mask = 0x7F  # All 7 axes
            ret=robot.config_clear_enc_error(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed!', f"{obj}Clear encoder error failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', str(e))

    def step_motion_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("Step motion")
        dialog.geometry("1000x820")
        dialog.configure(bg="white")
        dialog.transient(self.root)
        dialog.resizable(True, True)
        dialog.grab_set()

        self._step_dialog = dialog
        self._step_alive = True
        self._step_joint_target = [[0.0] * 7, [0.0] * 7]
        self._step_last_ratio = [None, None]
        self._step_last_cart_ratio = [None, None]
        self._step_jog = None
        self._step_hold_after = None
        self._step_cart_target = [[0.0] * 6, [0.0] * 6]
        self._step_cart_ref = [[0.0] * 7, [0.0] * 7]
        self._step_cart_jog = None
        self._step_cart_hold_after = None
        self._step_cart_flip_deg = 30.0
        dialog.bind("<Destroy>", lambda e: self._step_stop())
        dialog.bind("<ButtonRelease-1>", lambda e: (self._step_joint_release(), self._step_cart_release()))

        # Joint space header
        joint_head = tk.Frame(dialog, bg="white")
        joint_head.pack(fill="x", padx=5, pady=(10, 0))
        tk.Label(joint_head, text="Joint Space Jogging", bg="#2196F3", fg="white",
                 font=("Arial", 10, "bold")).pack(fill="x", padx=(5, 20))

        joint_frame = ttk.LabelFrame(dialog, text="", padding=10, relief=tk.GROOVE,
                                     borderwidth=2, style="MyCustom.TLabelframe")
        joint_frame.pack(fill="x", padx=10, pady=(0, 8))

        self._step_joint_range = tk.DoubleVar(value=1.0)
        self._step_joint_vel = tk.IntVar(value=20)
        self._step_joint_acc = tk.IntVar(value=20)
        joint_sliders = [
            ("Jog Range (deg)", self._step_joint_range, 0.5, 10, 0.5),
            ("Velocity (%)", self._step_joint_vel, 1, 100, 1),
            ("Acceleration (%)", self._step_joint_acc, 1, 100, 1),
        ]
        joint_sl_row = tk.Frame(joint_frame, bg="white")
        joint_sl_row.pack(fill="x", pady=(0, 8))
        for label_text, var, lo, hi, res in joint_sliders:
            cell = tk.Frame(joint_sl_row, bg="white")
            cell.pack(side="left", padx=10)
            tk.Label(cell, text=label_text, bg="white", font=("Arial", 9)).pack(anchor="w")
            tk.Scale(cell, from_=lo, to=hi, resolution=res, orient=tk.HORIZONTAL,
                     variable=var, length=210, bg="white", troughcolor="#E3F2FD",
                     highlightthickness=0, showvalue=True).pack()

        self._step_joint_entries = [[], []]
        joint_arms_row = tk.Frame(joint_frame, bg="white")
        joint_arms_row.pack(fill="x")
        for arm_idx, arm_name, arm_bg in [(0, "ARM0", "#D8F4F3"), (1, "ARM1", "#F4E4D8")]:
            panel = tk.Frame(joint_arms_row, bg="white")
            panel.pack(side="left", expand=True, padx=14)
            tk.Label(panel, text=arm_name, bg=arm_bg, width=12,
                     font=("Arial", 10, "bold")).pack(pady=(0, 4))
            # tk.Button(panel, text="Switch to Position Mode", width=22,
            #           command=lambda a=arm_idx: self._step_joint_to_position(a), bg="#87DAE3",
            #           font=("Arial", 10, "bold")).pack(pady=(0, 6))
            box = tk.Frame(panel, bg="white")
            box.pack()
            for j in range(7):
                tk.Label(box, text=f"J{j + 1}", bg=arm_bg, width=4,
                         font=("Arial", 10, "bold")).grid(row=j, column=0, sticky="e", padx=4, pady=2)
                e = tk.Entry(box, width=10, justify="center", font=("Consolas", 10),
                             readonlybackground="#F2F8FD")
                self._step_entry_init(e, "0.000")
                e.grid(row=j, column=1, padx=6, pady=2)
                self._step_joint_entries[arm_idx].append(e)
                b_minus = tk.Button(box, text="-", width=3, bg="#F08080", font=("Arial", 9, "bold"))
                b_minus.bind("<Button-1>", lambda e, a=arm_idx, idx=j: self._step_joint_press(a, idx, -1))
                b_minus.bind("<ButtonRelease-1>", lambda e: self._step_joint_release())
                b_minus.grid(row=j, column=2, padx=4, pady=2)
                b_plus = tk.Button(box, text="+", width=3, bg="#A2CD5A", font=("Arial", 9, "bold"))
                b_plus.bind("<Button-1>", lambda e, a=arm_idx, idx=j: self._step_joint_press(a, idx, 1))
                b_plus.bind("<ButtonRelease-1>", lambda e: self._step_joint_release())
                b_plus.grid(row=j, column=3, pady=2)

        # Cartesian space header
        cart_head = tk.Frame(dialog, bg="white")
        cart_head.pack(fill="x", padx=5, pady=(8, 0))
        tk.Label(cart_head, text="Cartesian Space Jogging", bg="#2196F3", fg="white",
                 font=("Arial", 10, "bold")).pack(fill="x", padx=(5, 20))

        cart_frame = ttk.LabelFrame(dialog, text="", padding=10, relief=tk.GROOVE,
                                    borderwidth=2, style="MyCustom.TLabelframe")
        cart_frame.pack(fill="x", padx=10, pady=(0, 10))

        self._step_cart_dist = tk.IntVar(value=10)
        self._step_cart_vel = tk.IntVar(value=20)
        self._step_cart_acc = tk.IntVar(value=20)
        cart_sliders = [
            ("Jog Distance (mm)", self._step_cart_dist, 1, 100, 1),
            ("Velocity (%)", self._step_cart_vel, 1, 100, 1),
            ("Acceleration (%)", self._step_cart_acc, 1, 100, 1),
        ]
        cart_sl_row = tk.Frame(cart_frame, bg="white")
        cart_sl_row.pack(fill="x", pady=(0, 8))
        for label_text, var, lo, hi, res in cart_sliders:
            cell = tk.Frame(cart_sl_row, bg="white")
            cell.pack(side="left", padx=10)
            tk.Label(cell, text=label_text, bg="white", font=("Arial", 9)).pack(anchor="w")
            tk.Scale(cell, from_=lo, to=hi, resolution=res, orient=tk.HORIZONTAL,
                     variable=var, length=210, bg="white", troughcolor="#E3F2FD",
                     highlightthickness=0, showvalue=True).pack()

        cart_labels = [("X", "mm"), ("Y", "mm"), ("Z", "mm"),
                       ("A", "deg"), ("B", "deg"), ("C", "deg")]
        self._step_cart_entries = [[], []]
        cart_arms_row = tk.Frame(cart_frame, bg="white")
        cart_arms_row.pack(fill="x")
        for arm_idx, arm_name, arm_bg in [(0, "ARM0", "#D8F4F3"), (1, "ARM1", "#F4E4D8")]:
            panel = tk.Frame(cart_arms_row, bg="white")
            panel.pack(side="left", expand=True, padx=14)
            tk.Label(panel, text=arm_name, bg=arm_bg, width=12,
                     font=("Arial", 10, "bold")).pack(pady=(0, 4))
            # tk.Button(panel, text="Switch to Position Mode", width=22,
            #           command=lambda a=arm_idx: self._step_cart_to_position(a), bg="#87DAE3",
            #           font=("Arial", 10, "bold")).pack(pady=(0, 6))
            box = tk.Frame(panel, bg="white")
            box.pack()
            for j, (name, unit) in enumerate(cart_labels):
                tk.Label(box, text=f"{name} ({unit})", bg=arm_bg, width=8,
                         font=("Arial", 10, "bold")).grid(row=j, column=0, sticky="e", padx=4, pady=2)
                e = tk.Entry(box, width=10, justify="center", font=("Consolas", 10),
                             readonlybackground="#F2F8FD")
                self._step_entry_init(e, "0.000")
                e.grid(row=j, column=1, padx=6, pady=2)
                self._step_cart_entries[arm_idx].append(e)
                cb_minus = tk.Button(box, text="-", width=3, bg="#F08080", font=("Arial", 9, "bold"))
                cb_minus.bind("<Button-1>", lambda e, a=arm_idx, idx=j: self._step_cart_press(a, idx, -1))
                cb_minus.bind("<ButtonRelease-1>", lambda e: self._step_cart_release())
                cb_minus.grid(row=j, column=2, padx=4, pady=2)
                cb_plus = tk.Button(box, text="+", width=3, bg="#A2CD5A", font=("Arial", 9, "bold"))
                cb_plus.bind("<Button-1>", lambda e, a=arm_idx, idx=j: self._step_cart_press(a, idx, 1))
                cb_plus.bind("<ButtonRelease-1>", lambda e: self._step_cart_release())
                cb_plus.grid(row=j, column=3, pady=2)

        self._step_fb_refresh()
        self._step_loop_50hz()

    def _step_entry_init(self, entry, text):
        entry.configure(state="normal")
        entry.delete(0, tk.END)
        entry.insert(0, text)
        entry.configure(state="readonly")

    def _step_entry_set(self, entry, value):
        entry.configure(state="normal")
        entry.delete(0, tk.END)
        entry.insert(0, f"{value:.3f}")
        entry.configure(state="readonly")

    def _step_entry_get(self, entry):
        try:
            return float(entry.get())
        except ValueError:
            return 0.0

    def _step_fk_xyzabc(self, arm_idx, joints=None):
        if joints is None:
            joints = self._step_fb_joints(arm_idx)
        if not joints:
            return None
        try:
            mat = robot.forward_kinematics(arm_idx, joints)
            if not isinstance(mat, list):
                return None
            return robot.matrix2xyzabc(mat)
        except Exception:
            return None

    def _step_cart_ik(self, arm_idx, target, ref_joints):
        try:
            mat = robot.xyzabc2matrix(target)
            mat16 = robot.mat4x4_to_mat1x16(mat)
            params = FX_InvKineSolverParams()
            for i in range(16):
                params.m_Input_IK_TargetTCP[i] = mat16[i]
            for i in range(7):
                params.m_Input_IK_RefJoint[i] = ref_joints[i]
            params.m_Input_IK_ZSPType = 0
            res = robot.inverse_kinematics(arm_idx, params)
            if not isinstance(res, FX_InvKineSolverParams):
                return None
            return [float(res.m_Output_RetJoint[i]) for i in range(7)]
        except Exception:
            return None

    def _step_cart_step(self, arm_idx, axis, direction):
        target = self._step_cart_target[arm_idx]
        ref = self._step_cart_ref[arm_idx]
        delta = float(self._step_cart_dist.get())
        target[axis] += delta * direction
        joints = self._step_cart_ik(arm_idx, target, ref)
        if joints is None:
            target[axis] -= delta * direction
            return
        if max(abs(joints[i] - ref[i]) for i in range(7)) > self._step_cart_flip_deg:
            target[axis] -= delta * direction
            return
        self._step_cart_ref[arm_idx] = joints
        obj = FXObjType.OBJ_ARM0 if arm_idx == 0 else FXObjType.OBJ_ARM1
        robot.runtime_set_joint_pos_cmd(obj, joints)

    def _step_cart_press(self, arm_idx, axis, direction):
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        fb = self._step_fb_joints(arm_idx)
        if not fb:
            messagebox.showerror('Error', 'Joint feedback not available yet')
            return
        pose = self._step_fk_xyzabc(arm_idx, fb)
        if not pose:
            messagebox.showerror('Error', 'Cartesian pose unavailable (initialize kinematics first)')
            return
        self._step_cart_target[arm_idx] = list(pose)
        self._step_cart_ref[arm_idx] = list(fb)
        self._step_cart_jog = {"arm": arm_idx, "axis": axis, "dir": direction, "repeat": False}
        self._step_cart_step(arm_idx, axis, direction)
        if self._step_cart_hold_after is not None:
            try:
                self._step_dialog.after_cancel(self._step_cart_hold_after)
            except Exception:
                pass
        self._step_cart_hold_after = self._step_dialog.after(300, self._step_cart_hold_start)

    def _step_cart_hold_start(self):
        self._step_cart_hold_after = None
        if self._step_cart_jog is not None:
            self._step_cart_jog["repeat"] = True

    def _step_cart_release(self):
        if self._step_cart_hold_after is not None:
            try:
                self._step_dialog.after_cancel(self._step_cart_hold_after)
            except Exception:
                pass
            self._step_cart_hold_after = None
        self._step_cart_jog = None

    def _step_stop(self):
        self._step_alive = False
        self._step_jog = None
        self._step_cart_jog = None
        for after_id in (getattr(self, "_step_hold_after", None),
                         getattr(self, "_step_cart_hold_after", None)):
            if after_id is not None:
                try:
                    self._step_dialog.after_cancel(after_id)
                except Exception:
                    pass
        self._step_hold_after = None
        self._step_cart_hold_after = None

    def _step_fb_joints(self, arm_idx):
        if not self.rt:
            return None
        try:
            fb = self.rt["arms"][arm_idx]["fb"]["fb_pos"]
        except (KeyError, TypeError, IndexError):
            return None
        if fb and len(fb) == 7:
            return [float(v) for v in fb]
        return None

    def _step_fb_refresh(self):
        if not self._step_alive:
            return
        d = self._step_dialog
        if d is None or not d.winfo_exists():
            self._step_alive = False
            return
        if self.rt:
            for arm_idx in (0, 1):
                fb = self._step_fb_joints(arm_idx)
                if fb:
                    for j in range(7):
                        self._step_entry_set(self._step_joint_entries[arm_idx][j], fb[j])
                    pose = self._step_fk_xyzabc(arm_idx, fb)
                    if pose:
                        for j in range(6):
                            self._step_entry_set(self._step_cart_entries[arm_idx][j], pose[j])
        d.after(200, self._step_fb_refresh)

    def _step_sync_ratio(self, arm_idx):
        vel = int(self._step_joint_vel.get())
        acc = int(self._step_joint_acc.get())
        if self._step_last_ratio[arm_idx] != (vel, acc):
            obj = FXObjType.OBJ_ARM0 if arm_idx == 0 else FXObjType.OBJ_ARM1
            robot.runtime_set_speed_ratio(obj, vel, acc)
            self._step_last_ratio[arm_idx] = (vel, acc)

    def _step_sync_cart_ratio(self, arm_idx):
        vel = int(self._step_cart_vel.get())
        acc = int(self._step_cart_acc.get())
        if self._step_last_cart_ratio[arm_idx] != (vel, acc):
            obj = FXObjType.OBJ_ARM0 if arm_idx == 0 else FXObjType.OBJ_ARM1
            robot.runtime_set_speed_ratio(obj, vel, acc)
            self._step_last_cart_ratio[arm_idx] = (vel, acc)

    def _step_joint_step(self, arm_idx, joint_idx, direction):
        step = self._step_joint_range.get() * direction
        self._step_joint_target[arm_idx][joint_idx] += step
        obj = FXObjType.OBJ_ARM0 if arm_idx == 0 else FXObjType.OBJ_ARM1
        robot.runtime_set_joint_pos_cmd(obj, list(self._step_joint_target[arm_idx]))

    def _step_loop_50hz(self):
        if not self._step_alive:
            return
        d = self._step_dialog
        if d is None or not d.winfo_exists():
            self._step_alive = False
            return
        self._step_sync_ratio(0)
        self._step_sync_cart_ratio(0)
        jog = self._step_jog
        if jog is not None and jog.get("repeat"):
            self._step_joint_step(jog["arm"], jog["joint"], jog["dir"])
        cjo = self._step_cart_jog
        if cjo is not None and cjo.get("repeat"):
            self._step_cart_step(cjo["arm"], cjo["axis"], cjo["dir"])
        d.after(20, self._step_loop_50hz)

    def _step_joint_press(self, arm_idx, joint_idx, direction):
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        fb = self._step_fb_joints(arm_idx)
        if not fb:
            messagebox.showerror('Error', 'Joint feedback not available yet')
            return
        self._step_joint_target[arm_idx] = fb
        self._step_joint_step(arm_idx, joint_idx, direction)
        self._step_jog = {"arm": arm_idx, "joint": joint_idx, "dir": direction, "repeat": False}
        if self._step_hold_after is not None:
            try:
                self._step_dialog.after_cancel(self._step_hold_after)
            except Exception:
                pass
        self._step_hold_after = self._step_dialog.after(300, self._step_joint_hold_start)

    def _step_joint_hold_start(self):
        self._step_hold_after = None
        if self._step_jog is not None:
            self._step_jog["repeat"] = True

    def _step_joint_release(self):
        if self._step_hold_after is not None:
            try:
                self._step_dialog.after_cancel(self._step_hold_after)
            except Exception:
                pass
            self._step_hold_after = None
        self._step_jog = None

    def file_client_dialog(self):
        dialog = tk.Toplevel(self.root)
        dialog.title("File Transfer")
        dialog.geometry("300x150")
        dialog.configure(bg="white")
        dialog.transient(self.root)
        dialog.grab_set()

        send_btn = tk.Button(dialog, text="Send File to Robot", width=20,
                             command=self._send_file_to_robot, bg="#4CAF50", fg="white")
        send_btn.pack(pady=20)

        recv_btn = tk.Button(dialog, text="Receive File from Robot", width=20,
                             command=self._receive_file_from_robot, bg="#2196F3", fg="white")
        recv_btn.pack(pady=10)

    def _send_file_to_robot(self):
        local_path = filedialog.askopenfilename(title="Select file to send")
        if not local_path:
            return
        remote_path = simpledialog.askstring("Remote Path", "Enter remote path (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        if robot.send_file(local_path, remote_path) == 0:
            messagebox.showinfo("Success", f"File sent to {remote_path}")
        else:
            messagebox.showerror("Failed", "Send file failed")
            return

    def _receive_file_from_robot(self):
        remote_path = simpledialog.askstring("Remote Path",
                                             "Enter remote path to receive (e.g., /home/robot/file.bin):")
        if not remote_path:
            return
        local_path = filedialog.asksaveasfilename(title="Save file as")
        if not local_path:
            return
        if robot.recv_file(local_path, remote_path) == 0:
            messagebox.showinfo("Success", f"File received and saved to {local_path}")
        else:
            messagebox.showerror("Failed", "Receive file failed")
            return

    def Estop(self):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return
        try:
            robot.emergency_stop(FXObjMask.OBJ_ALL_FLAG)
        except Exception as e:
            messagebox.showerror('Error', f'Emergency stop failed: {e}')

    def show_impedance_dialog(self, obj):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        impedance_dialog = tk.Toplevel(self.root)
        impedance_dialog.title(f"Impedance parameter settings for {obj}")
        impedance_dialog.geometry("1200x500")
        impedance_dialog.configure(bg="white")
        impedance_dialog.transient(self.root)
        impedance_dialog.resizable(True, True)
        impedance_dialog.grab_set()

        if obj == 'Arm0':
            main_frame = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame.pack(fill="both", expand=True)
            title_label = tk.Label(
                main_frame,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50',
                bg='white'
            )
            title_label.pack(pady=(0, 10))

            # ---- PD parameters row (first row) ----
            pd_frame = tk.Frame(main_frame, bg='white')
            pd_frame.pack(fill="x", pady=(5, 10))
            pd_btn = tk.Button(pd_frame, text="PD parameters", width=20,
                                command=lambda: self.pd_set('Arm0'))
            pd_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(pd_frame, text='PDP:', width=5, bg="white").grid(row=0, column=1)
            tk.Entry(pd_frame, textvariable=self.pdp_a_entry, width=50).grid(row=0, column=2, sticky="ew")
            tk.Label(pd_frame, text='PDD:', width=5, bg="white").grid(row=0, column=3)
            tk.Entry(pd_frame, textvariable=self.pdd_a_entry, width=30).grid(row=0, column=4)

            params_frame = tk.Frame(main_frame, bg='white')
            params_frame.pack(fill="x", pady=(5, 10))
            joint_kd_a_button = tk.Button(params_frame, text="JointImp parameters", width=20,
                                          command=lambda: self.joint_kd_set('Arm0'))
            joint_kd_a_button.grid(row=0, column=0, padx=5, pady=10)
            k_a_label = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label.grid(row=0, column=1)
            k_a_entry = tk.Entry(params_frame, textvariable=self.k_a_entry, width=50)
            k_a_entry.grid(row=0, column=2, sticky="ew")
            d_a_label = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label.grid(row=0, column=3)
            d_a_entry = tk.Entry(params_frame, textvariable=self.d_a_entry, width=30)
            d_a_entry.grid(row=0, column=4)

            cart_kd_a_button = tk.Button(params_frame, text="CartImp parameters", width=20,
                                         command=lambda: self.cart_kd_set('Arm0'))
            cart_kd_a_button.grid(row=1, column=0, padx=5, pady=(20, 10))
            k_a_label_ = tk.Label(params_frame, text='K:', width=5, bg="white")
            k_a_label_.grid(row=1, column=1)
            cart_k_a_entry = tk.Entry(params_frame, textvariable=self.cart_k_a_entry, width=50)
            cart_k_a_entry.grid(row=1, column=2, sticky="ew")
            d_a_label_ = tk.Label(params_frame, text='D:', width=5, bg="white")
            d_a_label_.grid(row=1, column=3)
            cart_d_a_entry = tk.Entry(params_frame, textvariable=self.cart_d_a_entry, width=30)
            cart_d_a_entry.grid(row=1, column=4)

            # Force/Torque parameters row
            force_torque_frame = tk.Frame(main_frame, bg='white')
            force_torque_frame.pack(fill="x", pady=(5, 10))
            set_ft_btn = tk.Button(force_torque_frame, text="Set Force/Torque", width=20,
                                   command=lambda: self.force_torque_set('Arm0'))
            set_ft_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(1mm~200mm)):",
                     font=('Arial', 9), bg='white', width=50).grid(row=0, column=1)
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_a_entry, width=30)
            force_entry.grid(row=0, column=2)
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white', width=50).grid(row=1, column=1)
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_a_entry, width=30)
            torque_entry.grid(row=1, column=2)


            # reference orientation 
            ref_ori_frame = tk.Frame(main_frame, bg='white')
            ref_ori_frame.pack(fill="x", pady=(5, 10))
            set_ro_btn = tk.Button(ref_ori_frame, text="Set reference orientation", width=20,
                                   command=lambda: self.ref_ori_set('Arm0'))
            set_ro_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(ref_ori_frame, text="orientation type:",
                     font=('Arial', 9), bg='white', width=15).grid(row=0, column=1)
            self.refori_combo_l = ttk.Combobox(ref_ori_frame, values=["disable", "anyBase", "TCP"],
                                         textvariable=self.refori_var_l,
                                         state="readonly", width=8)
            self.refori_combo_l.current(0)
            self.refori_combo_l.grid(row=0, column=2)
            tk.Label(ref_ori_frame, text="orientation(A/B/C):", font=('Arial', 9),
                     bg='white', width=20).grid(row=0, column=3)
            ref_ori_entry = tk.Entry(ref_ori_frame, textvariable=self.ref_ori_a_entry, width=20)
            ref_ori_entry.grid(row=0, column=4)
            tk.Label(ref_ori_frame, text="Select 'anyBase', please set reference orientation in euler angle A/B/C. ",
                      font=('Arial', 9),
                     bg='white', width=70).grid(row=0, column=5)


            params_save_frame = tk.Frame(main_frame, bg='white')
            params_save_frame.pack(fill="x", pady=(20, 10))
            load_ini_param_a_button = tk.Button(params_save_frame, text="Load default parameters",
                                                command=self.load_default_param)
            load_ini_param_a_button.pack(side='left', padx=(200, 0))

            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Arm0'))
            save_param_a_button.pack(side='left', padx=(50, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Arm0'))
            load_param_a_button.pack(side='left', padx=(50, 10))


        elif obj == 'Arm1':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the impedance parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50',
                bg='white'
            )
            title_label1.pack(pady=(0, 10))

            # ---- PD parameters row (first row) ----
            pd_frame1 = tk.Frame(main_frame1, bg='white')
            pd_frame1.pack(fill="x", pady=(5, 10))
            pd_btn1 = tk.Button(pd_frame1, text="PD parameters", width=20,
                                 command=lambda: self.pd_set('Arm1'))
            pd_btn1.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(pd_frame1, text='PDP:', width=5, bg="white").grid(row=0, column=1)
            tk.Entry(pd_frame1, textvariable=self.pdp_b_entry, width=50).grid(row=0, column=2, sticky="ew")
            tk.Label(pd_frame1, text='PDD:', width=5, bg="white").grid(row=0, column=3)
            tk.Entry(pd_frame1, textvariable=self.pdd_b_entry, width=30).grid(row=0, column=4)

            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            joint_kd_a_button1 = tk.Button(params_frame1, text="Set joint impedance parameters", width=20,
                                           command=lambda: self.joint_kd_set('Arm1'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            k_a_label1 = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)
            k_b_entry = tk.Entry(params_frame1, textvariable=self.k_b_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")
            d_b_label = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)
            d_b_entry = tk.Entry(params_frame1, textvariable=self.d_b_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            cart_kd_b_button = tk.Button(params_frame1, text="CartImp parameters", width=20,
                                         command=lambda: self.cart_kd_set('Arm1'))
            cart_kd_b_button.grid(row=1, column=0, padx=5, pady=(20, 10))
            k_b_label_ = tk.Label(params_frame1, text='K:', width=5, bg="white")
            k_b_label_.grid(row=1, column=1)
            cart_k_b_entry = tk.Entry(params_frame1, textvariable=self.cart_k_b_entry, width=50)
            cart_k_b_entry.grid(row=1, column=2, sticky="ew")
            d_b_label_ = tk.Label(params_frame1, text='D:', width=5, bg="white")
            d_b_label_.grid(row=1, column=3)
            cart_d_b_entry = tk.Entry(params_frame1, textvariable=self.cart_d_b_entry, width=30)
            cart_d_b_entry.grid(row=1, column=4)

            # Force/Torque parameters row
            force_torque_frame = tk.Frame(main_frame1, bg='white')
            force_torque_frame.pack(fill="x", pady=(10, 5))
            set_ft_btn = tk.Button(force_torque_frame, text="Set Force/Torque", width=20,
                                   command=lambda: self.force_torque_set('Arm1'))
            set_ft_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(force_torque_frame, text="Force(dir_x,dir_y,dir_z,force(-50~50N),distance(1mm~200mm)):",
                     font=('Arial', 9), bg='white', width=50).grid(row=0, column=1)
            force_entry = tk.Entry(force_torque_frame, textvariable=self.force_b_entry, width=25)
            force_entry.grid(row=0, column=2)
            tk.Label(force_torque_frame, text="Torque(dir_x,dir_y,dir_z,torque(N*m),distance(deg))", font=('Arial', 9),
                     bg='white', width=50).grid(row=1, column=1)
            torque_entry = tk.Entry(force_torque_frame, textvariable=self.torque_b_entry, width=25)
            torque_entry.grid(row=1, column=2)

            # reference orientation 
            ref_ori_frame = tk.Frame(main_frame1, bg='white')
            ref_ori_frame.pack(fill="x", pady=(5, 10))
            set_ro_btn = tk.Button(ref_ori_frame, text="Set reference orientation", width=20,
                                   command=lambda: self.ref_ori_set('Arm1'))
            set_ro_btn.grid(row=0, column=0, padx=5, pady=10)
            tk.Label(ref_ori_frame, text="orientation type:",
                     font=('Arial', 9), bg='white', width=15).grid(row=0, column=1)
            self.refori_combo_r = ttk.Combobox(ref_ori_frame, values=["disable", "anyBase", "TCP"],
                                         textvariable=self.refori_var_r,
                                         state="readonly", width=8)
            self.refori_combo_r.current(0)
            self.refori_combo_r.grid(row=0, column=2)

            tk.Label(ref_ori_frame, text="orientation(A/B/C):", font=('Arial', 9),
                     bg='white', width=20).grid(row=0, column=3)
            ref_ori_entry = tk.Entry(ref_ori_frame, textvariable=self.ref_ori_b_entry, width=20)
            ref_ori_entry.grid(row=0, column=4)

            tk.Label(ref_ori_frame, text="Select 'anyBase', please set reference orientation in euler angle A/B/C. ",
                      font=('Arial', 9),
                     bg='white', width=70).grid(row=0, column=5)

            params_save_frame = tk.Frame(main_frame1, bg='white')
            params_save_frame.pack(fill="x", pady=(20, 10))
            load_ini_param_a_button = tk.Button(params_save_frame, text="Load default parameters",
                                                command=self.load_default_param)
            load_ini_param_a_button.pack(side='left', padx=(200, 0))
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Arm1'))
            save_param_a_button.pack(side='left', padx=(50, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Arm1'))
            load_param_a_button.pack(side='left', padx=(50, 10))

        elif obj == 'Body':
            main_frame1 = tk.Frame(impedance_dialog, padx=20, pady=20, bg='white')
            main_frame1.pack(fill="both", expand=True)
            title_label1 = tk.Label(
                main_frame1,
                text=f"Set the PD parameters of {obj}",
                font=('Arial', 10, 'bold'),
                fg='#2c3e50',
                bg='white'
            )
            title_label1.pack(pady=(0, 10))

            params_frame1 = tk.Frame(main_frame1, bg='white')
            params_frame1.pack(fill="x", pady=(5, 10))

            joint_kd_a_button1 = tk.Button(params_frame1, text="PD parameters", width=20,
                                           command=lambda: self.pd_set('Body'))
            joint_kd_a_button1.grid(row=0, column=0, padx=5, pady=10)
            k_a_label1 = tk.Label(params_frame1, text='PDP:', width=5, bg="white")
            k_a_label1.grid(row=0, column=1)
            k_b_entry = tk.Entry(params_frame1, textvariable=self.pdp_entry, width=50)
            k_b_entry.grid(row=0, column=2, sticky="ew")
            d_b_label = tk.Label(params_frame1, text='PDD:', width=5, bg="white")
            d_b_label.grid(row=0, column=3)
            d_b_entry = tk.Entry(params_frame1, textvariable=self.pdd_entry, width=30)
            d_b_entry.grid(row=0, column=4)

            params_save_frame = tk.Frame(main_frame1, bg='white')
            params_save_frame.pack(fill="x", pady=(20, 10))
            load_ini_param_a_button = tk.Button(params_save_frame, text="Load default parameters",
                                                command=self.load_default_body_pd)
            load_ini_param_a_button.pack(side='left', padx=(200, 0))
            save_param_a_button = tk.Button(params_save_frame, text="Save parameters",
                                            command=lambda: self.save_param('Body'))
            save_param_a_button.pack(side='left', padx=(50, 0))
            load_param_a_button = tk.Button(params_save_frame, text="Import parameters",
                                            command=lambda: self.load_param('Body'))
            load_param_a_button.pack(side='left', padx=(50, 10))

    def pd_set(self, obj):
        """Unified PD parameter setter for Arm0/Arm1/Body.

        Uses the merged interface runtime_set_pd(obj_type, p, d) for all objects.
        Arm0/Arm1 use pdp_a/pdd_a (or pdp_b/pdd_b) StringVars; Body uses the
        shared pdp_entry/pdd_entry StringVars.
        """
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj == 'Arm0':
            p_var = self.pdp_a_entry
            d_var = self.pdd_a_entry
            obj_type = FXObjType.OBJ_ARM0
        elif obj == 'Arm1':
            p_var = self.pdp_b_entry
            d_var = self.pdd_b_entry
            obj_type = FXObjType.OBJ_ARM1
        elif obj == 'Body':
            p_var = self.pdp_entry
            d_var = self.pdd_entry
            obj_type = FXObjType.OBJ_BODY
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        num_joints = 7
        p_str = p_var.get().strip()
        if not p_str:
            messagebox.showerror("Error", "PDP parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(p_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid PDP format: {result}")
            return
        p_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "PDD parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid PDD format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]

        ret = robot.runtime_set_pd(obj_type, p_list, d_list)
        if ret != 0:
            messagebox.showerror('Failed!',
                                 f"Set {obj} PD parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        messagebox.showinfo('OK', f"Set {obj} PD parameters success.")

    def joint_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj == 'Arm0':
            k_var = self.k_a_entry
            d_var = self.d_a_entry
            obj_type = FXObjType.OBJ_ARM0
            num_joints = 7
        elif obj == 'Arm1':
            k_var = self.k_b_entry
            d_var = self.d_b_entry
            obj_type = FXObjType.OBJ_ARM1
            num_joints = 7
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "K parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid K format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "D parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid D format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]

        ret = robot.runtime_set_joint_k(obj_type, k_list)
        if ret != 0:
            messagebox.showerror('Failed!', f"Set {obj} K parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        ret = robot.runtime_set_joint_d(obj_type, d_list)
        if ret != 0:
            messagebox.showerror('Failed!', f"Set {obj} D parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    def cart_kd_set(self, obj):
        if not self.connected:
            messagebox.showerror('Error', 'Please connect robot')
            return

        if obj == 'Arm0':
            k_var = self.cart_k_a_entry
            d_var = self.cart_d_a_entry
            obj_type = FXObjType.OBJ_ARM0
            num_joints = 7
        elif obj == 'Arm1':
            k_var = self.cart_k_b_entry
            d_var = self.cart_d_b_entry
            obj_type = FXObjType.OBJ_ARM1
            num_joints = 7
        else:
            messagebox.showerror('Error', f'Unknown obj: {obj}')
            return

        k_str = k_var.get().strip()
        if not k_str:
            messagebox.showerror("Error", "K parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(k_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid K format: {result}")
            return
        k_list = [float(x) for x in result.split(',')]

        d_str = d_var.get().strip()
        if not d_str:
            messagebox.showerror("Error", "D parameter cannot be empty!")
            return
        is_valid, result = self.validate_point(d_str, num_joints)
        if not is_valid:
            messagebox.showerror("Error", f"Invalid D format: {result}")
            return
        d_list = [float(x) for x in result.split(',')]

        ret = robot.runtime_set_cart_k(obj_type, k_list)
        if ret != 0:
            messagebox.showerror('Failed!', f"Set {obj} Cartesian K parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return
        ret = robot.runtime_set_cart_d(obj_type, d_list)
        if ret != 0:
            messagebox.showerror('Failed!', f"Set {obj} Cartesian D parameters failed. Error msg: {robot._get_operate_error_msg(ret)}")
            return

    def load_default_param(self):
        self.cart_k_b_entry.set("3000,3000,3000,100,100,100,50")
        self.cart_k_a_entry.set("3000,3000,3000,100,100,100,50")
        self.cart_d_a_entry.set("0.1,0.1,0.1,0.1,0.1,0.1,0.11")
        self.cart_d_b_entry.set("0.1,0.1,0.1,0.1,0.1,0.1,0.11")

        self.k_a_entry.set("3,3,3,2,1,1,1")
        self.k_b_entry.set("3,3,3,2,1,1,1")
        self.d_a_entry.set("0.2,0.2,0.2,0.2,0.2,0.2,0.2")
        self.d_b_entry.set("0.2,0.2,0.2,0.2,0.2,0.2,0.2")

        self.force_a_entry.set("0,1,0,25,25")
        self.torque_a_entry.set("0,1,0,5,10")
        self.force_b_entry.set("0,1,0,25,25")
        self.torque_b_entry.set("0,1,0,5,10")

        # Arm0 / Arm1 PD parameters
        self.pdp_a_entry.set("14,14,14,10.5,5.6,5.6,5.6")
        self.pdd_a_entry.set("0.3,0.3,0.3,0.3,0.3,0.3,0.3")
        self.pdp_b_entry.set("14,14,14,10.5,5.6,5.6,5.6")
        self.pdd_b_entry.set("0.3,0.3,0.3,0.3,0.3,0.3,0.3")

    def load_default_body_pd(self):
        self.pdp_entry.set("28,26,28,14,6,4,0")
        self.pdd_entry.set("5.5,3.7,2.0,2.2,2.0,0.6,0")

    def load_default_tools(self):
        self.arm0_tool_dyn_entry.set("0,0,0,0,0,0,0,0,0,0")
        self.arm1_tool_dyn_entry.set("0,0,0,0,0,0,0,0,0,0")
        self.arm0_tool_kine_entry.set("0,0,0,0,0,0")
        self.arm1_tool_kine_entry.set("0,0,0,0,0,0")

    def save_param(self, obj):
        self.params = []
        if obj == 'Arm0':
            params_to_save = [
                self.k_a_entry.get(),
                self.d_a_entry.get(),
                self.cart_k_a_entry.get(),
                self.cart_d_a_entry.get(),
                self.force_a_entry.get(),
                self.torque_a_entry.get(),
            ]
        elif obj == 'Arm1':
            params_to_save = [
                self.k_b_entry.get(),
                self.d_b_entry.get(),
                self.cart_k_b_entry.get(),
                self.cart_d_b_entry.get(),
                self.force_b_entry.get(),
                self.torque_b_entry.get(),
            ]
        elif obj == 'Body':
            params_to_save = [
                self.pdp_entry.get(),
                self.pdd_entry.get()
            ]
        else:
            messagebox.showerror("Error", f"Unknown obj: {obj}")
            return

        file_path = filedialog.asksaveasfilename(
            defaultextension=".txt",
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title=f"Save {obj} parameters"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'w') as f:
                for param in params_to_save:
                    f.write(param.strip() + '\n')
            messagebox.showinfo("Success", f"{obj} parameters saved successfully")
        except Exception as e:
            messagebox.showerror("Error", f"Error saving file: {str(e)}")

    def load_param(self, obj):
        file_path = filedialog.askopenfilename(
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")],
            title=f"Select parameter file for {obj}"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'r') as f:
                lines = [line.strip() for line in f.readlines() if line.strip()]
        except Exception as e:
            messagebox.showerror("Error", f"Error reading file: {str(e)}")
            return

        if obj == 'Arm0':
            if len(lines) >= 4:
                self.k_a_entry.set(lines[0])
                self.d_a_entry.set(lines[1])
                self.cart_k_a_entry.set(lines[2])
                self.cart_d_a_entry.set(lines[3])
                self.force_a_entry.set(lines[4]),
                self.torque_a_entry.set(lines[5])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
                return
        elif obj == 'Arm1':
            if len(lines) >= 4:
                self.k_b_entry.set(lines[0])
                self.d_b_entry.set(lines[1])
                self.cart_k_b_entry.set(lines[2])
                self.cart_d_b_entry.set(lines[3])
                self.force_b_entry.set(lines[4]),
                self.torque_b_entry.set(lines[5])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 4)")
                return
        elif obj == 'Body':
            if len(lines) >= 2:
                self.pdp_entry.set(lines[0])
                self.pdd_entry.set(lines[1])
            else:
                messagebox.showerror("Error", "File does not contain enough parameters (need 2)")
                return
        else:
            messagebox.showerror("Error", f"Unknown obj: {obj}")

    def force_torque_set(self, obj):
        """Set force and torque control parameters for Arm0/Arm1."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'Force/Torque not supported for {obj}')
            return
        try:
            if obj == 'Arm0':
                force_str = self.force_a_entry.get().strip()
                torque_str = self.torque_a_entry.get().strip()
                obj_type = FXObjType.OBJ_ARM0
            else:
                force_str = self.force_b_entry.get().strip()
                torque_str = self.torque_b_entry.get().strip()
                obj_type = FXObjType.OBJ_ARM1

            # Parse force list (5 floats)
            force_list = [float(x) for x in force_str.split(',')] if force_str else [0] * 5
            if len(force_list) != 5:
                messagebox.showerror('Error', 'Force Ctrl must have 5 comma-separated values')
                return
            torque_list = [float(x) for x in torque_str.split(',')] if torque_str else [0] * 5
            if len(torque_list) != 5:
                messagebox.showerror('Error', 'Torque Ctrl must have 5 comma-separated values')
                return

            ret = robot.runtime_set_force_ctrl(obj_type, force_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set force ctrl failed for {obj}. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            ret = robot.runtime_set_torque_ctrl(obj_type, torque_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set torque ctrl failed for {obj}. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', f"Force/Torque set failed: {e}")

    def ref_ori_set(self,obj):
        """Set referenceorientation parameters for Arm0/Arm1."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'ref ori not supported for {obj}')
            return
        try:
            ref_type=''
            if obj == 'Arm0':
                obj_type = FXObjType.OBJ_ARM0
                ori=self.ref_ori_a_entry.get().strip()
                ref_type = self.refori_var_l.get()

            else:  # Arm1
                obj_type = FXObjType.OBJ_ARM1
                ori=self.ref_ori_b_entry.get().strip()
                ref_type = self.refori_var_r.get()

            if ref_type=="disable":
                ref_type=FXRefOriType.FX_REFORI_TYPE_NULL
            elif ref_type=="anyBase":
                ref_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
            elif ref_type=="TCP":
                ref_type=FXRefOriType.FX_REFORI_TYPE_TCP
            ori_list=[float(x) for x in ori.split(',')] if ori else [0] * 3
            if len(ori_list) != 3 :
                messagebox.showerror('Error', 'Orientation must have 3 values in cartesian impedance')
                return
            ret = robot.runtime_set_ref_ori(obj_type,ref_type, ori_list)
            if ret != 0:
                messagebox.showerror('Failed!', f"Set ref ori failed for {obj}. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', f"reference orientation set failed: {e}")

    def _obj_name_to_type(self, name):
        mapping = {
            'Arm0': FXObjType.OBJ_ARM0,
            'Arm1': FXObjType.OBJ_ARM1,
            'Body': FXObjType.OBJ_BODY,
            'Head': FXObjType.OBJ_HEAD,
            'Lift': FXObjType.OBJ_LIFT
        }
        return mapping.get(name, FXObjType.OBJ_ARM0)

    def _obj_name_to_mask(self, name):
        mapping = {
            'Arm0': FXObjMask.OBJ_ARM0_FLAG,
            'Arm1': FXObjMask.OBJ_ARM1_FLAG,
            'Body': FXObjMask.OBJ_BODY_FLAG,
            'Head': FXObjMask.OBJ_HEAD_FLAG,
            'Lift': FXObjMask.OBJ_LIFT_FLAG,
        }
        return mapping.get(name, 0)

    def pd_state(self, obj):
        """Switch to PD mode (Arm0/Arm1/Body)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1', 'Body'):
            messagebox.showerror('Error', f'{obj} does not support PD mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.k_a_entry.get().strip()
                d_str = self.d_a_entry.get().strip()
            elif obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.k_b_entry.get().strip()
                d_str = self.d_b_entry.get().strip()
            else:
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                k_str = self.pdp_entry.get().strip()
                d_str = self.pdd_entry.get().strip()
            if obj == 'Body':
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
                if len(k_list) != 7 or len(d_list) != 7:
                    messagebox.showerror('Error', 'body pdp/pdd must have 7 values')
                    return
            else:
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
                if len(k_list) != 7 or len(d_list) != 7:
                    messagebox.showerror('Error', 'K/D must have 7 values')
                    return
            ret = robot.switch_to_pd_mode(obj_type, 2000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!',
                                     f'{obj} switch to PD failed. Error msg: {robot._get_operate_error_msg(ret)}')
                return
        except Exception as e:
            messagebox.showerror('Error', f'PD switch failed: {e}')

    def jointImp_state(self, obj):
        """Switch to Joint Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1', 'Body'):
            messagebox.showerror('Error', f'{obj} does not support Joint Impedance mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.k_a_entry.get().strip()
                d_str = self.d_a_entry.get().strip()
            if obj == 'Arm1':
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.k_b_entry.get().strip()
                d_str = self.d_b_entry.get().strip()
            if obj == 'Body':
                vel = int(self.body_speed_entry.get())
                acc = int(self.body_accel_entry.get())
                k_str = self.pdp_entry.get().strip()
                d_str = self.pdd_entry.get().strip()
            # Parse K and D lists
            if obj == 'Body':
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 6
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 6
                if len(k_list) != 6 or len(d_list) != 6:
                    messagebox.showerror('Error', 'body pdp/pdd must have 6 values')
                    return
            else:
                k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
                d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
                if len(k_list) != 7 or len(d_list) != 7:
                    messagebox.showerror('Error', 'K/D must have 7 values')
                    return
            ret = robot.switch_to_imp_joint_mode(obj_type, 2000, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to joint impedance failed. Error msg: {robot._get_operate_error_msg(ret)}')
                return
        except Exception as e:
            messagebox.showerror('Error', f'Joint Impedance switch failed: {e}')

    def cartImp_state(self, obj):
        """Switch to Cartesian Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Cartesian Impedance mode')
            return

        print(f"====={self.refori_var_l.get(),self.refori_var_r.get()}")
        try:
            obj_type = self._obj_name_to_type(obj)
            ref_type=''
            if obj == 'Arm0':
                vel = int(self.left_speed_entry.get())
                acc = int(self.left_accel_entry.get())
                k_str = self.cart_k_a_entry.get().strip()
                d_str = self.cart_d_a_entry.get().strip()
                ori=self.ref_ori_a_entry.get().strip()
                ref_type = self.refori_var_l.get()

            else:  # Arm1
                vel = int(self.right_speed_entry.get())
                acc = int(self.right_accel_entry.get())
                k_str = self.cart_k_b_entry.get().strip()
                d_str = self.cart_d_b_entry.get().strip()
                ori=self.ref_ori_b_entry.get().strip()
                ref_type = self.refori_var_r.get()

            if ref_type=="disable":
                ref_type=FXRefOriType.FX_REFORI_TYPE_NULL
            elif ref_type=="anyBase":
                ref_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
            elif ref_type=="TCP":
                ref_type=FXRefOriType.FX_REFORI_TYPE_TCP
            
            k_list = [float(x) for x in k_str.split(',')] if k_str else [0] * 7
            d_list = [float(x) for x in d_str.split(',')] if d_str else [0] * 7
            ori_list=[float(x) for x in ori.split(',')] if ori else [0] * 3
            if len(k_list) != 7 or len(d_list) != 7:
                messagebox.showerror('Error', 'Cartesian K/D must have 7 values')
                return
            if len(ori_list) != 3 :
                messagebox.showerror('Error', 'Orientation must have 3 values in cartesian impedance')
                return
            ret = robot.switch_to_imp_cart_mode(obj_type, 2000,ref_type, ori_list, vel, acc, k_list, d_list)
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to cartesian impedance failed. Error msg: {robot._get_operate_error_msg(ret)}')
                return
        except Exception as e:
            messagebox.showerror('Error', f'Cartesian Impedance switch failed: {e}')

    def forceImp_state(self, obj):
        """Switch to Force Impedance mode (only for Arm0/Arm1)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support Force Impedance mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            ref_type=''
            if obj == 'Arm0':
                force_str = self.force_a_entry.get().strip()
                torque_str = self.torque_a_entry.get().strip()
                ori=self.ref_ori_a_entry.get().strip()
                ref_type = self.refori_var_l.get()
            else:
                force_str = self.force_b_entry.get().strip()
                torque_str = self.torque_b_entry.get().strip()
                ori=self.ref_ori_b_entry.get().strip()
                ref_type = self.refori_var_r.get()

            if ref_type=="disable":
                ref_type=FXRefOriType.FX_REFORI_TYPE_NULL
            elif ref_type=="anyBase":
                ref_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
            elif ref_type=="TCP":
                ref_type=FXRefOriType.FX_REFORI_TYPE_TCP

            ori_list=[float(x) for x in ori.split(',')] if ori else [0] * 3
            if len(ori_list) != 3 :
                messagebox.showerror('Error', 'Orientation must have 3 values in force impedance')
                return
            # Parse to list of floats
            force_ctrl = [float(x) for x in force_str.split(',')] if force_str else [0.0] * 5
            torque_ctrl = [float(x) for x in torque_str.split(',')] if torque_str else [0.0] * 5
            if len(force_ctrl) != 5 or len(torque_ctrl) != 5:
                messagebox.showerror('Error', 'Force Ctrl and Torque Ctrl must each have 5 comma-separated values')
                return
            ret = robot.switch_to_imp_force_mode(obj_type, 2000, ref_type, ori_list, force_ctrl, torque_ctrl)
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to force impedance failed. Error msg: {robot._get_operate_error_msg(ret)}')
                return
        except ValueError as e:
            messagebox.showerror('Error', f'Invalid number format in Force/Torque: {e}')
        except Exception as e:
            messagebox.showerror('Error', f'Force Impedance switch failed: {e}')

    def drag_state(self, obj):

        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        if obj not in ('Arm0', 'Arm1'):
            messagebox.showerror('Error', f'{obj} does not support drag mode')
            return
        try:
            obj_type = self._obj_name_to_type(obj)

            num_joints = 7
            if obj == 'Arm0':
                k_var = self.k_a_entry
                d_var = self.d_a_entry
                cart_k_ver = self.cart_k_a_entry
                cart_d_ver = self.cart_d_a_entry
            if obj == 'Arm1':
                k_var = self.k_b_entry
                d_var = self.d_b_entry
                cart_k_ver = self.cart_k_b_entry
                cart_d_ver = self.cart_d_b_entry
            k_str = k_var.get().strip()
            if not k_str:
                messagebox.showerror("Error", "K parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(k_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid K format: {result}")
                return
            k_list = [float(x) for x in result.split(',')]

            d_str = d_var.get().strip()
            if not d_str:
                messagebox.showerror("Error", "D parameter cannot be empty!")
                return
            is_valid, result = self.validate_point(d_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid D format: {result}")
                return
            d_list = [float(x) for x in result.split(',')]

            cart_k_str = cart_k_ver.get().strip()
            if not cart_k_str:
                messagebox.showerror("Error", "K parameter cannot be empty!")
                return
            is_valid, cart_result = self.validate_point(cart_k_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid K format: {result}")
                return
            cart_k_list = [float(x) for x in cart_result.split(',')]

            cart_d_str = cart_d_ver.get().strip()
            if not d_str:
                messagebox.showerror("Error", "D parameter cannot be empty!")
                return
            is_valid, cart_result = self.validate_point(cart_d_str, num_joints)
            if not is_valid:
                messagebox.showerror("Error", f"Invalid D format: {cart_result}")
                return
            cart_d_list = [float(x) for x in cart_result.split(',')]
            ref_type=''
            if obj == 'Arm0':
                mode = self.drag_combo.get()
                ori=self.ref_ori_a_entry.get().strip()
                ref_type = self.refori_var_l.get()
            else:
                mode = self.drag_combo_r.get()
                ori=self.ref_ori_b_entry.get().strip()
                ref_type = self.refori_var_r.get()
            ori_list=[float(x) for x in ori.split(',')] if ori else [0] * 3
            if len(ori_list) != 3 :
                messagebox.showerror('Error', 'Orientation must have 3 values in DragCart*')
                return
            if ref_type=="disable":
                ref_type=FXRefOriType.FX_REFORI_TYPE_NULL
            elif ref_type=="anyBase":
                ref_type=FXRefOriType.FX_REFORI_TYPE_ANYBASE
            elif ref_type=="TCP":
                ref_type=FXRefOriType.FX_REFORI_TYPE_TCP

            if mode == "joint":
                ret = robot.switch_to_drag_joint(obj_type, 1000, k_list, d_list)
            elif mode == "cartX":
                ret = robot.switch_to_drag_cart_x(obj_type, 1000, ref_type, ori_list, cart_k_list, cart_d_list)
            elif mode == "cartY":
                ret = robot.switch_to_drag_cart_y(obj_type, 1000, ref_type, ori_list, cart_k_list, cart_d_list)
            elif mode == "cartZ":
                ret = robot.switch_to_drag_cart_z(obj_type, 1000, ref_type, ori_list, cart_k_list, cart_d_list)
            elif mode == "cartR":
                ret = robot.switch_to_drag_cart_r(obj_type, 1000, ref_type, ori_list, cart_k_list, cart_d_list)
            else:
                messagebox.showerror('Error', f'Unknown drag mode: {mode}')
                return
            if ret != 0:
                messagebox.showerror('Failed!', f'{obj} switch to {mode} drag failed. Error msg: {robot._get_operate_error_msg(ret)}')
        except Exception as e:
            messagebox.showerror('Error', f'Drag mode switch failed: {e}')

    def error_get(self, obj):
        """Get servo error codes for the specified object and display in hex format."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            ret,msg = robot.get_servo_error_codes(obj_type)
            if ret !=0:
                messagebox.showerror("Failed!", f"{obj} get error codes failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
            else:
                messagebox.showinfo(f'{obj} Servo Error Details:', msg)
        except Exception as e:
            messagebox.showerror('Error', f"Failed to get error codes: {e}")

    def release_brake(self, obj):
        """Release brake for the specified object (unlock)."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            result = messagebox.askokcancel("Confirm",
                                            f"Confirm to perform the UnBrake operation on {obj}?")
            if result:
                obj_type = self._obj_name_to_type(obj)
                if obj in ('Arm0', 'Arm1'):
                    axis_mask = 0x7F  # 7 axes
                elif obj == 'Body':
                    axis_mask = 0x3F  # 6 axes
                elif obj == 'Head':
                    axis_mask = 0x07  # 3 axes
                elif obj == 'Lift':
                    axis_mask = 0x03  # 2 axes
                else:
                    axis_mask = 0xFF
                ret=robot.config_brake_unlock(obj_type, axis_mask)
                if ret!= 0:
                    messagebox.showerror('Failed!', f"{obj} release brake failed. Error msg: {robot._get_operate_error_msg(ret)}")
                    return
        except Exception as e:
            messagebox.showerror('Error', f"Release brake failed: {e}")

    def brake(self, obj):
        """Apply brake (lock) for the specified object."""
        if not self.connected:
            messagebox.showerror('Error', 'Robot not connected')
            return
        try:
            obj_type = self._obj_name_to_type(obj)
            # Lock brakes for all axes
            if obj in ('Arm0', 'Arm1'):
                axis_mask = 0x7F
            elif obj == 'Body':
                axis_mask = 0x3F
            elif obj == 'Head':
                axis_mask = 0x07
            elif obj == 'Lift':
                axis_mask = 0x03
            else:
                axis_mask = 0xFF
            ret=robot.config_brake_lock(obj_type, axis_mask)
            if ret!= 0:
                messagebox.showerror('Failed!', f"{obj} brake lock failed. Error msg: {robot._get_operate_error_msg(ret)}")
                return
        except Exception as e:
            messagebox.showerror('Error', f"Brake lock failed: {e}")

    def config_settings_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return

        config_ip_window = tk.Toplevel(self.root)
        config_ip_window.title("Robot Settings")
        config_ip_window.geometry("500x800")
        config_ip_window.configure(bg="white")
        config_ip_window.transient(self.root)
        config_ip_window.resizable(False, False)
        config_ip_window.grab_set()

        # ==================== Robot Serial Number ====================
        sn_title_frame = tk.Frame(config_ip_window, bg="white")
        sn_title_frame.pack(fill="x", padx=5, pady=(15, 5))
        tk.Label(sn_title_frame, text="Robot Serial Number", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x')

        # Current SN
        cur_sn_frame = tk.Frame(config_ip_window, bg="white")
        cur_sn_frame.pack(fill="x", padx=20, pady=(8, 4))
        tk.Label(cur_sn_frame, text="Current SN:", width=12, anchor='w', bg="white",
                 font=("Arial", 10)).pack(side="left")
        cur_sn_label = tk.Label(cur_sn_frame, text="", bg="white", fg="#2196F3",
                                 font=("Arial", 10, "bold"), anchor='w')
        cur_sn_label.pack(side="left", fill="x", expand=True, padx=5)

        def refresh_sn():
            try:
                ret, sn = robot.get_system_sn()
                if ret == 0 and sn:
                    cur_sn_label.config(text=sn)
                else:
                    cur_sn_label.config(text="Failed to get SN")
            except Exception as e:
                cur_sn_label.config(text=f"Error: {e}")

        refresh_sn()
        tk.Button(cur_sn_frame, text="Refresh", width=8,
                  font=("Arial", 9), command=refresh_sn).pack(side="left", padx=5)

        # New SN entry
        new_sn_frame = tk.Frame(config_ip_window, bg="white")
        new_sn_frame.pack(fill="x", padx=20, pady=(8, 4))
        tk.Label(new_sn_frame, text="New SN:", width=12, anchor='w', bg="white",
                 font=("Arial", 10)).pack(side="left")
        new_sn_var = tk.StringVar()
        tk.Entry(new_sn_frame, textvariable=new_sn_var, width=30,
                 font=("Arial", 10)).pack(side="left", padx=5)
        tk.Label(new_sn_frame, text="max 29 chars", bg="white",
                 fg="gray", font=("Arial", 8)).pack(side="left")

        def apply_new_sn():
            sn_str = new_sn_var.get().strip()
            if not sn_str:
                messagebox.showerror("Error", "Please enter a new serial number!")
                return
            if len(sn_str) > 29:
                messagebox.showerror("Error", "Serial number must be at most 29 characters!")
                return
            if not messagebox.askyesno("Confirm", f"Set robot SN to '{sn_str}'?"):
                return
            try:
                ret = robot.set_system_sn(sn_str)
                if ret == 0:
                    messagebox.showinfo("Success", f"Serial number set to '{sn_str}'.")
                    new_sn_var.set("")
                    refresh_sn()
                else:
                    messagebox.showerror("Failed",
                                         f"Failed to set SN.\nError msg:  {robot._get_operate_error_msg(ret)}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to set SN: {e}")

        sn_btn_frame = tk.Frame(config_ip_window, bg="white")
        sn_btn_frame.pack(pady=(4, 10))
        tk.Button(sn_btn_frame, text="Set SN", width=12,
                  font=("Arial", 10, "bold"), bg="#4CAF50", fg="white",
                  command=apply_new_sn).pack(side="left", padx=10)

        # ---------- Title ----------
        title_frame = tk.Frame(config_ip_window, bg="white")
        title_frame.pack(fill="x", padx=5, pady=(15, 10))
        tk.Label(title_frame, text="Modify Robot IP Address", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x')

        # ---------- Current IP ----------
        current_frame = tk.Frame(config_ip_window, bg="white")
        current_frame.pack(fill="x", padx=20, pady=(10, 5))
        tk.Label(current_frame, text="Current IP:", width=12, anchor='w', bg="white",
                 font=("Arial", 10)).pack(side="left")
        current_ip_label = tk.Label(current_frame, text="", bg="white", fg="#2196F3",
                                    font=("Arial", 10, "bold"), anchor='w')
        current_ip_label.pack(side="left", fill="x", expand=True, padx=5)

        ret, cur_ip = robot.get_system_ip()
        if ret == 0 and cur_ip:
            current_ip_label.config(text=cur_ip)
        else:
            current_ip_label.config(text="Failed to get IP")

        # ---------- New IP entry ----------
        new_ip_frame = tk.Frame(config_ip_window, bg="white")
        new_ip_frame.pack(fill="x", padx=20, pady=(15, 5))
        tk.Label(new_ip_frame, text="New IP:", width=12, anchor='w', bg="white",
                 font=("Arial", 10)).pack(side="left")
        new_ip_var = tk.StringVar()
        new_ip_entry = tk.Entry(new_ip_frame, textvariable=new_ip_var, width=20,
                                font=("Arial", 10))
        new_ip_entry.pack(side="left", padx=5)
        tk.Label(new_ip_frame, text="e.g. 192.168.1.100", bg="white",
                 fg="gray", font=("Arial", 8)).pack(side="left")

        # ---------- Hint ----------
        hint_frame = tk.Frame(config_ip_window, bg="white")
        hint_frame.pack(fill="x", padx=20, pady=(5, 5))
        tk.Label(hint_frame, text="After modification, the system will restart.\n"
                                  "You will need to reconnect with the new IP.",
                 bg="white", fg="#F44336", font=("Arial", 8), anchor='w',
                 justify='left').pack(fill="x")

        # ---------- Buttons ----------
        def apply_new_ip():
            ip_str = new_ip_var.get().strip()
            if not ip_str:
                messagebox.showerror("Error", "Please enter a new IP address!")
                return

            # Validate IPv4 format
            pattern = re.compile(
                r'^((25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)\.){3}'
                r'(25[0-5]|2[0-4][0-9]|[01]?[0-9][0-9]?)$')
            if not pattern.match(ip_str):
                messagebox.showerror("IP Error",
                                     f"'{ip_str}' is not a valid IPv4 address.\n"
                                     "Please enter a valid IP, e.g. 192.168.1.100")
                return

            ip_parts = [int(x) for x in ip_str.split('.')]
            if not messagebox.askyesno("Confirm",
                                       f"Set robot IP to {ip_str}?\n\n"
                                       "The system will restart after changing the IP.\n"
                                       "You will need to reconnect manually."):
                return

            try:
                ret = robot.set_system_ip(ip_parts)
                if ret == 0:
                    messagebox.showinfo("Success",
                                        f"IP address set to {ip_str}.\n"
                                        "The system will restart.\n"
                                        "Please reconnect using the new IP after the robot restarts.")
                    # Update the main UI's IP entry field
                    self.arm_ip_entry.delete(0, tk.END)
                    self.arm_ip_entry.insert(0, ip_str)
                    config_ip_window.destroy()
                else:
                    messagebox.showerror("Failed",
                                         f"Failed to set IP address.\n"
                                         f"Error msg: {robot._get_operate_error_msg(ret)}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to set IP: {e}")

        btn_frame = tk.Frame(config_ip_window, bg="white")
        btn_frame.pack(pady=20)
        tk.Button(btn_frame, text="Apply", width=12,
                  font=("Arial", 11, "bold"), bg="#4CAF50", fg="white",
                  command=apply_new_ip).pack(side="left", padx=10)
        tk.Button(btn_frame, text="Cancel", width=12,
                  font=("Arial", 11), bg="#9E9E9E", fg="white",
                  command=config_ip_window.destroy).pack(side="left", padx=10)

        # ==================== System Time ====================
        time_title_frame = tk.Frame(config_ip_window, bg="white")
        time_title_frame.pack(fill="x", padx=5, pady=(10, 5))
        tk.Label(time_title_frame, text="System Time", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x')

        # Current robot time
        cur_time_frame = tk.Frame(config_ip_window, bg="white")
        cur_time_frame.pack(fill="x", padx=20, pady=(8, 4))
        tk.Label(cur_time_frame, text="Robot time:", width=12, anchor='w', bg="white",
                 font=("Arial", 10)).pack(side="left")
        cur_time_label = tk.Label(cur_time_frame, text="", bg="white", fg="#2196F3",
                                   font=("Arial", 10, "bold"), anchor='w')
        cur_time_label.pack(side="left", fill="x", expand=True, padx=5)

        def refresh_robot_time():
            try:
                ret, t = robot.get_system_time()
                if ret == 0 and t:
                    cur_time_label.config(text=t)
                else:
                    cur_time_label.config(text="Failed to get time")
            except Exception as e:
                cur_time_label.config(text=f"Error: {e}")

        refresh_robot_time()
        tk.Button(cur_time_frame, text="Refresh", width=8,
                  font=("Arial", 9), command=refresh_robot_time).pack(side="left", padx=5)

        # New time entry (pre-filled with local PC time)
        new_time_frame = tk.Frame(config_ip_window, bg="white")
        new_time_frame.pack(fill="x", padx=20, pady=(8, 4))

        def _local_time_tuple():
            lt = time.localtime()
            return (f"{lt.tm_year:04d}", f"{lt.tm_mon:02d}", f"{lt.tm_mday:02d}",
                    f"{lt.tm_hour:02d}", f"{lt.tm_min:02d}", f"{lt.tm_sec:02d}")

        lt = _local_time_tuple()
        year_var  = tk.StringVar(value=lt[0])
        month_var = tk.StringVar(value=lt[1])
        day_var   = tk.StringVar(value=lt[2])
        hour_var  = tk.StringVar(value=lt[3])
        min_var   = tk.StringVar(value=lt[4])
        sec_var   = tk.StringVar(value=lt[5])

        tk.Label(new_time_frame, text="New time:", width=12, anchor='w', bg="white",
                 font=("Arial", 10)).pack(side="left")
        for var, w in ((year_var, 5), (month_var, 3), (day_var, 3),
                       (hour_var, 3), (min_var, 3), (sec_var, 3)):
            tk.Entry(new_time_frame, textvariable=var, width=w,
                     font=("Arial", 10), justify="center").pack(side="left", padx=1)
        tk.Label(new_time_frame, text="Y-M-D H:M:S", bg="white",
                 fg="gray", font=("Arial", 8)).pack(side="left", padx=5)

        # Sync-from-PC + Set buttons
        time_btn_frame = tk.Frame(config_ip_window, bg="white")
        time_btn_frame.pack(pady=(4, 10))

        def sync_from_pc():
            lt2 = _local_time_tuple()
            year_var.set(lt2[0]); month_var.set(lt2[1]); day_var.set(lt2[2])
            hour_var.set(lt2[3]); min_var.set(lt2[4]); sec_var.set(lt2[5])

        def apply_new_time():
            try:
                y = int(year_var.get()); mo = int(month_var.get()); d = int(day_var.get())
                h = int(hour_var.get()); mi = int(min_var.get()); s = int(sec_var.get())
            except ValueError:
                messagebox.showerror("Error", "Time fields must be integers.")
                return
            if not messagebox.askyesno("Confirm",
                                       f"Set robot time to {y:04d}-{mo:02d}-{d:02d} {h:02d}:{mi:02d}:{s:02d}?"):
                return
            try:
                ret = robot.set_system_time(y, mo, d, h, mi, s)
                if ret == 0:
                    messagebox.showinfo("Success", "Robot time updated.")
                    refresh_robot_time()
                else:
                    messagebox.showerror("Failed",
                                         f"Failed to set time.\nError msg: {robot._get_operate_error_msg(ret)}")
            except Exception as e:
                messagebox.showerror("Error", f"Failed to set time: {e}")

        tk.Button(time_btn_frame, text="Sync from PC", width=12,
                  font=("Arial", 10), bg="#9E9E9E", fg="white",
                  command=sync_from_pc).pack(side="left", padx=10)
        tk.Button(time_btn_frame, text="Set Time", width=12,
                  font=("Arial", 10, "bold"), bg="#4CAF50", fg="white",
                  command=apply_new_time).pack(side="left", padx=10)

        # ==================== Reboot ====================
        reboot_title_frame = tk.Frame(config_ip_window, bg="white")
        reboot_title_frame.pack(fill="x", padx=5, pady=(10, 5))
        tk.Label(reboot_title_frame, text="System Reboot", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x')

        reboot_hint_frame = tk.Frame(config_ip_window, bg="white")
        reboot_hint_frame.pack(fill="x", padx=20, pady=(4, 8))
        tk.Label(reboot_hint_frame,
                 text="Rebooting will disconnect the controller.\nPlease reconnect after the robot restarts.",
                 bg="white", fg="#F44336", font=("Arial", 8), anchor='w',
                 justify='left').pack(fill="x")

        def do_reboot():
            if not messagebox.askyesno("Confirm Reboot",
                                       "Are you sure you want to reboot the robot controller?\n\n"
                                       "The connection will be lost and you must reconnect afterwards."):
                return
            try:
                ret = robot.reboot()
                if ret == 0:
                    messagebox.showinfo("Rebooting",
                                        "Robot is rebooting.\nPlease reconnect after it restarts.")
                    config_ip_window.destroy()
                else:
                    messagebox.showerror("Failed",
                                         f"Reboot failed.\nError msg: {robot._get_operate_error_msg(ret)}")
            except Exception as e:
                messagebox.showerror("Error", f"Reboot failed: {e}")

        reboot_btn_frame = tk.Frame(config_ip_window, bg="white")
        reboot_btn_frame.pack(pady=(0, 15))
        tk.Button(reboot_btn_frame, text="Reboot Robot", width=14,
                  font=("Arial", 11, "bold"), bg="#F44336", fg="white",
                  command=do_reboot).pack(side="left", padx=10)

    def system_update_dialog(self):
        # if not self.connected:
        #     messagebox.showerror('Error', "Please connect robot first!")
        #     return
        hidden_window = tk.Toplevel(self.root)
        hidden_window.title("System Upgrade")
        hidden_window.geometry("800x600")
        hidden_window.configure(bg="white")
        hidden_window.transient(self.root)
        hidden_window.resizable(True, True)
        hidden_window.grab_set()

        title_frame = tk.Frame(hidden_window, bg="white")
        title_frame.pack(fill="x", padx=5, pady=(15, 10))
        title_label = tk.Label(title_frame, text="Robot configuration file", bg="#2196F3",
                               fg="white", font=("Arial", 10, "bold"))
        title_label.pack(fill='x')

        state_a_frame = tk.Frame(hidden_window, bg="white")
        state_a_frame.pack(fill="x", pady=5)
        self.download_ini_path = tk.StringVar()
        reset_a_button = tk.Button(state_a_frame, text="Download current config", width=30,
                                   command=self.get_ini)
        reset_a_button.pack(side='left', expand=True)

        param_c_btn = tk.Button(state_a_frame, text='Configuration comparison', width=30,
                                command=self.compare_parameters_dialog, fg='#033341', bg='#DFC88C')
        param_c_btn.pack(side='left', expand=True)

        '''====system upgrade===='''
        self.system_pkg_path = tk.StringVar()
        self.config_pkg_path = tk.StringVar()

        update_frame0 = tk.Frame(hidden_window, bg="white")
        update_frame0.pack(fill="x", padx=5, pady=(25, 10))

        tk.Label(update_frame0, text="Update Operations", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x', expand=True)

        update_frame1 = tk.Frame(hidden_window, bg="white")
        update_frame1.pack(fill="x", padx=5, pady=(10, 5))

        btn1 = tk.Button(update_frame1, text="Select system upgrade package", width=40,
                         command=self.choose_system_pkg)
        btn1.pack(side='left', padx=(0, 10))

        lbl1 = tk.Label(update_frame1, textvariable=self.system_pkg_path, bg="white",
                        fg="gray", anchor="w", width=50)
        lbl1.pack(side='left', fill='x', expand=True)

        update_frame2 = tk.Frame(hidden_window, bg="white")
        update_frame2.pack(fill="x", padx=5, pady=(5, 10))

        btn2 = tk.Button(update_frame2, text="Select config file", width=40,
                         command=self.choose_config_pkg)
        btn2.pack(side='left', padx=(0, 10))

        lbl2 = tk.Label(update_frame2, textvariable=self.config_pkg_path, bg="white",
                        fg="gray", anchor="w", width=50)
        lbl2.pack(side='left', fill='x', expand=True)

        update_frame3 = tk.Frame(hidden_window, bg="white")
        update_frame3.pack(fill="x", padx=5, pady=(25, 10))

        tk.Button(update_frame3, text="Update", width=30,
                  command=self.update_sys, bg="#F6FC39",
                  fg="#151513",
                  font=("Arial", 10, "bold")).pack(side='left', expand=True)

        tips_text_frame = tk.Frame(hidden_window, bg="white")
        tips_text_frame.pack(fill="x", pady=5)
        label = tk.Label(tips_text_frame,
                         text='Configuration files and system packages can be updated individually or simultaneously. \nAfter the update, the system will restart, and the software must reconnect.',
                         bg='white')
        label.pack(padx=5, pady=10)

        '''====restore image===='''
        restore_frame0 = tk.Frame(hidden_window, bg="white")
        restore_frame0.pack(fill="x", padx=5, pady=(25, 10))

        tk.Label(restore_frame0, text="Restore Operations", bg="#2196F3",
                 fg="white", font=("Arial", 10, "bold")).pack(fill='x', expand=True)

        restore_frame1 = tk.Frame(hidden_window, bg="white")
        restore_frame1.pack(fill="x", padx=5, pady=(10, 10))

        tk.Button(restore_frame1, text="Restore Image", width=30,
                  command=self.restore_image_sys, bg="#F39639",
                  fg="#151513",
                  font=("Arial", 10, "bold")).pack(side='left', expand=True)

        restore_tips_frame = tk.Frame(hidden_window, bg="white")
        restore_tips_frame.pack(fill="x", pady=5)
        tk.Label(restore_tips_frame,
                 text='Restoring the image will overwrite the current system with the factory backup. \nAfter restoring, the system will restart, and the software must reconnect.',
                 bg='white').pack(padx=5, pady=10)

    def load_file(self, filepath):
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return f.read()
        except FileNotFoundError:
            messagebox.showerror("Error", f"file '{filepath}' doesn't exist！")
            return ""
        except Exception as e:
            messagebox.showerror("Error", f"File reading failed: {str(e)}")
            return ""

    def compare_parameters_dialog(self):
        compare_window = tk.Toplevel(self.root)
        compare_window.title("Configuration comparison update")
        compare_window.geometry("1000x900")
        compare_window.configure(bg="#f0f0f0")
        compare_window.transient(self.root)
        compare_window.resizable(True, True)
        compare_window.grab_set()

        self.template_path = ""
        self.target_path = ""
        self.template_structure = {}

        top_frame = tk.Frame(compare_window, bg="#f0f0f0")
        top_frame.pack(pady=5, fill=tk.X)

        btn_template = tk.Button(top_frame, text="Select Template File",
                                 command=lambda: self.load_template(compare_window))
        btn_template.pack(side=tk.LEFT, padx=5)

        self.template_label = tk.Label(top_frame, text="No template file selected", fg="gray", bg="#f0f0f0")
        self.template_label.pack(side=tk.LEFT, padx=5)

        btn_target = tk.Button(top_frame, text="Select Target File", command=lambda: self.load_target(compare_window))
        btn_target.pack(side=tk.LEFT, padx=5)

        self.target_label = tk.Label(top_frame, text="No target file selected", fg="gray", bg="#f0f0f0")
        self.target_label.pack(side=tk.LEFT, padx=5)

        middle_frame = tk.Frame(compare_window, bg="#f0f0f0")
        middle_frame.pack(pady=5, fill=tk.BOTH, expand=True)

        left_frame = tk.LabelFrame(middle_frame, text="Template File Content", bg="#f0f0f0")
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        self.template_text = scrolledtext.ScrolledText(left_frame, wrap=tk.NONE, font=("Consolas", 10))
        self.template_text.pack(fill=tk.BOTH, expand=True)
        self.template_text.config(state=tk.DISABLED)

        right_frame = tk.LabelFrame(middle_frame, text="Target File Content (editable)", bg="#f0f0f0")
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True, padx=5)
        self.target_text = scrolledtext.ScrolledText(right_frame, wrap=tk.NONE, font=("Consolas", 10))
        self.target_text.pack(fill=tk.BOTH, expand=True)

        bottom_frame = tk.Frame(compare_window, bg="#f0f0f0")
        bottom_frame.pack(pady=5, fill=tk.BOTH, expand=True)

        report_frame = tk.LabelFrame(bottom_frame, text="Missing Report", bg="#f0f0f0")
        report_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        self.report_text = scrolledtext.ScrolledText(report_frame, wrap=tk.WORD, height=10, font=("Segoe UI", 9))
        self.report_text.pack(fill=tk.BOTH, expand=True)
        self.report_text.config(state=tk.DISABLED)

        action_frame = tk.Frame(compare_window, bg="#f0f0f0")
        action_frame.pack(pady=5, fill=tk.X)

        btn_refresh = tk.Button(action_frame, text="Refresh Comparison",
                                command=lambda: self.refresh_comparison(compare_window))
        btn_refresh.pack(side=tk.LEFT, padx=5)

        btn_save = tk.Button(action_frame, text="Save Target As New File",
                             command=lambda: self.save_target(compare_window), bg="lightgreen")
        btn_save.pack(side=tk.LEFT, padx=5)

        btn_close = tk.Button(action_frame, text="Close", command=compare_window.destroy)
        btn_close.pack(side=tk.RIGHT, padx=5)

    def load_template(self, parent):
        file_path = filedialog.askopenfilename(
            title="Select Template File",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )
        if not file_path:
            return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            try:
                with open(file_path, "r", encoding="gbk") as f:
                    content = f.read()
            except Exception as e:
                messagebox.showerror("Error", f"Cannot read template file: {e}")
                return
        except Exception as e:
            messagebox.showerror("Error", f"Cannot read template file: {e}")
            return

        self.template_text.config(state=tk.NORMAL)
        self.template_text.delete(1.0, tk.END)
        self.template_text.insert(tk.END, content)
        self.template_text.config(state=tk.DISABLED)

        self.template_structure = self.parse_text_to_structure(content)
        self.template_path = file_path
        self.template_label.config(text=os.path.basename(file_path), fg="black")

        if self.target_text.get(1.0, tk.END).strip():
            self.refresh_comparison(parent)
        else:
            self.clear_report("Loaded template. Load target file and click Refresh Comparison.")

    def load_target(self, parent):
        file_path = filedialog.askopenfilename(
            title="Select Target File",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )
        if not file_path:
            return
        try:
            with open(file_path, "r", encoding="utf-8") as f:
                content = f.read()
        except UnicodeDecodeError:
            try:
                with open(file_path, "r", encoding="gbk") as f:
                    content = f.read()
            except Exception as e:
                messagebox.showerror("Error", f"Cannot read target file: {e}")
                return
        except Exception as e:
            messagebox.showerror("Error", f"Cannot read target file: {e}")
            return

        self.target_text.delete(1.0, tk.END)
        self.target_text.insert(tk.END, content)
        self.target_path = file_path
        self.target_label.config(text=os.path.basename(file_path), fg="black")

        if self.template_structure:
            self.refresh_comparison(parent)

    def parse_text_to_structure(self, text):
        structure = {}
        current_section = None
        lines = text.splitlines()
        for line in lines:
            line = line.strip()
            if not line:
                continue
            if line.startswith('[') and line.endswith(']'):
                current_section = line[1:-1].strip()
                if current_section not in structure:
                    structure[current_section] = set()
            elif '=' in line:
                if current_section is None:
                    continue
                key = line.split('=', 1)[0].strip()
                structure[current_section].add(key)
        return structure

    def refresh_comparison(self, parent):
        if not self.template_structure:
            self.clear_report("Please select a template file first.")
            return

        target_content = self.target_text.get(1.0, tk.END)
        target_structure = self.parse_text_to_structure(target_content)

        missing_sections, missing_keys_per_section = self.compare_structures(self.template_structure, target_structure)
        self.display_report(missing_sections, missing_keys_per_section)

    def compare_structures(self, template, target):
        missing_sections = []
        missing_keys_per_section = {}

        for section, tmpl_keys in template.items():
            if section not in target:
                missing_sections.append(section)
            else:
                target_keys = target[section]
                missing_keys = tmpl_keys - target_keys
                if missing_keys:
                    missing_keys_per_section[section] = sorted(missing_keys)

        return missing_sections, missing_keys_per_section

    def display_report(self, missing_sections, missing_keys_per_section):
        self.report_text.config(state=tk.NORMAL)
        self.report_text.delete(1.0, tk.END)

        if not missing_sections and not missing_keys_per_section:
            self.report_text.insert(tk.END, "Success! No missing sections or keys in target file.\n")
        else:
            if missing_sections:
                self.report_text.insert(tk.END, "Missing Sections:\n", "header")
                for sec in missing_sections:
                    self.report_text.insert(tk.END, f"  - [{sec}]\n", "section")
                self.report_text.insert(tk.END, "\n")

            if missing_keys_per_section:
                self.report_text.insert(tk.END, "Missing Keys in Sections:\n", "header")
                for sec, keys in missing_keys_per_section.items():
                    self.report_text.insert(tk.END, f"  Section [{sec}] missing keys:\n", "section")
                    for key in keys:
                        self.report_text.insert(tk.END, f"      - {key}\n", "key")
                    self.report_text.insert(tk.END, "\n")

            self.report_text.insert(tk.END, "Tip: Add the missing sections/keys in the right text box, then save.")

        self.report_text.tag_config("header", foreground="blue", font=("Segoe UI", 9, "bold"))
        self.report_text.tag_config("section", foreground="darkgreen", font=("Consolas", 9))
        self.report_text.tag_config("key", foreground="maroon", font=("Consolas", 9))
        self.report_text.config(state=tk.DISABLED)

    def clear_report(self, msg):
        self.report_text.config(state=tk.NORMAL)
        self.report_text.delete(1.0, tk.END)
        self.report_text.insert(tk.END, msg)
        self.report_text.config(state=tk.DISABLED)

    def save_target(self, parent):
        content = self.target_text.get(1.0, tk.END).rstrip()
        if not content.strip():
            messagebox.showwarning("Warning", "Target text box is empty. Nothing to save.")
            return

        initial_dir = os.path.dirname(self.target_path) if self.target_path else os.getcwd()
        initial_file = "modified_" + os.path.basename(self.target_path) if self.target_path else "new_config.ini"
        save_path = filedialog.asksaveasfilename(
            title="Save Target File As",
            initialdir=initial_dir,
            initialfile=initial_file,
            defaultextension=".txt",
            filetypes=[("INI files", "*.ini"), ("All files", "*.*")]
        )
        if not save_path:
            return

        try:
            with open(save_path, "w", encoding="utf-8") as f:
                f.write(content)
            messagebox.showinfo("Success", f"File saved to:\n{save_path}")
            self.target_path = save_path
            self.target_label.config(text=os.path.basename(save_path), fg="black")
        except Exception as e:
            messagebox.showerror("Save Failed", f"Cannot save file: {e}")

    def get_ini(self):
        if self.connected:
            file_path = filedialog.asksaveasfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="Save robot configuration parameter file"
            )
            if file_path:
                if robot.recv_file(file_path, "/home/FUSION/Config/cfg/robot.ini") == 0:
                    messagebox.showinfo('Success', 'Configuration file has been saved')
                    self.download_ini_path.set(file_path)
                else:
                    messagebox.showerror("Failed", "Receive file failed")

        else:
            messagebox.showerror('Error', 'Please connect robot')

    def update_ini(self):
        if self.connected:
            file_path = filedialog.askopenfilename(
                defaultextension=".ini",
                filetypes=[("ini files", "*.ini"), ("All files", "*.*")],
                title="Select robot configuration parameter file"
            )
            if file_path:
                if robot.send_file(file_path, "/home/FUSION/Config/cfg/robot.ini") == 0:
                    messagebox.showinfo('Success', 'Configuration file has been saved')
                else:
                    messagebox.showinfo('Failed', 'Configuration file download failed')
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def choose_system_pkg(self):
        self.system_file_path = filedialog.askopenfilename(
            filetypes=[("All files", "*.UPDATE")],
            title="Select system update file"
        )
        if self.system_file_path:
            self.system_pkg_path.set(self.system_file_path)

    def choose_config_pkg(self):
        self.ini_file_path = filedialog.askopenfilename(
            filetypes=[("All files", "*.ini")],
            title="Select ini update file"
        )
        if self.ini_file_path:
            self.config_pkg_path.set(self.ini_file_path)

    def update_sys(self):
        if self.connected:
            result = messagebox.askokcancel("Confirm",
                                            "After uploading the file, restarting will automatically update the system version. \nConfirm the upload?")
            if result:
                print(f"----{self.system_file_path, self.ini_file_path}")
                if not self.system_file_path and not self.ini_file_path:
                    messagebox.showerror('Error', 'Please select update package or ini file')
                    return
                if robot.system_update(self.system_file_path, self.ini_file_path) == 0:
                    messagebox.showinfo('Success',
                                        'The system files have been uploaded. Please restart the controller to update automatically.')
                else:
                    messagebox.showerror('Error', 'System file upload failed, please upload again.')
                    return
        else:
            messagebox.showerror('Error', 'Please connect robot')

    def restore_image_sys(self):
        if self.connected:
            result = messagebox.askokcancel(
                "Confirm",
                "Restoring the image will overwrite the current system with the factory backup "
                "and restart the controller.\nConfirm the restore?"
            )
            if result:
                if robot.system_restore_image() == 0:
                    messagebox.showinfo(
                        'Success',
                        'The system image is being restored. The controller will restart '
                        'automatically; please reconnect afterwards.'
                    )
                else:
                    messagebox.showerror('Error', 'System image restore failed, please try again.')
                    return
        else:
            messagebox.showerror('Error', 'Please connect robot')


if __name__ == "__main__":
    DBL_EPSILON = sys.float_info.epsilon
    arm_main_state_with = 130
    data_queue = queue.Queue()
    crr_pth = os.getcwd()
    robot = GentoRobot()
    sdk_version = robot.get_sdk_version()

    root = tk.Tk()
    style = ttk.Style()
    style.configure(
        "MyCustom.TLabelframe",
        font=("Arial", 12, "italic"),
        foreground="darkblue",
        background="white"
    )

    def _report_tk_exception(exc, val, tb):
        traceback.print_exception(exc, val, tb)

    root.report_callback_exception = _report_tk_exception

    app = App(root)
    root.mainloop()
