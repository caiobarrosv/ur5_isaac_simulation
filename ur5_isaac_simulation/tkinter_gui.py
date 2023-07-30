"""Tkinter GUI class for the UR5 Isaac Simulation."""
import tkinter as tk
import numpy as np


class TkinterGui():
    """Tkinter GUI class for the UR5 Isaac Simulation."""

    def __init__(self, send_goal):
        self.root = tk.Tk()
        self.root.title("UR5 Isaac Simulation")
        self.root.geometry("300x450+1600+600")
        self.root.resizable(False, False)

        self.send_goal = send_goal
        self.list_of_values = []
        self.joint_values_home = [0, -1.57, 0, -1.57, 0, 0]

    def create_action_buttons_tkinter(
        self,
        buttons_lists: list
    ):
        """
        The last parameter is the frame where the button is clicked.
        """
        for button in buttons_lists:
            tk.Button(
                button[2],
                text=button[0],
                bg="white",
                fg="black",
                command=button[1],
                width=30).pack()

    def change_frame_tkinter(self, previous_frame, next_frame):
        """Change the frame in the Tkinter GUI"""
        previous_frame.pack_forget()
        next_frame.pack()

    def generate_main_frame(self, main_frame, inverse_kinematics_frame):
        """Generate the main frame in the Tkinter GUI.

        Parameters
        ----------
        main_frame : tk.Frame
            The main frame in the Tkinter GUI.
        inverse_kinematics_frame : tk.Frame
            The inverse kinematics frame in the Tkinter GUI.

        """
        desired_pose = [0.5, 0.0, 0.5, 0, 90, 0]
        main_frame_buttons = [
            ["FRONT",
             lambda: [
                 self.send_goal(desired_pose,
                                movement="slow",
                                inv_kin=True)],
             main_frame],
            ["HOME",
             lambda: [
                 self.send_goal(self.joint_values_home, movement="slow")],
             main_frame],
            ["Inv/Direct Kinematics",
             lambda: [
                 self.change_frame_tkinter(main_frame,
                                           inverse_kinematics_frame)],
             main_frame]
        ]
        self.create_action_buttons_tkinter(main_frame_buttons)

    def generate_inverse_kinematics_frame(
        self,
        main_frame,
        inverse_kinematics_frame
    ):
        """Generate the inverse kinematics frame in the Tkinter GUI.

        Parameters
        ----------
        main_frame : tk.Frame
            The main frame in the Tkinter GUI.
        inverse_kinematics_frame : tk.Frame
            The inverse kinematics frame in the Tkinter GUI.

        """
        labels = [
            "X or J1:",
            "Y or J2:",
            "Z or J3:",
            "Rx or J4:",
            "Ry or J5:",
            "Rz or J6:",
            "Solution index:"
        ]
        default_values = [0, -90, 0, -90, 0, 0, 5]
        entries = []
        tk.Label(inverse_kinematics_frame,
                 text="Values in deg and meters",
                 height=1).pack()
        for i, label in enumerate(labels):
            tk.Label(inverse_kinematics_frame,
                     text=label).pack()
            default_value = default_values[i]
            df_tk = tk.DoubleVar()
            df_tk.set(default_value)
            entry = tk.Entry(inverse_kinematics_frame,
                             textvariable=df_tk,
                             width=10)
            entries.append(entry)
            entry.pack()

        self.list_of_values = []
        inv_kin_option = tk.BooleanVar()
        checkbox = tk.Checkbutton(
            inverse_kinematics_frame,
            text="Inv Kin",
            onvalue=True,
            offvalue=False,
            variable=inv_kin_option,
            state="active")
        checkbox.pack()
        inv_kin_debug_option = tk.BooleanVar()
        checkbox2 = tk.Checkbutton(
            inverse_kinematics_frame,
            text="Debug IKin",
            onvalue=True,
            offvalue=False,
            variable=inv_kin_debug_option,
            state="active")
        checkbox2.pack()

        def get_values():
            inv_kin_option_var = inv_kin_option.get()
            inv_kin_debug_option_var = inv_kin_debug_option.get()
            self.list_of_values = []
            if not inv_kin_option_var:
                for entry in entries:
                    value = float(entry.get())
                    value = np.radians(value)
                    self.list_of_values.append(value)
            else:
                for idx, entry in enumerate(entries):
                    value = float(entry.get())
                    if idx < 3 and value > 0.85:
                        tk.messagebox.showinfo("Mensagem",
                                               "The maxium UR5 reach is 0.85m")
                        return
                    self.list_of_values.append(value)

            self.send_goal(self.list_of_values[:-1],
                           inv_kin=inv_kin_option_var,
                           movement='slow',
                           solution_index=int(self.list_of_values[-1]),
                           debug_inv_kin=inv_kin_debug_option_var)

        tk.Label(inverse_kinematics_frame, text="", height=1).pack()

        tk.Button(inverse_kinematics_frame,
                  text="Send goal",
                  command=get_values
                  ).pack(side="left")
        tk.Button(inverse_kinematics_frame,
                  text="Main Menu",
                  command=lambda: self.change_frame_tkinter(inverse_kinematics_frame,
                                                            main_frame)
                  ).pack(side="left")

    def build_frames(self):
        """Build the frames in the Tkinter GUI"""
        # Frame do botao atual
        main_frame = tk.Frame(self.root)
        main_frame.pack(side=tk.TOP)
        inverse_kinematics_frame = tk.Frame()

        self.generate_main_frame(main_frame,
                                 inverse_kinematics_frame)
        self.generate_inverse_kinematics_frame(main_frame,
                                               inverse_kinematics_frame)

        return self.root
