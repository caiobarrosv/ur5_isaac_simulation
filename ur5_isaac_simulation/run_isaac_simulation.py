#!/usr/bin/env python3
import os
import carb
import omni
from omni.isaac.kit import SimulationApp
from definitions import ROOT_DIR


class URIsaacExtension():
    def __init__(self):
        usd_name = 'ur5_isaac_sim.usd'
        project_folder = os.path.join(ROOT_DIR, "usd")
        self.robot_path = '/World/ur5_robot'
        self.usd_path = os.path.join(project_folder, usd_name)

        CONFIG = {
            "width": 1920,
            "height": 1080,
            "sync_loads": True,
            "headless": False,
            "renderer": "RayTracedLighting",
            # "open_usd": os.path.join(project_folder, usd_name)
        }

        self.simulation_app = SimulationApp(launch_config=CONFIG)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        ext_manager.add_path(
            project_folder, omni.ext.ExtensionPathType.COLLECTION_USER)

        # Wait two frames so that stage starts loading
        self.simulation_app.update()
        self.simulation_app.update()

    @staticmethod
    def hide_ui_menu():
        # workslace must be imported after SimulationApp initialization
        from omni.ui import Workspace
        windows = Workspace.get_windows()
        ui_visible = ["Main ToolBar", "Viewport", "DockSpace"]
        for window in windows:
            window_name = window.title
            if window_name not in ui_visible:
                Workspace.show_window(window_name, False)

    @staticmethod
    def enable_extensions():
        # extensions must be imported after SimulationApp initialization
        from omni.isaac.core.utils.extensions import enable_extension
        # enable_extension("omni.isaac.ros2_bridge")
        carb.log_warn("Loading extensions...")
        enable_extension("omni.isaac.ros2_bridge-humble")
        enable_extension("omni.kit.tool.measure")

    @staticmethod
    def set_fullscreen():
        """Set the viewport to fullscreen"""
        appwindow = omni.appwindow.get_default_app_window()
        was_fullscreen = appwindow.is_fullscreen()
        appwindow.set_fullscreen(not was_fullscreen)

    @staticmethod
    def configure_camera(camera_path: str) -> None:
        """Link a camera to the viewport create at the beginning"""
        from omni.kit.viewport.utility import get_active_viewport
        viewport_api = get_active_viewport()
        camera_path = "/World/Camera"
        viewport_api.set_active_camera(camera_path)

    def open_stage(self):
        """Open the stage file"""
        omni.usd.get_context().open_stage(str(self.usd_path))

    def run_simulation(self):
        """Run the Isaac Sim ssimulation"""
        from omni.isaac.sensor import _sensor
        self._cs = _sensor.acquire_contact_sensor_interface()
        sensors = [
            "/robotiq_arg2f_140_model/left_inner_finger_pad/left_inner_finger_pad_contact"]

        while self.simulation_app.is_running():
            # Run in realtime mode, we don't specify the step size
            self.simulation_app.update()
            for sensor in sensors:
                reading = self._cs.get_sensor_readings(
                    (self.robot_path + sensor))

                if reading.shape[0]:
                    for r1 in reading:
                        left_finger_in_collision = bool(r1["inContact"])

        self.simulation_app.close()


def main():
    ur_extension = URIsaacExtension()
    ur_extension.open_stage()
    # ur_extension.hide_ui_menu()
    # ur_extension.set_fullscreen()
    # ur_extension.configure_camera("/World/Camera")
    ur_extension.enable_extensions()
    ur_extension.run_simulation()


if __name__ == "__main__":
    main()
