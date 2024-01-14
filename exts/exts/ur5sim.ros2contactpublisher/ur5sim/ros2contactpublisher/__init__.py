
import omni.ext
import carb
import os, sys
import rclpy

# Any class derived from `omni.ext.IExt` in a top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when the extension is enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() will be called.
class Ur5simRos2contactpublisherExtension(omni.ext.IExt):
    # ext_id is the current extension id. It can be used with the extension manager to query additional information,
    # such as where this extension is located in the filesystem.
    def on_startup(self, ext_id):
        print("[ur5sim.ros2contactpublisher] Ur5simRos2contactpublisherExtension startup", flush=True)

        ament_prefix = os.environ.get("AMENT_PREFIX_PATH")
        if ament_prefix is not None and os.environ.get("OLD_PYTHONPATH") is not None:
            for python_path in os.environ.get("OLD_PYTHONPATH").split(":"):
                for ament_path in ament_prefix.split(":"):
                    if python_path.startswith(os.path.abspath(ament_path) + os.sep):
                        sys.path.append(os.path.join(python_path))
                        break

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = ext_manager.get_extension_path(ext_id)
        try:
            import rclpy

            rclpy.init()
            rclpy.shutdown()
            carb.log_warn("rclpy loaded")
        except Exception as e:
            carb.log_error(f"Could not import internal rclpy: {e}")
            carb.log_error(
                f"To use the internal libraries included with the extension please set: \nRMW_IMPLEMENTATION=rmw_fastrtps_cpp\nLD_LIBRARY_PATH=$LD_LIBRARY_PATH:{self._extension_path}/humble/lib\nBefore starting Isaac Sim"
            )

    def on_shutdown(self):
        print("[ur5sim.ros2contactpublisher] Ur5simRos2contactpublisherExtension shutdown", flush=True)
