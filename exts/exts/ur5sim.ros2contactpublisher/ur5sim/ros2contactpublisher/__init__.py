
import omni.ext
import carb

# Any class derived from `omni.ext.IExt` in a top level module (defined in `python.modules` of `extension.toml`) will be
# instantiated when the extension is enabled and `on_startup(ext_id)` will be called. Later when extension gets disabled
# on_shutdown() will be called.
class Ur5simRos2contactpublisherExtension(omni.ext.IExt):
    # ext_id is the current extension id. It can be used with the extension manager to query additional information,
    # such as where this extension is located in the filesystem.
    def on_startup(self, ext_id):
        print("[ur5sim.ros2contactpublisher] Ur5simRos2contactpublisherExtension startup", flush=True)

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = ext_manager.get_extension_path(ext_id)
        try:
            import rclpy

            rclpy.init()
            rclpy.shutdown()
            carb.log_warn("rclpy loaded successfully from isaacsim.ros2.bridge")
        except Exception as e:
            carb.log_error(f"Could not import rclpy from isaacsim.ros2.bridge: {e}")
            carb.log_error(
                "Make sure 'isaacsim.ros2.bridge' extension is enabled and loaded before this extension."
            )

    def on_shutdown(self):
        print("[ur5sim.ros2contactpublisher] Ur5simRos2contactpublisherExtension shutdown", flush=True)
