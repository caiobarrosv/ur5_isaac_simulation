"""Interactive marker for the UR5 robot simulation in Isaac Sim"""
import numpy as np
from geometry_msgs.msg import Point, Pose, Quaternion, Vector3
from interactive_markers import MenuHandler
from std_msgs.msg import ColorRGBA, Header
from tf_transformations import (euler_from_matrix, quaternion_from_euler,
                                quaternion_matrix)
from visualization_msgs.msg import (InteractiveMarker,
                                    InteractiveMarkerControl, Marker)


class InteractiveMarkerUR5():
    """Interactive marker for the UR5 robot simulation in Isaac Sim."""
    def __init__(self, interactive_server, send_goal,
                 transform_between_frames, ros_parameters):
        """Interactive marker for the UR5 robot simulation in Isaac Sim.

        Parameters
        ----------
        interactive_server : InteractiveMarkerServer
            The interactive marker server
        send_goal : function
            The function to send the goal to the UR5 robot
        transform_between_frames : function
            The function to transform between tf frames
        ros_parameters : dict
            The ROS parameters

        """
        self.interactive_marker_server = interactive_server
        self.send_goal = send_goal
        self.transform_between_frames = transform_between_frames

        self.menu_handler = MenuHandler()
        self.menu_handler.insert(
            title='Go to pose (Inv Kin)',
            callback=self._send_goal)
        self.menu_handler.insert(
            title='Reset position',
            callback=self.reset_interactive_marker_pose)
        self.menu_handler.insert(
            title='Set position to End-effector',
            callback=self.set_interactive_marker_pose)
        self.cube_size = ros_parameters['cube_size']
        self.interactive_marker_size =\
            ros_parameters['interactive_marker_size']

    def _get_marker_feedback(self, feedback):
        """Not implemented yet."""
        return NotImplemented

    def set_interactive_marker_pose(self, feedback):
        """Set the interactive marker pose to the end-effector pose.

        Parameters
        ----------
        feedback : InteractiveMarkerFeedback
            The feedback from the interactive marker

        """
        position_link_06, _ =\
            self.transform_between_frames("base_link_inertia", "wrist_3_link")
        x = position_link_06.transform.translation.x
        y = position_link_06.transform.translation.y
        z = position_link_06.transform.translation.z

        self.interactive_marker_server.setPose(
            "base_link_inertia",
            pose=Pose(
                position=Point(x=x, y=y, z=z),
                orientation=position_link_06.transform.rotation),
            header=Header(frame_id="base_link_inertia"))
        self.interactive_marker_server.applyChanges()

    def reset_interactive_marker_pose(self, feedback):
        """Reset the interactive marker pose to the initial position.

        Parameters
        ----------
        feedback : InteractiveMarkerFeedback
            The feedback from the interactive marker

        """
        quaternion = quaternion_from_euler(0, 0, 0)
        self.interactive_marker_server.setPose(
            "base_link_inertia",
            pose=Pose(
                position=Point(x=0., y=0., z=0.),
                orientation=Quaternion(x=quaternion[0],
                                       y=quaternion[1],
                                       z=quaternion[2],
                                       w=quaternion[3])),
            header=Header(frame_id="base_link_inertia"))
        self.interactive_marker_server.applyChanges()

    def _send_goal(self, feedback):
        """Send goal to the UR5 robot.

        Parameters
        ----------
        feedback : InteractiveMarkerFeedback
            The feedback from the interactive marker

        """
        rotation_matrix = quaternion_matrix([feedback.pose.orientation.x,
                                             feedback.pose.orientation.y,
                                             feedback.pose.orientation.z,
                                             feedback.pose.orientation.w])
        euler_angles = np.degrees(euler_from_matrix(rotation_matrix))
        target_pos = [feedback.pose.position.x,
                      feedback.pose.position.y,
                      feedback.pose.position.z,
                      euler_angles[0],
                      euler_angles[1],
                      euler_angles[2]]
        self.send_goal(target_pos,
                       movement="slow",
                       inv_kin=True,
                       debug_inv_kin=True)

    def add_interactive_maker(
        self,
        frame_id,
        child_frame_id,
        position=Point(x=0., y=0., z=0.),
        orientation=Quaternion(x=0., y=0., z=0., w=1.)
    ):
        """Add interactive marker to the scene

        Parameters
        ----------
        frame_id : str
            The frame id of the interactive marker
        child_frame_id : str
            The child frame id of the interactive marker
        position : Point
            The position of the interactive marker
        orientation : Quaternion
            The orientation of the interactive marker

        """
        self.interactive_marker_server.insert(
            marker=self._create_interactive_marker(
                position=position,
                orientation=orientation,
                frame_id=frame_id,
                child_frame_id=child_frame_id
            ),
            feedback_callback=self._get_marker_feedback
        )
        self.menu_handler.apply(
            server=self.interactive_marker_server, marker_name=child_frame_id)
        self.interactive_marker_server.applyChanges()

    def _create_interactive_marker(
        self,
        position,
        orientation,
        frame_id,
        child_frame_id
    ):
        """Create interactive marker.

        Parameters
        ----------
        position : Point
            The position of the interactive marker
        orientation : Quaternion
            The orientation of the interactive marker
        frame_id : str
            The frame id of the interactive marker
        child_frame_id : str
            The child frame id of the interactive marker

        Returns
        -------
        InteractiveMarker
            The interactive marker

        """
        return InteractiveMarker(
            header=Header(frame_id=frame_id),
            pose=Pose(
                position=position,
                orientation=orientation
            ),
            scale=self.interactive_marker_size,
            name=child_frame_id,
            description="target_pose",
            controls=[
                # Cube
                InteractiveMarkerControl(
                    always_visible=True,
                    interaction_mode=InteractiveMarkerControl.BUTTON,
                    markers=[Marker(
                        type=Marker.CUBE,
                        color=ColorRGBA(
                            r=0.0,
                            g=1.0,
                            b=0.0,
                            a=1.
                        ),
                        scale=Vector3(
                            x=self.cube_size,
                            y=self.cube_size,
                            z=self.cube_size
                        )
                    )]
                ),
                # X axis
                InteractiveMarkerControl(
                    name='x',
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    # orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.7071067811865476,
                        y=0.,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
                # Y axis
                InteractiveMarkerControl(
                    name='y',
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    # orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.,
                        z=0.7071067811865476,
                        w=0.7071067811865476
                    )
                ),
                # Z axis
                InteractiveMarkerControl(
                    name='z',
                    interaction_mode=InteractiveMarkerControl.MOVE_AXIS,
                    # orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.7071067811865476,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
                # Roll
                InteractiveMarkerControl(
                    name='roll',
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    # orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.7071067811865476,
                        y=0.,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
                # Pitch
                InteractiveMarkerControl(
                    name='pitch',
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    # orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.,
                        z=0.7071067811865476,
                        w=0.7071067811865476
                    )
                ),
                # Yaw
                InteractiveMarkerControl(
                    name='yaw',
                    interaction_mode=InteractiveMarkerControl.ROTATE_AXIS,
                    # orientation_mode=InteractiveMarkerControl.FIXED,
                    orientation=Quaternion(
                        x=0.,
                        y=0.7071067811865476,
                        z=0.,
                        w=0.7071067811865476
                    )
                ),
            ]
        )
