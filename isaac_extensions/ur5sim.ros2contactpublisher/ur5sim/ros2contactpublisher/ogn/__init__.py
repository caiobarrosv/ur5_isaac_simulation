
"""
Dynamically import every file in a directory tree that looks like a Python Ogn Node.
This includes linked directories, which is the mechanism by which nodes can be hot-reloaded from the source tree.
"""
import omni.graph.core as og
og.register_ogn_nodes(__file__, "ur5sim.ros2contactpublisher")
