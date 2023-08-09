"""Support for simplified access to data on nodes of type ur5sim.ros2contactpublisher.ROS2ContactPublisher

Publish contacts of the robotic gripper 2F140
"""

import omni.graph.core as og
import omni.graph.core._omni_graph_core as _og
import omni.graph.tools.ogn as ogn
import sys
import traceback
class UR5ContactPublisherDatabase(og.Database):
    """Helper class providing simplified access to data on nodes of type ur5sim.ros2contactpublisher.ROS2ContactPublisher

    Class Members:
        node: Node being evaluated

    Attribute Value Properties:
        Inputs:
            inputs.left_contact_bool
            inputs.left_force_value
            inputs.right_contact_bool
            inputs.right_force_value
    """
    # This is an internal object that provides per-class storage of a per-node data dictionary
    PER_NODE_DATA = {}
    # This is an internal object that describes unchanging attributes in a generic way
    # The values in this list are in no particular order, as a per-attribute tuple
    #     Name, Type, ExtendedTypeIndex, UiName, Description, Metadata,
    #     Is_Required, DefaultValue, Is_Deprecated, DeprecationMsg
    # You should not need to access any of this data directly, use the defined database interfaces
    INTERFACE = og.Database._get_interface([
        ('inputs:execIn', 'execution', 0, None, 'The input execution port', {}, True, None, False, ''),
        ('inputs:left_contact_bool', 'bool', 0, 'Left Contact Bool', 'True if the mesh is in contact with another one', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:left_force_value', 'float', 0, 'Left Force Value', 'Force being applied to the left pad', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
        ('inputs:right_contact_bool', 'bool', 0, 'Right contact bool', 'True if the mesh is in contact with another one', {ogn.MetadataKeys.DEFAULT: 'false'}, True, False, False, ''),
        ('inputs:right_force_value', 'float', 0, 'Right Force Value', 'Force being applied to the right pad', {ogn.MetadataKeys.DEFAULT: '0'}, True, 0, False, ''),
    ])
    class ValuesForInputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = {"left_contact_bool", "left_force_value", "right_contact_bool", "right_force_value", "_setting_locked", "_batchedReadAttributes", "_batchedReadValues"}
        """Helper class that creates natural hierarchical access to input attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedReadAttributes = [self._attributes.left_contact_bool, self._attributes.left_force_value, self._attributes.right_contact_bool, self._attributes.right_force_value]
            self._batchedReadValues = [False, 0, False, 0]

        @property
        def left_contact_bool(self):
            return self._batchedReadValues[0]

        @left_contact_bool.setter
        def left_contact_bool(self, value):
            self._batchedReadValues[0] = value

        @property
        def left_force_value(self):
            return self._batchedReadValues[1]

        @left_force_value.setter
        def left_force_value(self, value):
            self._batchedReadValues[1] = value

        @property
        def right_contact_bool(self):
            return self._batchedReadValues[2]

        @right_contact_bool.setter
        def right_contact_bool(self, value):
            self._batchedReadValues[2] = value

        @property
        def right_force_value(self):
            return self._batchedReadValues[3]

        @right_force_value.setter
        def right_force_value(self, value):
            self._batchedReadValues[3] = value

        def __getattr__(self, item: str):
            if item in self.LOCAL_PROPERTY_NAMES:
                return object.__getattribute__(self, item)
            else:
                return super().__getattr__(item)

        def __setattr__(self, item: str, new_value):
            if item in self.LOCAL_PROPERTY_NAMES:
                object.__setattr__(self, item, new_value)
            else:
                super().__setattr__(item, new_value)

        def _prefetch(self):
            readAttributes = self._batchedReadAttributes
            newValues = _og._prefetch_input_attributes_data(readAttributes)
            if len(readAttributes) == len(newValues):
                self._batchedReadValues = newValues
    class ValuesForOutputs(og.DynamicAttributeAccess):
        LOCAL_PROPERTY_NAMES = { }
        """Helper class that creates natural hierarchical access to output attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
            self._batchedWriteValues = { }

        def _commit(self):
            _og._commit_output_attributes_data(self._batchedWriteValues)
            self._batchedWriteValues = { }
    class ValuesForState(og.DynamicAttributeAccess):
        """Helper class that creates natural hierarchical access to state attributes"""
        def __init__(self, node: og.Node, attributes, dynamic_attributes: og.DynamicAttributeInterface):
            """Initialize simplified access for the attribute data"""
            context = node.get_graph().get_default_graph_context()
            super().__init__(context, node, attributes, dynamic_attributes)
    def __init__(self, node):
        super().__init__(node)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_INPUT)
        self.inputs = UR5ContactPublisherDatabase.ValuesForInputs(node, self.attributes.inputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_OUTPUT)
        self.outputs = UR5ContactPublisherDatabase.ValuesForOutputs(node, self.attributes.outputs, dynamic_attributes)
        dynamic_attributes = self.dynamic_attribute_data(node, og.AttributePortType.ATTRIBUTE_PORT_TYPE_STATE)
        self.state = UR5ContactPublisherDatabase.ValuesForState(node, self.attributes.state, dynamic_attributes)
    class abi:
        """Class defining the ABI interface for the node type"""
        @staticmethod
        def get_node_type():
            get_node_type_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'get_node_type', None)
            if callable(get_node_type_function):
                return get_node_type_function()
            return 'ur5sim.ros2contactpublisher.ROS2ContactPublisher'
        @staticmethod
        def compute(context, node):
            try:
                per_node_data = UR5ContactPublisherDatabase.PER_NODE_DATA[node.node_id()]
                db = per_node_data.get('_db')
                if db is None:
                    db = UR5ContactPublisherDatabase(node)
                    per_node_data['_db'] = db
            except:
                db = UR5ContactPublisherDatabase(node)

            try:
                compute_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'compute', None)
                if callable(compute_function) and compute_function.__code__.co_argcount > 1:
                    return compute_function(context, node)

                db.inputs._prefetch()
                db.inputs._setting_locked = True
                with og.in_compute():
                    return UR5ContactPublisherDatabase.NODE_TYPE_CLASS.compute(db)
            except Exception as error:
                stack_trace = "".join(traceback.format_tb(sys.exc_info()[2].tb_next))
                db.log_error(f'Assertion raised in compute - {error}\n{stack_trace}', add_context=False)
            finally:
                db.inputs._setting_locked = False
                db.outputs._commit()
            return False
        @staticmethod
        def initialize(context, node):
            UR5ContactPublisherDatabase._initialize_per_node_data(node)
            initialize_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'initialize', None)
            if callable(initialize_function):
                initialize_function(context, node)
        @staticmethod
        def release(node):
            release_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'release', None)
            if callable(release_function):
                release_function(node)
            UR5ContactPublisherDatabase._release_per_node_data(node)
        @staticmethod
        def update_node_version(context, node, old_version, new_version):
            update_node_version_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'update_node_version', None)
            if callable(update_node_version_function):
                return update_node_version_function(context, node, old_version, new_version)
            return False
        @staticmethod
        def initialize_type(node_type):
            initialize_type_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'initialize_type', None)
            needs_initializing = True
            if callable(initialize_type_function):
                needs_initializing = initialize_type_function(node_type)
            if needs_initializing:
                node_type.set_metadata(ogn.MetadataKeys.EXTENSION, "ur5sim.ros2contactpublisher")
                node_type.set_metadata(ogn.MetadataKeys.UI_NAME, "ROS2ContactPublisher")
                node_type.set_metadata(ogn.MetadataKeys.DESCRIPTION, "Publish contacts of the robotic gripper 2F140")
                node_type.set_metadata(ogn.MetadataKeys.LANGUAGE, "Python")
                UR5ContactPublisherDatabase.INTERFACE.add_to_node_type(node_type)
        @staticmethod
        def on_connection_type_resolve(node):
            on_connection_type_resolve_function = getattr(UR5ContactPublisherDatabase.NODE_TYPE_CLASS, 'on_connection_type_resolve', None)
            if callable(on_connection_type_resolve_function):
                on_connection_type_resolve_function(node)
    NODE_TYPE_CLASS = None
    GENERATOR_VERSION = (1, 17, 2)
    TARGET_VERSION = (2, 65, 4)
    @staticmethod
    def register(node_type_class):
        UR5ContactPublisherDatabase.NODE_TYPE_CLASS = node_type_class
        og.register_node_type(UR5ContactPublisherDatabase.abi, 1)
    @staticmethod
    def deregister():
        og.deregister_node_type("ur5sim.ros2contactpublisher.ROS2ContactPublisher")
