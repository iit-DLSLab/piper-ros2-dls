# flake8: noqa

# auto-generated DO NOT EDIT

from rcl_interfaces.msg import ParameterDescriptor
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.msg import FloatingPointRange, IntegerRange
from rclpy.clock import Clock
from rclpy.exceptions import InvalidParameterValueException
from rclpy.time import Time
import copy
import rclpy
import rclpy.parameter
from generate_parameter_library_py.python_validators import ParameterValidators



class compliance_bridge:

    class Params:
        # for detecting if the parameter struct has been updated
        stamp_ = Time()

        input_topic = "~/move_mit_joint_states"
        output_topic = "~/control/move_mit"
        joints = None
        class __Compliance:
            class __MapJoints:
                kp = 1.0
                kd = 0.5
                feedforward_torque = 0.0
            __map_type = __MapJoints
            def add_entry(self, name):
                if not hasattr(self, name):
                    setattr(self, name, self.__map_type())
                return getattr(self, name)
            def get_entry(self, name):
                return getattr(self, name)
        compliance = __Compliance()



    class ParamListener:
        def __init__(self, node, prefix=""):
            self.prefix_ = prefix
            self.params_ = compliance_bridge.Params()
            self.node_ = node
            self.logger_ = rclpy.logging.get_logger("compliance_bridge." + prefix)

            self.declare_params()

            self.node_.add_on_set_parameters_callback(self.update)
            self.user_callback = None
            self.clock_ = Clock()

        def get_params(self):
            tmp = self.params_.stamp_
            self.params_.stamp_ = None
            paramCopy = copy.deepcopy(self.params_)
            paramCopy.stamp_ = tmp
            self.params_.stamp_ = tmp
            return paramCopy

        def is_old(self, other_param):
            return self.params_.stamp_ != other_param.stamp_

        def unpack_parameter_dict(self, namespace: str, parameter_dict: dict):
            """
            Flatten a parameter dictionary recursively.

            :param namespace: The namespace to prepend to the parameter names.
            :param parameter_dict: A dictionary of parameters keyed by the parameter names
            :return: A list of rclpy Parameter objects
            """
            parameters = []
            for param_name, param_value in parameter_dict.items():
                full_param_name = namespace + param_name
                # Unroll nested parameters
                if isinstance(param_value, dict):
                    nested_params = self.unpack_parameter_dict(
                            namespace=full_param_name + rclpy.parameter.PARAMETER_SEPARATOR_STRING,
                            parameter_dict=param_value)
                    parameters.extend(nested_params)
                else:
                    parameters.append(rclpy.parameter.Parameter(full_param_name, value=param_value))
            return parameters

        def set_params_from_dict(self, param_dict):
            params_to_set = self.unpack_parameter_dict('', param_dict)
            self.update(params_to_set)

        def set_user_callback(self, callback):
            self.user_callback = callback

        def clear_user_callback(self):
            self.user_callback = None

        def refresh_dynamic_parameters(self):
            updated_params = self.get_params()
            # TODO remove any destroyed dynamic parameters

            # declare any new dynamic parameters

            for value_1 in updated_params.joints:

                updated_params.compliance.add_entry(value_1)
                entry = updated_params.compliance.get_entry(value_1)
                param_name = f"{self.prefix_}compliance.{value_1}.kp"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="MIT-mode position gain (stiffness) for this joint.", read_only = False)
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0
                    descriptor.floating_point_range[-1].to_value = float('inf')
                    parameter = entry.kp
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt_eq(param, 0)
                if validation_result:
                    raise InvalidParameterValueException('compliance.__map_joints.kp',param.value, 'Invalid value set during initialization for parameter compliance.__map_joints.kp: ' + validation_result)
                entry.kp = param.value

            for value_1 in updated_params.joints:

                updated_params.compliance.add_entry(value_1)
                entry = updated_params.compliance.get_entry(value_1)
                param_name = f"{self.prefix_}compliance.{value_1}.kd"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="MIT-mode velocity gain (damping) for this joint.", read_only = False)
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0
                    descriptor.floating_point_range[-1].to_value = float('inf')
                    parameter = entry.kd
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt_eq(param, 0)
                if validation_result:
                    raise InvalidParameterValueException('compliance.__map_joints.kd',param.value, 'Invalid value set during initialization for parameter compliance.__map_joints.kd: ' + validation_result)
                entry.kd = param.value

            for value_1 in updated_params.joints:

                updated_params.compliance.add_entry(value_1)
                entry = updated_params.compliance.get_entry(value_1)
                param_name = f"{self.prefix_}compliance.{value_1}.feedforward_torque"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="MIT-mode feed-forward torque added for this joint.", read_only = False)
                    parameter = entry.feedforward_torque
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.feedforward_torque = param.value

        def update(self, parameters):
            updated_params = self.get_params()

            for param in parameters:
                if param.name == self.prefix_ + "input_topic":
                    updated_params.input_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "output_topic":
                    updated_params.output_topic = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))

                if param.name == self.prefix_ + "joints":
                    updated_params.joints = param.value
                    self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))


            # update dynamic parameters
            for param in parameters:

                    for value_1 in updated_params.joints:

                        param_name = f"{self.prefix_}compliance.{value_1}.kp"
                        if param.name == param_name:
                            validation_result = ParameterValidators.gt_eq(param, 0)
                            if validation_result:
                                return SetParametersResult(successful=False, reason=validation_result)

                            updated_params.compliance.get_entry(value_1).kp = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))


                    for value_1 in updated_params.joints:

                        param_name = f"{self.prefix_}compliance.{value_1}.kd"
                        if param.name == param_name:
                            validation_result = ParameterValidators.gt_eq(param, 0)
                            if validation_result:
                                return SetParametersResult(successful=False, reason=validation_result)

                            updated_params.compliance.get_entry(value_1).kd = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))


                    for value_1 in updated_params.joints:

                        param_name = f"{self.prefix_}compliance.{value_1}.feedforward_torque"
                        if param.name == param_name:

                            updated_params.compliance.get_entry(value_1).feedforward_torque = param.value
                            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))


            updated_params.stamp_ = self.clock_.now()
            self.update_internal_params(updated_params)
            if self.user_callback:
                self.user_callback(self.get_params())
            return SetParametersResult(successful=True)

        def update_internal_params(self, updated_params):
            self.params_ = updated_params

        def declare_params(self):
            updated_params = self.get_params()
            # declare all parameters and give default values to non-required ones
            if not self.node_.has_parameter(self.prefix_ + "input_topic"):
                descriptor = ParameterDescriptor(description="sensor_msgs/msg/JointState topic to subscribe to.", read_only = False)
                parameter = updated_params.input_topic
                self.node_.declare_parameter(self.prefix_ + "input_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "output_topic"):
                descriptor = ParameterDescriptor(description="agx_arm_msgs/msg/MoveMITMsg topic to publish compliant commands on.", read_only = False)
                parameter = updated_params.output_topic
                self.node_.declare_parameter(self.prefix_ + "output_topic", parameter, descriptor)

            if not self.node_.has_parameter(self.prefix_ + "joints"):
                descriptor = ParameterDescriptor(description="Ordered joint names to control. A joint's position in this list is the joint_index it is published with in MoveMITMsg. Only JointState entries whose name appears here are forwarded. Required, must be set via YAML.", read_only = False)
                parameter = rclpy.Parameter.Type.STRING_ARRAY
                self.node_.declare_parameter(self.prefix_ + "joints", parameter, descriptor)

            # TODO: need validation
            # get parameters and fill struct fields
            param = self.node_.get_parameter(self.prefix_ + "input_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.input_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "output_topic")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.output_topic = param.value
            param = self.node_.get_parameter(self.prefix_ + "joints")
            self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
            updated_params.joints = param.value


            # declare and set all dynamic parameters

            for value_1 in updated_params.joints:

                updated_params.compliance.add_entry(value_1)
                entry = updated_params.compliance.get_entry(value_1)
                param_name = f"{self.prefix_}compliance.{value_1}.kp"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="MIT-mode position gain (stiffness) for this joint.", read_only = False)
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0
                    descriptor.floating_point_range[-1].to_value = float('inf')
                    parameter = entry.kp
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt_eq(param, 0)
                if validation_result:
                    raise InvalidParameterValueException('compliance.__map_joints.kp',param.value, 'Invalid value set during initialization for parameter compliance.__map_joints.kp: ' + validation_result)
                entry.kp = param.value

            for value_1 in updated_params.joints:

                updated_params.compliance.add_entry(value_1)
                entry = updated_params.compliance.get_entry(value_1)
                param_name = f"{self.prefix_}compliance.{value_1}.kd"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="MIT-mode velocity gain (damping) for this joint.", read_only = False)
                    descriptor.floating_point_range.append(FloatingPointRange())
                    descriptor.floating_point_range[-1].from_value = 0
                    descriptor.floating_point_range[-1].to_value = float('inf')
                    parameter = entry.kd
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                validation_result = ParameterValidators.gt_eq(param, 0)
                if validation_result:
                    raise InvalidParameterValueException('compliance.__map_joints.kd',param.value, 'Invalid value set during initialization for parameter compliance.__map_joints.kd: ' + validation_result)
                entry.kd = param.value

            for value_1 in updated_params.joints:

                updated_params.compliance.add_entry(value_1)
                entry = updated_params.compliance.get_entry(value_1)
                param_name = f"{self.prefix_}compliance.{value_1}.feedforward_torque"
                if not self.node_.has_parameter(self.prefix_ + param_name):
                    descriptor = ParameterDescriptor(description="MIT-mode feed-forward torque added for this joint.", read_only = False)
                    parameter = entry.feedforward_torque
                    self.node_.declare_parameter(param_name, parameter, descriptor)
                param = self.node_.get_parameter(param_name)
                self.logger_.debug(param.name + ": " + param.type_.name + " = " + str(param.value))
                entry.feedforward_torque = param.value

            self.update_internal_params(updated_params)
