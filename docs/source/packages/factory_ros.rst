.. ros:package:: factory_ros

``factory_ros``
***************

.. contents::
  :local:
  :depth: 1

***********
Description
***********

The :ros:pkg:`factory_ros` package implements the RMEvo Factory.
Its main purpouse is to parse the given input modules and generate the morphological configurations evolved by the :ros:pkg`rmevo`.
The robots generated are sent to the :ros:pkg`rmevo_gazebo`, which spawns them into the simulation.

:Links: * `Repository <https://github.com/aslab/rmevo>`_

************
Dependencies
************

:Depend: :ros:pkg:`rospy`

:Build: :ros:pkg:`message_generation`

:Build export: * :ros:pkg:`std_msgs`
                  * :ros:pkg:`std_srvs`

:Build tool: :ros:pkg:`catkin`

:Execution: :ros:pkg:`message_runtime`


********
Messages
********

.. ros:message:: Foo

  :msg_param header: Header of the message.
  :msg_paramtype header: :ros:msg:`Header`
  :msg_param pose: The 3D pose of the foo that is detected.
  :msg_paramtype pose: :ros:msg:`geometry_msgs/Pose`
  :msg_param color: The color of the foo.
  :msg_paramtype color: :ros:msg:`string`


********
Services
********

.. ros:service:: ImportModules

  :req_param input_file: Path of the folder containing the modules to import into the Factory.
  :req_paramtype input_file: :ros:msg:`string`
  :resp_param ouput_message: Not asigned.
  :resp_paramtype ouput_message: :ros:msg:`string`

  This service calls the function import_modules_from_dir and imports the modules of the input folder.

.. ros:service:: OutputString

  :resp_param Output_message: Answer to a query asked to the node.
  :resp_paramtype Output_message: :ros:msg:`string`

  This service calls the function import_modules_from_dir and imports the modules of the input folder.

.. ros:service:: ImportModules

  :req_param input_file: Path of the folder containing the modules to import into the Factory.
  :req_paramtype input_file: :ros:msg:`string`
  :resp_param ouput_message: Not asigned.
  :resp_paramtype ouput_message: :ros:msg:`string`

  This service calls the function import_modules_from_dir and imports the modules of the input folder.



********
Actions
********

.. ros:action:: load_modules

  :goal_param setpoint: The setpoint to reach.
  :goal_paramtype setpoint: :ros:msg:`geometry_msgs/Point`
  :result_param steady_state_error: Error between achieved point and setpoint.
  :result_paramtype steady_state_error: :ros:msg:`geometry_msgs/Point`
  :feedback_param tracking_error: Error between ideal trajectory and current
                                  trajectory.
  :feedback_paramtype tracking_error: :ros:msg:`geometry_msgs/Point`
  :feedback_param power: Current power usage per joint.
  :feedback_paramtype power: :ros:msg:`float32[]`


.. toctree::
   :maxdepth: 3
   :caption: Contents

.. automodule:: pyfactory
   :members:
   :undoc-members:
   :inherited-members:
   :show-inheritance: