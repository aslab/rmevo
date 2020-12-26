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

**************
Specifications
**************
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
Services
********

.. ros:service:: load_modules

  :req_param input_file: Path of the folder containing the modules to import into the Factory.
  :req_paramtype input_file: :ros:msg:`string`
  :resp_param ouput_message: Not asigned.
  :resp_paramtype ouput_message: :ros:msg:`string`

  This service calls the function import_modules_from_dir and imports the modules of the input folder.

.. ros:service:: list_modules

  :resp_param Output_message: Response containing the available modules.
  
      The modules are concatenated using a */n*.
    
      The modules have the form **[type, children_number]** where:

      * **type**: name of the module.
      * **children_number**: number of slots available in the module.

  :resp_paramtype Output_message: :ros:msg:`string`

  The :ros:srv:`factory_ros/list_modules` returns a string with the list of modules available.
  The modules are concatenated using a */n*.

.. ros:service:: generate_robot

  :req_param model_name: Name of the model to spawn.
  :req_paramtype model_name: :ros:msg:`string`
  :req_param model_yaml: String with yaml format with the robot configuration.
  :req_paramtype model_yaml: :ros:msg:`string`
  :resp_param success: Return true if spawnning success.
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param status_message: Feedback if available.
  :resp_paramtype status_message: :ros:msg:`string`

  This service calls the factory method *generate_sdf* to assemble all the required modules
  with the given configuration. The generated SDF is then spawned into the gazebo simulation,
  through the service :ros:srv:`rmevo_gazebo/spawn_sdf_model`.


.. toctree::
   :maxdepth: 3
   :caption: Contents

.. automodule:: pyfactory
   :members:
   :undoc-members:
   :inherited-members:
   :show-inheritance: