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
Its main purpouse is to parse the given input modules and generate the morphological configurations evolved by the :ros:pkg:`rmevo`.
The robots generated are sent to the :ros:pkg:`rmevo_gazebo`, which spawns them into the simulation.

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

*********
Interface
*********

Factory Node
------------

To start the Factory Node run:

``rosrun factory_ros run_factory.py``

Using the flag ``--modules``, the user can directly provide the path to the directory containing the module
library.

The modules can also be imported using the service :func:`factory_ros/load_modules`.


Services
--------

The node starts several services that can be call through ``rosservice``.


.. function:: factory_ros/load_modules

  :Service description: :ros:srv:`~factory_ros/ImportModules`

  Calls the function :meth:`~src.run_factory.FactoryNode.import_modules_from_dir` and imports the modules from the input folder.
 

.. function:: factory_ros/list_modules

  :Service description: :ros:srv:`~factory_ros/OutputString`

  Returns a string with the list of modules available. Some information of the modules is also given.
  The modules are concatenated using a */n*.

  This service provides the basic information of the modules to the :ros:node:`RMEvo_Core`.
  This information is used to generate the genotype of the population.
 

.. function:: factory_ros/generate_robot

  :Service description: :ros:srv:`~factory_ros/RobotConfiguration`

  This service calls the factory method :class:`~src.run_factory.FactoryNode.generate_robot()` to assemble all the required modules
  with the given configuration. The generated SDF is then spawned into the gazebo simulation,
  through the service :ros:srv:`rmevo_gazebo/spawn_sdf_model`. 

**************
Specifications
**************

Services
--------

.. ros:service:: ImportModules

  :req_param input_file: Path of the folder containing the modules to import into the Factory.
  :req_paramtype input_file: :ros:msg:`string`
  :resp_param ouput_message: Not asigned.
  :resp_paramtype ouput_message: :ros:msg:`string`

.. ros:service:: OutputString

  :resp_param Output_message: Response containing the available modules.
  
      The modules are concatenated using a */n*.
    
      The modules have the form **[type, children_number]** where:

      * **type**: name of the module.
      * **children_number**: number of slots available in the module.

  :resp_paramtype Output_message: :ros:msg:`string`


.. ros:service:: RobotConfiguration

  :req_param model_name: Name of the model to spawn.
  :req_paramtype model_name: :ros:msg:`string`
  :req_param model_yaml: String with yaml format with the robot configuration.
  :req_paramtype model_yaml: :ros:msg:`string`
  :resp_param success: Return true if spawnning success.
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param status_message: Feedback if available.
  :resp_paramtype status_message: :ros:msg:`string`


****************
Internal Classes
****************

  .. autoclass:: src.run_factory.FactoryNode
   :members:
   :undoc-members:
   :show-inheritance:



.. toctree::
   :maxdepth: 3
   :caption: Contents

.. automodule:: pyfactory
   :members:
   :undoc-members:
   :inherited-members:
   :show-inheritance: