.. ros:package:: rmevo_gazebo

``rmevo_gazebo``
****************

.. contents::
  :local:
  :depth: 1

***********
Description
***********

The :ros:pkg:`rmevo_gazebo` package implements the Gazebo World used by the RMEvo_Core :ros:pkg:`rmevo`.

The world is implemented in SDF, and can be found

:Links: * `Repository <https://github.com/aslab/rmevo>`_

************
Dependencies
************

:Depend: :ros:pkg:`rospy`
          :ros:pkg:`gazebo`

:Build: :ros:pkg:`message_generation`

:Build export: * :ros:pkg:`std_msgs`
                  * :ros:pkg:`std_srvs`

:Build tool: :ros:pkg:`catkin`

:Execution: :ros:pkg:`message_runtime`

*********
Interface
*********

Gazebo World Node
-----------------

The Gazebo World Node can be started running:

``roslaunch rmevo_gazebo empty_world.launch``

This command launches the script `empty_world_launch`_.

The world can be changed by modifying the file `empty_world`_ respecting the Gazebo rules
or changing the `world_name` param to target a different file.

This World includes the plugin `plugin_gazebo`_ which implements the methods needed for the evolution
of the population.

Since this Node inherits the properties of a `Gazebo`_ Node, all its methods are available too.

.. _empty_world_launch: ../../../../src/rmevo_gazebo/launch/empty_world.launch
.. _empty_world: ../../../../src/rmevo_gazebo/world/empty_world.world

.. _plugin_gazebo: ../../../../src/rmevo_gazebo/plugins/world_control.cc

.. _Gazebo : http://wiki.ros.org/gazebo_ros

Services
--------

The node starts several services that can be call through ``rosservice``.


.. function:: /worldcontrol/evaluate_fitness

  :Service description: :ros:srv:`~rmevo_gazebo/FitnessEvaluation`

  Calls the internal method :meth:`~gazebo::WorldControl.evaluateFitness()`, which evaluates the fitness of the given
  using the defined method (by default uses :meth:`~~gazebo::WorldControl.count_entity_children()`).
 
.. function:: /worldcontrol/pause_simulation_service

  :Service description: :ros:srv:`~std_srvs/Empty`

  Calls the internal method :meth:`~gazebo::WorldControl.pauseSimulation`, which pauses the simulation.

**************
Specifications
**************

Services
--------

.. ros:service:: FitnessEvaluation

  :req_param robot_id: Id of the targeted robot
  :req_paramtype robot_id: :ros:msg:`string`
  :resp_param success: Whether the evaluation succed or not
  :resp_paramtype success: :ros:msg:`bool`
  :resp_param status_message: Complementary information about the status of the evaluation.
  :resp_paramtype status_message: :ros:msg:`string`
  :resp_param robot_fitness: Fitness value calculated for the targeted robot
  :resp_paramtype robot_fitness: :ros:msg:`float32`

*******************************
World Control Internal Classes
*******************************

  .. doxygenclass:: gazebo::WorldControl
    :members:

.. toctree::
   :maxdepth: 3
   :caption: Contents

.. automodule:: pyfactory
   :members:
   :undoc-members:
   :inherited-members:
   :show-inheritance: