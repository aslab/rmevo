.. ros:package:: rmevo

``rmevo`` package
******************

.. contents::
  :local:
  :depth: 1

***********
Description
***********

The :ros:pkg:`rmevo` package contains all the method used to manage the population. In the documentation receives the name RMEvo_Core.

:Links: * `Repository <https://github.com/aslab/rmevo>`_

************
Dependencies
************

:Depend: :ros:pkg:`rospy`
          :ros:pkg:`roscpp`

:Build: :ros:pkg:`message_generation`

:Build export: * :ros:pkg:`std_msgs`
                  * :ros:pkg:`std_srvs`
                  * :ros:srv:`factory_ros/OutputString`
                  * :ros:srv:`factory_ros/RobotConfiguration`
                  * :ros:srv:`rmevo_gazebo/FitnessEvaluation`

:Build tool: :ros:pkg:`catkin`

:Execution: :ros:pkg:`message_runtime`

*********
Interface
*********

RMEvo_Core Node
-----------------

The RMevo_Core Node is started running:

``rosrun rmevo run_rmevo.py --manager *manager*``

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

This Node doesn't offer any services. However, it is important to note that
uses the services published by :ros:pkg:`rmevo_gazebo` and :ros:pkg:`factory_ros`.

*******************************
RMevo_Core Internal Classes
*******************************

.. toctree::
  :maxdepth: 5

  apidoc/pyrmevo