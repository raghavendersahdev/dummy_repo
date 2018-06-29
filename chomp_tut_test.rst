CHOMP Planner
===============

.. image:: chomp.png
   :width: 700px

Covariant Hamiltonian optimization for motion planning (CHOMP) is a novel gradient-based trajectory optimization procedure that makes many everyday motion planning problems both simple and trainable (Ratliff et al., 2009c). While most high-dimensional motion planners separate trajectory generation into distinct planning and optimization stages, this algorithm capitalizes on covariant gradient and functional gradient approaches to the optimization stage to design a motion planning algorithm based entirely on trajectory optimization. Given an infeasible naive trajectory, CHOMP reacts to the surrounding environment to quickly pull the trajectory out of collision while simultaneously optimizing dynamical quantities such as joint velocities and accelerations. It rapidly converges to a smooth collision-free trajectory that can be executed efficiently on the robot. Integration into latest version of MoveIt! is `work in progress <https://github.com/ros-planning/moveit/issues/702>`_. `More info <http://www.nathanratliff.com/thesis-research/chomp>`_

Getting Started
---------------
If you haven't already done so, make sure you've completed the steps in `Getting Started <../getting_started/getting_started.html>`_.

You should also have gone through the steps in `Visualization with MoveIt! RViz Plugin <../quickstart_in_rviz/quickstart_in_rviz_tutorial.html>`_

Prerequisites
--------------
 1. You must have the latest version of MoveIt! installed. On ROS Kinetic you will need to build MoveIt! from source. A build from source is required as CHOMP is not officially released so ``apt-get install`` for moveIt would not be appropriate here. We will go through the steps for doing this below.
 2. To use CHOMP with your robot you must already have a MoveIt! configuration package for your robot already. For example, if you have a Panda robot, it's probably called ``panda_moveit_config``. This is typically built using the `MoveIt! Setup Assistant <../setup_assistant/setup_assistant_tutorial.html>`_.

Installing MoveIt! from Source
------------------------------
As you add and remove packages from your workspace you will need to clean your workspace and re-run the command to install new missing dependencies. Clean your workspace to remove references to the system wide installation of MoveIt!: ::

  cd ~/ws_moveit/src
  catkin clean

Now follow the instructions on the MoveIt! homepage for `installing MoveIt! Kinetic from source <http://moveit.ros.org/install/source/>`_. Note that you can skip the **Prerequisites** section since you should already have a Catkin workspace.

Re-source the setup files: ::

  source ~/ws_moveit/devel/setup.bash

Using CHOMP with Your Robot
---------------------------
**Note:** if you are following this demo using the ``panda_moveit_config`` from the `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ repository, these steps are already done for you and you can skip this section. You only need to do step 3.

#. Simply download `stomp_planning_pipeline.launch.xml <https://github.com/ros-planning/panda_moveit_config/blob/master/launch/stomp_planning_pipeline.launch.xml>`_ file into the launch directory of your MoveIt! config package. In our case, we will save this file in the ``panda_moveit_config/launch`` directory. Create a "*stomp_planning_pipeline.launch.xml*" file in the **launch** directory of your **moveit_config** package.  The file should contain the following: ::
   
   <launch>

     <!-- Stomp Plugin for MoveIt! -->
     <arg name="planning_plugin" value="stomp_moveit/StompPlannerManager" />

     <!-- The request adapters (plugins) ORDER MATTERS -->
     <arg name="planning_adapters" value="default_planner_request_adapters/FixWorkspaceBounds
                                          default_planner_request_adapters/FixStartStateBounds
                                          default_planner_request_adapters/FixStartStateCollision
                                          default_planner_request_adapters/FixStartStatePathConstraints" />

     <arg name="start_state_max_bounds_error" value="0.1" />

     <param name="planning_plugin" value="$(arg planning_plugin)" />
     <param name="request_adapters" value="$(arg planning_adapters)" />
     <param name="start_state_max_bounds_error" value="$(arg start_state_max_bounds_error)" />
     <rosparam command="load" file="$(find myworkcell_moveit_config)/config/stomp_planning.yaml"/>

   </launch>
   
     
   **!!!** Take notice of the **stomp_planning.yaml** configuration file, this file must exists in moveit_config package.

#. Adjust the line ``<rosparam command="load" file="$(find panda_moveit_config)/config/stomp_planning.yaml" />`` to ``<rosparam command="load" file="$(find <robot_moveit_config>)/config/stomp_planning.yaml" />`` replacing ``<robot_moveit_config>`` with the name of your MoveIt! configuration package.
#. Download `stomp_planning.yaml <https://github.com/ros-planning/panda_moveit_config/blob/master/config/stomp_planning.yaml>`_ file into the config directory of your MoveIt! config package. In our case, we will save this file in the ``panda_moveit_config/config`` directory. Create the "*stomp_planning.yaml*" configuration file. This file contains the parameters required by STOMP.  The parameters are specific to each ''planning group'' defined in   the SRDF file.  So if there are three planning groups, then the configuration file defines a specific set of parameters for each  planning group. In our case there is only one planning group, the "panda_arm": ::

   
   stomp/manipulator_rail:
     group_name: panda_arm
     optimization:
       num_timesteps: 60
       num_iterations: 40
       num_iterations_after_valid: 0    
       num_rollouts: 30
       max_rollouts: 30 
       initialization_method: 1 #[1 : LINEAR_INTERPOLATION, 2 : CUBIC_POLYNOMIAL, 3 : MININUM_CONTROL_COST
       control_cost_weight: 0.0
     task:
       noise_generator:
         - class: stomp_moveit/NormalDistributionSampling
           stddev: [0.05, 0.8, 1.0, 0.8, 0.4, 0.4, 0.4]
       cost_functions:
         - class: stomp_moveit/CollisionCheck
           collision_penalty: 1.0
           cost_weight: 1.0
           kernel_window_percentage: 0.2
           longest_valid_joint_move: 0.05 
       noisy_filters:
         - class: stomp_moveit/JointLimits
           lock_start: True
           lock_goal: True
         - class: stomp_moveit/MultiTrajectoryVisualization
           line_width: 0.02
           rgb: [255, 255, 0]
           marker_array_topic: stomp_trajectories
           marker_namespace: noisy
       update_filters:
         - class: stomp_moveit/PolynomialSmoother
           poly_order: 6
         - class: stomp_moveit/TrajectoryVisualization
           line_width: 0.05
           rgb: [0, 191, 255]
           error_rgb: [255, 0, 0]
           publish_intermediate: True
           marker_topic: stomp_trajectory
           marker_namespace: optimized      
     
    **!!!** *Save this file in the* **config** *directory of the moveit_config package*

#. Copy the ``demo.launch`` file to ``demo_chomp.launch``. Note that this file is also in the launch directory of your MoveIt! config package. In our case, the ``panda_moveit_config/launch`` directory.
#. Modify the **move_group.launch** file. Open the **move_group.launch** in the launch directory and change the ```pipeline``` parameter value to ```stomp``` as shown below: ::
   
       .
       .
       .
   <!-- move_group settings -->
   <arg name="allow_trajectory_execution" default="true"/>
   <arg name="fake_execution" default="false"/>
   <arg name="max_safe_path_cost" default="1"/>
   <arg name="jiggle_fraction" default="0.05" />
   <arg name="publish_monitored_planning_scene" default="true"/>

   <!-- Planning Functionality -->
   <include ns="move_group" file="$(find myworkcell_moveit_config)/launch/planning_pipeline.launch.xml">
     <arg name="pipeline" value="stomp" />
   </include>

       .
       .
       .

Running the Demo
----------------
If you have the ``panda_moveit_config`` from the `ros-planning/panda_moveit_config <https://github.com/ros-planning/panda_moveit_config>`_ repository you should be able to simply run the demo: ::

  roslaunch panda_moveit_config demo_chomp.launch

Testing CHOMP with Obstacles in the Scene
-----------------------------------------
To test CHOMP in an evironment with obstacles, you can run any of the sample python scripts (`collision_scene_test_1.py <./collision_scene_test1.py>`_ or `collision_scene_test_2.py <./collision_scene_test2.py>`_). The first scripts creates a complex scene with four ostacles. The second script creates a simple environment with one obstacle. One can change the position/size of the obstacles to change the scene. 


To run the CHOMP planner with obstacles, do the following in two seperate terminals: ::

  roslaunch panda_moveit_config demo_chomp.launch
  python collision_scene_test_1.py OR python collision_scene_test_2.py
