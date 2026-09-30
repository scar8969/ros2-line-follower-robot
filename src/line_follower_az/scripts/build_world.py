#!/usr/bin/env python3
"""build_world.py — assemble worlds/line_track.world from the procedural track
generator (generate_track.py) plus the embedded robot model.

The original repo shipped a hand-assembled world file. Here the world is
BUILT from code: run this script to regenerate worlds/line_track.world after
changing track geometry (STRAIGHT_LEN / TURN_RADIUS in generate_track.py).

Run with: python3 scripts/build_world.py
"""

import os
import sys

# generate_track.py lives in the same scripts/ directory.
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import generate_track as gt  # noqa: E402

WORLD_HEAD = '''<?xml version="1.0" ?>
<sdf version="1.9">
  <world name="line_follower_world">

    <physics name="1ms" type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-contact-system" name="gz::sim::systems::Contact"></plugin>

    <gravity>0 0 -9.8</gravity>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.9 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.2 -0.9</direction>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>8 6</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>8 6</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.75 0.75 0.75 1</ambient>
            <diffuse>0.75 0.75 0.75 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ====== painted line track (procedurally generated) ====== -->
    <!-- The base slab sits at z=0.0..0.005 and the line segments sit at
         z=0.006..0.008 (see generate_track.py), so nothing z-fights. -->
    <model name="line_track">
      <static>true</static>
      <link name="track_link">
        <visual name="track_visual">
          <geometry>
            <box>
              <size>3.2 1.2 0.005</size>
            </box>
          </geometry>
          <material>
            <ambient>0.9 0.9 0.9 1</ambient>
            <diffuse>0.9 0.9 0.9 1</diffuse>
          </material>
        </visual>
'''

WORLD_MID = '''
      </link>
    </model>

    <!-- ====== robot model (embedded, no spawn step needed) ====== -->
    <model name="line_follower">
      <pose>-0.9 0.6 0.05 0 0 0</pose>

      <link name="base_link">
        <inertial>
          <mass>1.0</mass>
          <inertia>
            <ixx>0.002</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>0.003</iyy><iyz>0.0</iyz><izz>0.004</izz>
          </inertia>
        </inertial>
        <visual name="chassis_vis">
          <geometry><box><size>0.18 0.13 0.04</size></box></geometry>
          <material>
            <ambient>0.2 0.3 0.8 1</ambient>
            <diffuse>0.2 0.3 0.8 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
        <collision name="chassis_col">
          <geometry><box><size>0.18 0.13 0.04</size></box></geometry>
        </collision>
      </link>

      <link name="left_wheel">
        <pose>0 0.075 -0.015 1.5708 0 0</pose>
        <inertial>
          <mass>0.05</mass>
          <inertia>
            <ixx>0.00002</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>0.00002</iyy><iyz>0.0</iyz><izz>0.00003</izz>
          </inertia>
        </inertial>
        <visual name="lw_vis">
          <geometry><cylinder><radius>0.033</radius><length>0.02</length></cylinder></geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="lw_col">
          <geometry><cylinder><radius>0.033</radius><length>0.02</length></cylinder></geometry>
        </collision>
      </link>

      <joint name="left_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>left_wheel</child>
        <axis><xyz>0 1 0</xyz></axis>
      </joint>

      <link name="right_wheel">
        <pose>0 -0.075 -0.015 1.5708 0 0</pose>
        <inertial>
          <mass>0.05</mass>
          <inertia>
            <ixx>0.00002</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>0.00002</iyy><iyz>0.0</iyz><izz>0.00003</izz>
          </inertia>
        </inertial>
        <visual name="rw_vis">
          <geometry><cylinder><radius>0.033</radius><length>0.02</length></cylinder></geometry>
          <material>
            <ambient>0.1 0.1 0.1 1</ambient>
            <diffuse>0.1 0.1 0.1 1</diffuse>
          </material>
        </visual>
        <collision name="rw_col">
          <geometry><cylinder><radius>0.033</radius><length>0.02</length></cylinder></geometry>
        </collision>
      </link>

      <joint name="right_wheel_joint" type="revolute">
        <parent>base_link</parent>
        <child>right_wheel</child>
        <axis><xyz>0 1 0</xyz></axis>
      </joint>

      <link name="caster_wheel">
        <pose>0.08 0 -0.033 0 0 0</pose>
        <inertial>
          <mass>0.02</mass>
          <inertia>
            <ixx>0.000004</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>0.000004</iyy><iyz>0.0</iyz><izz>0.000004</izz>
          </inertia>
        </inertial>
        <visual name="caster_vis">
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
          <material>
            <ambient>0.3 0.3 0.3 1</ambient>
            <diffuse>0.3 0.3 0.3 1</diffuse>
          </material>
        </visual>
        <collision name="caster_col">
          <geometry><sphere><radius>0.015</radius></sphere></geometry>
        </collision>
      </link>

      <joint name="caster_joint" type="fixed">
        <parent>base_link</parent>
        <child>caster_wheel</child>
      </joint>

      <link name="camera_link">
        <pose>0.09 0 0 0 0.95 0</pose>
        <inertial>
          <mass>0.01</mass>
          <inertia>
            <ixx>0.000001</ixx><ixy>0.0</ixy><ixz>0.0</ixz>
            <iyy>0.000001</iyy><iyz>0.0</iyz><izz>0.000001</izz>
          </inertia>
        </inertial>
        <visual name="camera_vis">
          <geometry><box><size>0.02 0.04 0.02</size></box></geometry>
          <material><ambient>0.8 0.1 0.1 1</ambient><diffuse>0.8 0.1 0.1 1</diffuse></material>
        </visual>
        <sensor name="line_camera" type="camera">
          <topic>/camera</topic>
          <update_rate>30</update_rate>
          <camera>
            <horizontal_fov>1.05</horizontal_fov>
            <image><width>320</width><height>240</height><format>R8G8B8</format></image>
            <clip><near>0.02</near><far>3.0</far></clip>
          </camera>
          <always_on>true</always_on>
          <visualize>true</visualize>
        </sensor>
      </link>

      <joint name="camera_joint" type="fixed">
        <parent>base_link</parent>
        <child>camera_link</child>
      </joint>

      <plugin filename="gz-sim-diff-drive-system"
              name="gz::sim::systems::DiffDrive">
        <left_joint>left_wheel_joint</left_joint>
        <right_joint>right_wheel_joint</right_joint>
        <wheel_separation>0.15</wheel_separation>
        <wheel_radius>0.033</wheel_radius>
        <max_linear_acceleration>3</max_linear_acceleration>
        <topic>/model/line_follower/cmd_vel</topic>
        <odom_topic>/model/line_follower/odometry</odom_topic>
        <tf_topic>/model/line_follower/tf</tf_topic>
        <frame_id>odom</frame_id>
        <child_frame_id>base_link</child_frame_id>
        <odom_publish_frequency>30</odom_publish_frequency>
      </plugin>

      <plugin filename="gz-sim-joint-state-publisher-system"
              name="gz::sim::systems::JointStatePublisher">
      </plugin>

      <plugin filename="gz-sim-pose-publisher-system"
              name="gz::sim::systems::PosePublisher">
        <publish_link_pose>true</publish_link_pose>
        <use_pose_vector_msg>true</use_pose_vector_msg>
        <static_publisher>true</static_publisher>
        <static_update_frequency>1</static_update_frequency>
      </plugin>
    </model>

  </world>
</sdf>
'''


def main():
    # Regenerate the track visuals.
    track_visuals = gt.build_track_visuals()

    world = WORLD_HEAD + '\n'.join(track_visuals) + WORLD_MID

    out_dir = os.path.join(os.path.dirname(__file__), '..', 'worlds')
    os.makedirs(out_dir, exist_ok=True)
    out_path = os.path.abspath(os.path.join(out_dir, 'line_track.world'))
    with open(out_path, 'w', newline='\n') as f:
        f.write(world)

    total_len = 2 * gt.STRAIGHT_LEN + 2 * 3.141592653589793 * gt.TURN_RADIUS
    print(f"wrote {out_path}")
    print(f"track: straight={gt.STRAIGHT_LEN}m radius={gt.TURN_RADIUS}m, "
          f"{len(track_visuals)} visual segments, path length ~{total_len:.2f} m")


if __name__ == '__main__':
    main()
