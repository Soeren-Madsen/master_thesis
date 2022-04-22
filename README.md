Guide for running ocean wave sim in Gazebo
test
Follow installation guide here:
https://github.com/srmainwaring/asv_wave_sim

First type: beit-ws

Run this command to start an ocean wave world new
roslaunch asv_wave_sim_gazebo mavros_posix_aed.launch

To enable the waves run the next line. Parameters can be changed, these are default.
./devel/lib/asv_wave_sim_gazebo_plugins/WaveMsgPublisher   --number 3   --amplitude 0.5   --period 7   --direction 1 1   --scale 2   --angle 1   --steepness 1

Steepness til 0 og angle til 0, så har jeg rene sinus waves, hvis scale er 1 er alle 3 sinus samme størrelse.


Adding following plugin allows to enable a model to float in the ocean.

<plugin name="hydrodynamics" filename="libHydrodynamicsPlugin.so">
      <!-- Wave Model -->
      <wave_model>ocean_waves</wave_model>

      <!-- Hydrodynamics -->
      <damping_on>true</damping_on>
      <viscous_drag_on>true</viscous_drag_on>
      <pressure_drag_on>true</pressure_drag_on>

      <!-- Markers -->
      <markers>
        <update_rate>30</update_rate>
        <water_patch>false</water_patch>
        <waterline>false</waterline>
        <underwater_surface>false</underwater_surface>
      </markers>
    </plugin>


NOT NECESSARY
Spawn model:
roslaunch thesis posix_spawn.launch vehicle:=ship_thesis x:=0 y:=0 z:=0.2 ID:=0
roslaunch thesis posix_spawn.launch vehicle:=sdu_aed x:=0 y:=0 z:=1.374659 ID:=0
Spawn model on platform:
roslaunch thesis posix_spawn.launch vehicle:=sdu_aed x:=50 y:=0 z:=1 ID:=0

(Un)Pause Gazebo while spawning:
rosservice call gazebo/(un)pause_physics

Run drone:
rosrun thesis drone_control.py

Kan bruges til at spawn ocean waves, hvis det er et andet world.
rosrun gazebo_ros spawn_model -file `pwd`/src/asv_wave_sim/asv_wave_sim_gazebo/world_models/ocean_waves/model.sdf -sdf -model ocean_waves

Hector plugins til gazebo skal installeres før GPS til båden + drone virker



OPTITRACK
Skal køre commandoen:
source /opt/ros/noetic/setup.bash
før start
MAV_1_CONFIG skal ændres til telem2!!!!!!!!!!!!!!!

Screen (efterfulgt af 2x enter)
roslaunch vrpn_client_ros sample.launch server:=192.168.0.50
Screen igen
roslaunch mavros px4.launch fcu_url:="/dev/ttyUSB0:921600"
screen igen
rosrun topic_tools relay /vrpn_client_node/thesis/pose /mavros/vision_pose/pose

Husk at ændre calibration yaml filen i pose_est_board_3d.py



For at ændre landings platform. Få dronen til at hover i et par sekunder og kør så nedenstående inden man prøver at ændre på roll/pitch.

rosservice call /gazebo/set_link_state "link_state:
  link_name: 'sdu_master::landing::landing_gear'
  pose:
    position:
      x: 0.0
      y: 0.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
  twist:
    linear:
      x: 0.0
      y: 0.0
      z: 0.0
    angular:
      x: 0.0
      y: 0.0
      z: 0.0
  reference_frame: ''" 

