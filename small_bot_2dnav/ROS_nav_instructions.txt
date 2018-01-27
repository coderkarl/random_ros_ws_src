On Rpi (Robot):
sudo ntpdate VKLinux (Need to figure out how you got this working again)
    Remove (uninstall) chrony, install chrony, updated chrony.conf on both laptop and rpi
    Document the chrony.conf files (https://chrony.tuxfamily.org/faq.html#_what_is_the_minimum_recommended_configuration_for_an_ntp_client)
rosrun xv_11_laser_driver neato_laser_publisher _port:=/dev/neato_ler _firmware_version:=2
roslaunch ca_driver create_2.launch
rosrun map_server map_server floor_plan_2017a.yaml (from catkin_ws to find .yaml)

On laptop:
roslaunch small_bot_2dnav move_base.launch
roslaunch small_bot_2dnav laser_filt.launch
(roslaunch my_joystick create_joystick.launch)

rosrun rviz rviz
rosrun rqt_graph rqt_graph
rosrun tf view_frames

sudo nano /opt/ros/kinetic/share/amcl/examples/amcl_diff.launch
<launch>
<node pkg="amcl" type="amcl" name="amcl" output="screen">
  <!-- Publish scans from best pose at a max of 10 Hz -->
  <param name="odom_model_type" value="diff-corrected"/>
  <param name="odom_alpha5" value="0.1"/>
  <param name="gui_publish_rate" value="10.0"/>
  <param name="laser_max_beams" value="30"/>
  <param name="min_particles" value="200"/>
  <param name="max_particles" value="1000"/>
  <param name="kld_err" value="0.05"/>
  <param name="kld_z" value="0.99"/>
  <param name="odom_alpha1" value="1.0"/>
  <param name="odom_alpha2" value="1.0"/>
  <!-- translation std dev, m -->
  <param name="odom_alpha3" value="0.8"/>
  <param name="odom_alpha4" value="0.8"/>
  <param name="laser_z_hit" value="0.5"/>
  <param name="laser_z_short" value="0.05"/>
  <param name="laser_z_max" value="0.05"/>
  <param name="laser_z_rand" value="0.5"/>
  <param name="laser_sigma_hit" value="0.1"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_lambda_short" value="0.1"/>
  <param name="laser_model_type" value="likelihood_field"/>
  <!-- <param name="laser_model_type" value="beam"/> -->
  <param name="laser_likelihood_max_dist" value="2.0"/>
  <param name="update_min_d" value="0.0"/>
  <param name="update_min_a" value="0.0"/>
  <param name="odom_frame_id" value="odom"/>
  <param name="resample_interval" value="1"/>
  <param name="transform_tolerance" value="0.3"/>
  <param name="recovery_alpha_slow" value="0.001"/>
  <param name="recovery_alpha_fast" value="0.1"/>
  <param name="use_map_topic" value="true"/>
  <param name="first_map_only" value="true"/>
  <remap from="scan" to="scan_filtered" />
</node>
</launch>

