import rospy
import rosbag
# from gmplot import gmplot
import yaml
import numpy as np

# /driver/piston/position
time_piston_position = []
piston_position = []

# /driver/piston/state
time_piston_state = []
piston_state_position = []
piston_state_position = []
piston_state_switch_out = []
piston_state_switch_in = []
piston_state_state = []
piston_state_motor_on = []
piston_state_enable_on = []
piston_state_position_set_point = []
piston_state_motor_speed = []

# /driver/piston/velocity
time_piston_velocity = []
piston_velocity = []

# /driver/power/battery
time_battery = []
battery1 = []
battery2 = []
battery3 = []
battery4 = []

# /fusion/battery
time_fusion_battery = []
fusion_battery1 = []
fusion_battery2 = []
fusion_battery3 = []
fusion_battery4 = []

# /driver/sensor_external
time_sensor_external = []
sensor_external_pressure = []
sensor_external_temperature = []

# /driver/sensor_internal
time_sensor_internal = []
sensor_internal_pressure = []
sensor_internal_temperature = []
sensor_internal_humidity = []

# /fusion/sensor_internal
time_fusion_sensor_internal = []
sensor_fusion_internal_pressure = []
sensor_fusion_internal_temperature = []

# /driver/thruster/engine
time_engine = []
engine_left = []
engine_right = []

# /fusion/depth
time_fusion_depth = []
fusion_depth = []
fusion_velocity = []

# /regulation/debug
time_regulation_debug = []
regulation_velocity_error = []
regulation_depth_error = []
regulation_vector_field_target = []
regulation_u = []
regulation_piston_set_point = []
regulation_piston_set_point_offset = []
regulation_antiwindup = []

# /regulation/depth_set_point
time_regulation_depth_set_point = []
regulation_depth_set_point = []

# /fusion/pose
time_fusion_pose = []
fusion_pose_north = []
fusion_pose_east = []

# /driver/extended_fix
time_fix = []
fix_status = []

# /driver/mag
time_mag = []
mag_x = []
mag_y = []
mag_z = []

# /safety/safety
time_safety = []
safety_published_frequency = []
safety_depth_limit = []
safety_batteries_limit = []
safety_depressurization = []

# /safety/debug
time_safety_debug = []
safety_debug_flash = []
safety_debug_ratio_p_t = []
safety_debug_ratio_delta = []
safety_debug_volume = []
safety_debug_volume_delta = []

# /driver/euler
time_euler = []
euler_x = []
euler_y = []
euler_z = []

# /fusion/kalman
time_kalman = []
kalman_depth = []
kalman_volume = []
kalman_velocity = []
kalman_offset = []
kalman_error_velocity = []

def load_bag(filename):

	bag = rosbag.Bag(filename, 'r')

	print(bag)

	startTime = rospy.Time.from_sec(bag.get_start_time())# + rospy.Duration(600)
	end_time = rospy.Time.from_sec(bag.get_end_time())# + rospy.Duration(100)

	nb_piston_position = bag.get_message_count('/driver/piston/position')

	for topic, msg, t in bag.read_messages(start_time=startTime, end_time=end_time):
		if(topic=="/driver/piston/position"):
			if(len(time_piston_position)>0):
				time_piston_position.append((t-startTime).to_sec())
				piston_position.append(piston_position[-1])	
			time_piston_position.append((t-startTime).to_sec())
			piston_position.append(msg.position)

		elif(topic=="/driver/piston/state"):
			if(len(time_piston_state)>0):
				time_piston_state.append((t-startTime).to_sec())
				piston_state_position.append(msg.position)
				piston_state_switch_out.append(piston_state_switch_out[-1])
				piston_state_switch_in.append(piston_state_switch_in[-1])
				piston_state_state.append(piston_state_state[-1])
				piston_state_motor_on.append(piston_state_motor_on[-1])
				piston_state_enable_on.append(piston_state_enable_on[-1])
				piston_state_position_set_point.append(piston_state_position_set_point[-1])
				piston_state_motor_speed.append(piston_state_motor_speed[-1])

			time_piston_state.append((t-startTime).to_sec())
			piston_state_position.append(msg.position)
			piston_state_switch_out.append(msg.switch_out)
			piston_state_switch_in.append(msg.switch_in)
			piston_state_state.append(msg.state)
			piston_state_motor_on.append(msg.motor_on)
			piston_state_enable_on.append(msg.enable_on)
			piston_state_position_set_point.append(msg.position_set_point)
			piston_state_motor_speed.append(msg.motor_speed)

		elif(topic=="/driver/power/battery"):
			time_battery.append((t-startTime).to_sec())
			battery1.append(msg.battery1)
			battery2.append(msg.battery2)
			battery3.append(msg.battery3)
			battery4.append(msg.battery4)

		elif(topic=="/fusion/battery"):
			time_fusion_battery.append((t-startTime).to_sec())
			fusion_battery1.append(msg.battery1)
			fusion_battery2.append(msg.battery2)
			fusion_battery3.append(msg.battery3)
			fusion_battery4.append(msg.battery4)

		elif(topic=="/driver/sensor_external"):
			time_sensor_external.append((t-startTime).to_sec())
			sensor_external_pressure.append(msg.pressure)
			sensor_external_temperature.append(msg.temperature)

		elif(topic=="/driver/sensor_internal"):
			time_sensor_internal.append((t-startTime).to_sec())
			sensor_internal_pressure.append(msg.pressure)
			sensor_internal_temperature.append(msg.temperature)
			sensor_internal_humidity.append(msg.humidity)

		elif(topic=="/fusion/sensor_internal"):
			time_fusion_sensor_internal.append((t-startTime).to_sec())
			sensor_fusion_internal_pressure.append(msg.pressure)
			sensor_fusion_internal_temperature.append(msg.temperature)

		elif(topic=="/driver/thruster/engine"):
			time_engine.append((t-startTime).to_sec())
			engine_left.append(msg.left)
			engine_right.append(msg.right)

		elif(topic=="/fusion/depth"):
			time_fusion_depth.append((t-startTime).to_sec())
			fusion_depth.append(msg.depth)
			fusion_velocity.append(msg.velocity)

		elif(topic=="/regulation/debug"):
			time_regulation_debug.append((t-startTime).to_sec())
			# regulation_velocity_error.append(msg.velocity_error)
			# regulation_depth_error.append(msg.depth_error)
			# regulation_vector_field_target.append(msg.vector_field_target)
			regulation_u.append(msg.u)
			regulation_piston_set_point.append(msg.piston_set_point)
			# regulation_piston_set_point_offset.append(msg.piston_set_point_offset)
			# if(msg.antiwindup):
			# 	regulation_antiwindup.append(1)
			# else:
			# 	regulation_antiwindup.append(0)

		elif(topic=="/fusion/pose"):
			time_fusion_pose.append((t-startTime).to_sec())
			fusion_pose_east.append(msg.east)
			fusion_pose_north.append(msg.north)

		elif(topic=="/regulation/depth_set_point"):
			if(len(regulation_depth_set_point)>0):
				time_regulation_depth_set_point.append((t-startTime).to_sec())
				regulation_depth_set_point.append(regulation_depth_set_point[-1])
			time_regulation_depth_set_point.append((t-startTime).to_sec())
			regulation_depth_set_point.append(msg.depth)

		elif(topic=="/driver/fix"):
			time_fix.append((t-startTime).to_sec())
			fix_status.append(msg.status.status)

		elif(topic=="/driver/piston/velocity"):
			time_piston_velocity.append((t-startTime).to_sec())
			piston_velocity.append(msg.velocity)

		elif(topic=="/driver/mag"):
			time_mag.append((t-startTime).to_sec())
			mag_x.append(msg.magnetic_field.x)
			mag_y.append(msg.magnetic_field.y)
			mag_z.append(msg.magnetic_field.z)

		elif(topic=="/driver/euler"):
			time_euler.append((t-startTime).to_sec())
			euler_x.append(msg.x)
			euler_y.append(msg.y)
			euler_z.append(msg.z)

		elif(topic=="/fusion/kalman"):
			time_kalman.append((t-startTime).to_sec())
			kalman_depth.append(msg.depth)
			kalman_volume.append(msg.volume)
			kalman_velocity.append(msg.velocity)
			kalman_offset.append(msg.offset)
			kalman_error_velocity.append(msg.covariance[6])

		elif(topic=="/safety/safety"):
			time_safety.append((t-startTime).to_sec())
			if(msg.published_frequency):
				safety_published_frequency.append(1)
			else:
				safety_published_frequency.append(0)
			
			if(msg.depth_limit):
				safety_depth_limit.append(1)
			else:
				safety_depth_limit.append(0)
			if(msg.batteries_limit):
				safety_batteries_limit.append(1)
			else:
				safety_batteries_limit.append(0)
			if(msg.depressurization):
				safety_depressurization.append(1)
			else:
				safety_depressurization.append(0)

		elif(topic=="/safety/debug"):
			time_safety_debug.append((t-startTime).to_sec())
			if(msg.flash):
				safety_debug_flash.append(1)
			else:
				safety_debug_flash.append(0)
			safety_debug_ratio_p_t.append(msg.ratio_p_t)
			safety_debug_ratio_delta.append(msg.ratio_delta)
			safety_debug_volume.append(msg.volume)
			safety_debug_volume_delta.append(msg.volume_delta)

	bag.close()

	if(len(time_regulation_depth_set_point)>0):
		time_regulation_depth_set_point.append((end_time-startTime).to_sec())
		regulation_depth_set_point.append(regulation_depth_set_point[-1])