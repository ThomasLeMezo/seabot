import rospy
import rosbag
# from gmplot import gmplot
import yaml

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

# /driver/power/battery
time_battery = []
battery1 = []
battery2 = []
battery3 = []
battery4 = []

# /driver/sensor_external
time_sensor_external = []
sensor_external_pressure = []
sensor_external_temperature = []

# /driver/sensor_internal
time_sensor_internal = []
sensor_internal_pressure = []
sensor_internal_temperature = []
sensor_internal_humidity = []

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
regulation_debug_acceleration = []
regulation_debug_velocity = []
regulation_debug_depth_error = []
regulation_debug_u = []
regulation_debug_piston_set_point = []
regulation_debug_piston_set_point_offset = []

# /regulation/depth_set_point
time_regulation_depth_set_point = []
regulation_depth_set_point = []

# /regulation/depth_set_point
time_regulation_set_point = []
regulation_set_point_depth = []

# /fusion/pose
time_fusion_pose = []
fusion_pose_x = []
fusion_pose_y = []
fusion_pose_z = []

# /driver/extended_fix
time_extend_fix = []
extend_fix_status = []

# bag = rosbag.Bag('bag/2018-06-21-14-02-06.bag', 'r')
# bag = rosbag.Bag('bag/2018-06-26-15-29-06.bag', 'r')

def load_bag(filename):

	bag = rosbag.Bag(filename, 'r')

	print(bag)

	startTime = rospy.Time.from_sec(bag.get_start_time())# + rospy.Duration(600)
	end_time = rospy.Time.from_sec(bag.get_end_time())# + rospy.Duration(100)

	for topic, msg, t in bag.read_messages(start_time=startTime, end_time=end_time):
		if(topic=="/driver/piston/position"):
			time_piston_position.append((t-startTime).to_sec())
			piston_position.append(msg.position)

		elif(topic=="/driver/piston/state"):
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

		elif(topic=="/driver/sensor_external"):
			time_sensor_external.append((t-startTime).to_sec())
			sensor_external_pressure.append(msg.pressure)
			sensor_external_temperature.append(msg.temperature)

		elif(topic=="/driver/sensor_internal"):
			time_sensor_internal.append((t-startTime).to_sec())
			sensor_internal_pressure.append(msg.pressure)
			sensor_internal_temperature.append(msg.temperature)
			sensor_internal_humidity.append(msg.humidity)

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
			regulation_debug_acceleration.append(msg.acceleration)
			regulation_debug_velocity.append(msg.velocity)
			regulation_debug_depth_error.append(msg.depth_error)
			regulation_debug_u.append(msg.u)
			regulation_debug_piston_set_point.append(msg.piston_set_point)
			regulation_debug_piston_set_point_offset.append(msg.piston_set_point_offset)

		elif(topic=="/regulation/depth_set_point"):
			time_regulation_set_point.append((t-startTime).to_sec())
			regulation_set_point_depth.append(msg.depth)

		elif(topic=="/fusion/pose"):
			time_fusion_pose.append((t-startTime).to_sec())
			fusion_pose_x.append(msg.x)
			fusion_pose_y.append(msg.y)
			fusion_pose_z.append(msg.z)

		elif(topic=="/regulation/depth_set_point"):
			time_regulation_depth_set_point.append((t-startTime).to_sec())
			regulation_depth_set_point.append(msg.depth)

		elif(topic=="/driver/extended_fix"):
			time_extend_fix.append((t-startTime).to_sec())
			extend_fix_status.append(msg.status.status)

	bag.close()