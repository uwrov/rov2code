extends Control

export var websocket_url = "ws://localhost:8002"

var _client = WebSocketClient.new()

var ready = false

var rov_orientation: Basis
var target_orientation: Basis
#var last_time = 0.0
#var time = 0.0
var power_scale := 0.5 setget set_power_scale
var hold_time := 0.0
var hold_repeat_delay := 0.1  # Seconds between repeated steps
var mode_index = 1

var mode_names = {
	1: "CW/CCW hold",
	2: "CW/CCW toggle",
	3: "Toolchanging",
}
var totalModes = mode_names.size()

var light_on = false

func _ready():
	_client.connect("connection_closed", self, "_closed")
	_client.connect("connection_error", self, "_closed")
	_client.connect("connection_established", self, "_connected")
	_client.connect("data_received", self, "_on_data")
	
	var err = _client.connect_to_url(websocket_url)
	if err != OK:
		print("Unable to connect")
		set_process(false)

func _closed(was_clean = false):
	print("Closed, clean: ", was_clean)
	set_process(false)

func _connected(proto = ""):
	print("Connected with protocol: ", proto)
	ready = true
	

var suspicous_gyro_values = 0

#var gravity_calibration_rotation = Vector3.ZERO

func basis_xform(operator: Basis, input: Basis):
	return Basis(
		operator.xform(input.x),
		operator.xform(input.y),
		operator.xform(input.z)
	)

var derivative = Vector3.ZERO
var imu_last_time = -1.0
var thrusters = []
var motors = []
var gantry = {"x" : 0.0, "y" : 0.0}
var arm_angle = 0.0

func set_power_scale(value: float) -> void:
	power_scale = clamp(value, 0.05, 1)
	$PowerBoost.text = "Power scale: %.3f" % power_scale

func _on_data():
	var data = _client.get_peer(1).get_packet().get_string_from_utf8()
	print("Got data from server: ", data)
	var parsed = JSON.parse(data).result
#	$Label.text = data
#	$Label.text = str(parsed)
# warning-ignore:unused_variable
	var acc  = parsed["accelerometer"]
	thrusters = parsed["thrusters"]
	motors = parsed["motors"]
	gantry = parsed["gantry"]
	arm_angle = parsed["arm_angle"]
	var gyro = parsed["quaternion"]
#	$Label.text = str(acc)
	# IMU: x left, y forward, z up
	# ROV: x left, y backward, z up
	# Godot: x left, y up, z forward
	if gyro[0] == null:
		$Label.text = str(gyro)
		#$LabelDebug.text = str(gyro)
		return
	# var gyrotext = "%.5f %.5f %.5f\n%.5f %.5f %.5f %.5f" % [acc[0], acc[1], acc[2], gyro[0], gyro[1], gyro[2], gyro[3]]
	# $Label.text = gyrotext
	
		# if not acc[0]:
		# 	$LabelDebug.text = gyrotext

	var prev_rov_orientation = rov_orientation
	
	# convert quaternion from IMU to basis
	rov_orientation = Basis(Quat(gyro[0], gyro[2], gyro[1], gyro[3]))

	# swap yaw and pitch
	var old_x = rov_orientation.x
	var old_y = rov_orientation.y
	rov_orientation.x = -old_y
	rov_orientation.y = -old_x
	
	# rotate to correct "up" direction
	rov_orientation = rov_orientation.rotated(Vector3(0.0, 0.0, -1.0), PI / 2)
	
	# https://stackoverflow.com/a/55718733
	# mat3x3 ros_to_unity = /* construct this by hand by mapping input axes to output axes */;
	# mat3x3 unity_to_ros = ros_to_unity.inverse();
	# quat q_ros = ...;
	# mat3x3 m_unity = ros_to_unity * mat3x3(q_ros) * unity_to_ros;
	# quat q_unity = mat_to_quat(m_unity);
	
	# x left, y forward, z up
	
#	var godot_to_gyro: Basis = Basis(Vector3(1.0, 0.0, 0.0), Vector3(0.0, 0.0, 1.0), Vector3(0.0, 1.0, 0.0))
#	var gyro_to_godot: Basis = godot_to_gyro.inverse()
#	var gyro_basis: Basis = Basis(Quat(gyro[0], gyro[1], gyro[2], gyro[3]))
##	var godot_basis = basis_xform(godot_to_gyro, basis_xform(gyro_basis, gyro_to_godot))
##	var godot_basis = basis_xform(godot_to_gyro, basis_xform(gyro_basis, gyro_to_godot))
#	var godot_basis = basis_xform(gyro_to_godot, gyro_basis)
#	rov_orientation = godot_basis
	
	
	# rov_orientation is now in y-up space
	
	
	var diff = (
		abs(prev_rov_orientation.x.angle_to(rov_orientation.x)) +
		abs(prev_rov_orientation.y.angle_to(rov_orientation.y)) + 
		abs(prev_rov_orientation.z.angle_to(rov_orientation.z))
	)
	
	var axis = (
		prev_rov_orientation.x.cross(rov_orientation.x) +
		prev_rov_orientation.y.cross(rov_orientation.y) + 
		prev_rov_orientation.z.cross(rov_orientation.z)
	).normalized()
	
	var imu_current_time = OS.get_system_time_msecs() / 1000.0
	var imu_delta = imu_current_time - imu_last_time
	imu_last_time = imu_current_time
	
	derivative = axis * diff / imu_delta
	
	$LabelDiff.text = str(diff)
	
	# TODO: for the future: what if we get a bad value on the 10th loop?
	if diff > 1.0 and suspicous_gyro_values < 10:
		rov_orientation = prev_rov_orientation
		suspicous_gyro_values += 1
		$LabelDebug2.text = "Last suspicious count: " + str(suspicous_gyro_values)
		$LabelDiff.modulate = Color.red
	else:
		suspicous_gyro_values = 0
		$LabelDiff.modulate = Color.white
	
#	if Input.is_action_pressed("calibrate_gravity"):
#		var imu_gravity = Vector3(acc[0], acc[1], acc[2])
	
#	if gravity_calibration_rotation:
#		rov_orientation = rov_orientation.rotated(gravity_calibration_rotation, gravity_calibration_rotation.length())
	
	var euler = rov_orientation.get_euler()

	$Label4.text = "Euler X: " + str(euler.x) + "\nEuler Y: " + str(euler.y) + "\nEuler Z: " + str(euler.z)
	$"%ROVProxy".transform.basis = rov_orientation


var rotation_boost_i = Vector3.ZERO

var error_integral = Vector3.ZERO

var spinPWM = 1500
var toggle_manipulator = false
func _process(delta):
	
#	time += delta
	_client.poll()
	
	var holding_increase := Input.is_action_pressed("dpad_up")
	var holding_decrease := Input.is_action_pressed("dpad_down")
	var next_mode := Input.is_action_pressed("dpad_right")
	var prev_mode := Input.is_action_pressed("dpad_left")

	if holding_increase or holding_decrease:
		hold_time -= delta
		if hold_time <= 0.0:
			if holding_increase:
				set_power_scale(power_scale * 1.1)
			elif holding_decrease:
				set_power_scale(power_scale / 1.1)
			hold_time = hold_repeat_delay
	elif next_mode or prev_mode:
		hold_time -= delta
		if hold_time <= 0.0:
			mode_index += 1 if next_mode else -1
				
			if mode_index > totalModes:
				mode_index = 1
			elif mode_index < 1:
				mode_index = totalModes
				
			hold_time = hold_repeat_delay
			
		$LabelDebug.text = "Manipulator Mode: %s (Index: %d)" % [
			mode_names.get(mode_index, "Unknown"), 
			mode_index
		]
	else:
		hold_time = 0  # Reset when not held

	
	var translation := Vector3(
		Input.get_axis("right_bumper", "left_bumper"), #strafe
		Input.get_axis("left_stick_up", "left_stick_down"), #fwd/bckwd
		Input.get_axis("left_trigger", "right_trigger") #heave
	)
	
	var rotation := Vector3(
		Input.get_axis("button_a", "button_y"), #pitch
		#Input.get_axis("right_stick_right", "right_stick_left"), #roll
		0.0, # we no longer want to roll like ever
		Input.get_axis("left_stick_right", "left_stick_left") #yaw
	)
	var override = {
		"motor_a" : 1700 if Input.is_action_pressed("motor_a") else 1500,
		"motor_b" : 1700 if Input.is_action_pressed("motor_b") else 1500,
		"motor_c" : 1700 if Input.is_action_pressed("motor_c") else 1500,
		"motor_d" : 1700 if Input.is_action_pressed("motor_d") else 1500,
		"motor_e" : 1700 if Input.is_action_pressed("motor_e") else 1500,
		"motor_f" : 1700 if Input.is_action_pressed("motor_f") else 1500,
		"motor_g" : 1700 if Input.is_action_pressed("motor_g") else 1500,
		"motor_h" : 1700 if Input.is_action_pressed("motor_h") else 1500,
		"motor_i" : 1700 if Input.is_action_pressed("motor_i") else 1500,
		"motor_j" : 1700 if Input.is_action_pressed("motor_j") else 1500
	}
	$LabelSASState.text = "SAS state: inactive"
	
	var rotation_boost = Vector3.ZERO
	
	# TODO: should this be "just pressed" ?
	if Input.is_action_pressed("save_orientation"):
		$LabelSASState.text = "SAS state: saving"
#		pass
		target_orientation = rov_orientation
	
	if Input.is_action_pressed("hold_orientation"):
		$LabelSASState.text = "SAS state: holding - "
#
		rotation_boost = Vector3.ZERO

		var x_displacement = rov_orientation.x.cross(target_orientation.x)
		var y_displacement = rov_orientation.y.cross(target_orientation.y)
		var z_displacement = rov_orientation.z.cross(target_orientation.z)

		$LabelSASState.text += "\nX ctrl: " + str(x_displacement)
		$LabelSASState.text += "\nY ctrl: " + str(y_displacement)
		$LabelSASState.text += "\nZ ctrl: " + str(z_displacement)
		
#		var temp = x_displacement
#		x_displacement = z_displacement
#		z_displacement = temp

#		x_displacement *= -1
#		y_displacement *= -1
#		z_displacement *= -1

		var error: Vector3 = x_displacement + y_displacement + z_displacement
		
		
		
		var diff = (
			abs(rov_orientation.x.angle_to(target_orientation.x)) +
			abs(rov_orientation.y.angle_to(target_orientation.y)) + 
			abs(rov_orientation.z.angle_to(target_orientation.z))
		)
		
		error = error.normalized() * diff
		error_integral += error * delta * 0.5
		error_integral.x = clamp(error_integral.x, -0.1, 0.1)
		error_integral.y = clamp(error_integral.y, -0.1, 0.1)
		error_integral.z = clamp(error_integral.z, -0.1, 0.1)
		
		
		var proportional = error * 0.1
#		var derivative = (error - last_error) * -0.1

		var integral = error_integral * 0.1 * 0.0
		
		var d = -derivative * 0.1 * 0.1
		
		var ctrl = proportional + integral + d
		
	
	var manipulator_pwm = 1500
	var x = Input.get_axis("right_stick_left", "right_stick_right")
	var y = Input.get_axis("right_stick_down", "right_stick_up")

	var v = Vector2(x, y)
	var r = v.length()

	var motor_1 = 0.0
	var motor_2 = 0.0

	if r >= 0.02:
		var mag = clamp((r - 0.02) / (1.0 - 0.02), 0.0, 1.0)
		var cmd = v / r * pow(mag, 2.2)

		motor_1 = cmd.y + cmd.x
		motor_2 = cmd.y - cmd.x

		var m = max(abs(motor_1), abs(motor_2))
		if m > 1.0:
			motor_1 /= m
			motor_2 /= m

	var left_gantry = int(round(1500 + motor_1 * 200.0))
	var right_gantry  = int(round(1500 + motor_2 * 200.0))
	
	if mode_index == 1:
		if Input.is_action_pressed("button_b"):
			manipulator_pwm -= 200
		if Input.is_action_pressed("button_x"):
			manipulator_pwm += 200
	elif mode_index == 2:
		if Input.is_action_pressed("button_b"):
			toggle_manipulator *= -1
			if toggle_manipulator:
				manipulator_pwm = 1700
		if Input.is_action_pressed("button_x"):
			toggle_manipulator *= -1
			if toggle_manipulator:
				manipulator_pwm = 1300
	elif mode_index == 3:
		if Input.is_action_pressed("button_b"):
			print("foo")
		if Input.is_action_pressed("button_x"):
			print("bar")

	if Input.is_action_pressed("light_on"):
		light_on = true
	else:
		light_on = false
	rotation.x *= pow(abs(rotation.x), 1.0)
	rotation.y *= pow(abs(rotation.y), 1.0)
	rotation.z *= pow(abs(rotation.z), 1.0)
	translation.y *= abs(pow(translation.y, 1.0))
	translation.x *= -1.0
	
	$InputLabel.text = "%s : %s" % [str(translation), str(rotation)]
	
	$"%TranslationXValue".text = str("%0.3f" % translation.x)
	$"%TranslationYValue".text = str("%0.3f" % translation.y)
	$"%TranslationZValue".text = str("%0.3f" % translation.z)
	
	$"%RotationXValue".text = str("%0.3f" % rotation.x)
	$"%RotationYValue".text = str("%0.3f" % rotation.y)
	$"%RotationZValue".text = str("%0.3f" % rotation.z)
	$"%TopValue".text = str("%0.1f" %thrusters.top)
	$"%BottomValue".text = str("%0.1f" %thrusters.bottom)
	$"%RightUpValue".text = str("%0.1f" %thrusters.right_up)
	$"%LeftUpValue".text = str("%0.1f" %thrusters.left_up)
	$"%RightFwdValue".text = str("%0.1f" %thrusters.right_back)
	$"%LeftFwdValue".text = str("%0.1f" %thrusters.left_back)
	
	$"%GantryLeftValue".text = str("%0.1f" %motors.gantry_left)
	$"%GantryRightValue".text = str("%0.1f" %motors.gantry_right)
	$"%ManipulatorValue".text = str("%0.1f" %motors.manipulator)
	$"%ArmValue".text = str("%0.1f" %motors.buoyancy_arm)
	$"%ConduitSimplified".gantry_x = gantry["x"]
	$"%ConduitSimplified".gantry_y = gantry["y"]
	$"%ConduitSimplified".arm_angle = arm_angle
	$"%ArmValue".text = str("%0.1f" %arm_angle)
	
	$ServoCurrentPWMLabel.text = str("%0.1f" %manipulator_pwm)
	
	$InputLabel.text = str(translation)
	
	if ready:
		var data = {
			"type": "control_input",
			"translation": [translation.x * 60, translation.y * 80, translation.z * 60],
			"rotation": [rotation.x, rotation.y, rotation.z * 50],
			"power_scale" : power_scale,
			"manipulator_pwm": int(manipulator_pwm),
			"left_gantry": left_gantry,
			"right_gantry": right_gantry,
			"light_on": light_on,
			"direct_motors" : $"%DirectMotorsButton".pressed,
			"override" : override}
		_client.get_peer(1).put_packet(JSON.print(data).to_ascii())
