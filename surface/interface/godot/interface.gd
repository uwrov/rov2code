extends Control

export var websocket_url = "ws://localhost:8002"

var _client = WebSocketClient.new()

var ready = false

var rov_orientation: Basis
var target_orientation: Basis
var power_scale := 0.5 setget set_power_scale
var hold_time := 0.0
var hold_repeat_delay := 0.1  # Seconds between repeated steps
var mode_index = 1

var mode_names = {
	1: "fast",
	2: "slow",
	3: "yaw"
}
var totalModes = mode_names.size()

func _ready():
	_client.connect("connection_closed", self, "_closed")
	_client.connect("connection_error", self, "_closed")
	_client.connect("connection_established", self, "_connected")
	_client.connect("data_received", self, "_on_data")
	OS.window_position = Vector2(0, 0)
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

var derivative = Vector3.ZERO
var imu_last_time = -1.0
var thrusters = []
var motors = []
var gantry = {"x" : 0.0, "y" : 0.0}
var arm_angle = 0.0

func set_power_scale(value: float) -> void:
	power_scale = clamp(value, 0.05, 1)
	$PowerBoost.text = "Power scale: %.3f" % power_scale

# Converts a gantry motor command (-1 to 1) to PWM
func gantry_pwm(motor: float) -> int:
	var deadband = 20
	var delta = motor * 140.0
	if delta > 5.0:
		delta += deadband
	elif delta < -5.0:
		delta -= deadband
	return int(round(1500 + delta))

func _on_data():
	var data = _client.get_peer(1).get_packet().get_string_from_utf8()
	var parsed = JSON.parse(data).result
# warning-ignore:unused_variable
	var acc  = parsed["accelerometer"]
	thrusters = parsed["thrusters"]
	motors = parsed["motors"]
	gantry = parsed["gantry"]
	arm_angle = parsed["arm_angle"]
	var gyro = parsed["quaternion"]
	# IMU: x left, y forward, z up
	# ROV: x left, y backward, z up
	# Godot: x left, y up, z forward
	if gyro and gyro[0] == null:
		$Label.text = str(gyro)
		return

	var prev_rov_orientation = rov_orientation
	
	# convert quaternion from IMU to basis
	if gyro:
		rov_orientation = Basis(Quat(gyro[0], gyro[2], gyro[1], gyro[3]))

	# swap yaw and pitch
	var old_x = rov_orientation.x
	var old_y = rov_orientation.y
	rov_orientation.x = -old_y
	rov_orientation.y = -old_x
	
	# rotate to correct "up" direction
	rov_orientation = rov_orientation.rotated(Vector3(0.0, 0.0, -1.0), PI / 2)
	
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
	
	var euler = rov_orientation.get_euler()

	$Label4.text = "Euler X: " + str(euler.x) + "\nEuler Y: " + str(euler.y) + "\nEuler Z: " + str(euler.z)
	$"%ROVProxy".transform.basis = rov_orientation


var error_integral = Vector3.ZERO

var captureFrame = false
var depthHold = false
func _process(delta):
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
		if hold_time <= -0.25:
			mode_index += 1 if next_mode else -1
				
			if mode_index > totalModes:
				mode_index = 1
			elif mode_index < 1:
				mode_index = totalModes
				
			hold_time = hold_repeat_delay
			
		$LabelDebug.text = "Mode: %s (Index: %d)" % [
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
		0.0, # we no longer want to roll like ever
		Input.get_axis("left_stick_right", "left_stick_left") #yaw
	)
	var override = {}
	for letter in "abcdefghij":
		var action = "motor_" + letter
		override[action] = 1700 if Input.is_action_pressed(action) else 1500
	$LabelSASState.text = "SAS state: inactive"
	
	var rotation_boost = Vector3.ZERO
	
	# TODO: should this be "just pressed" ?
	if Input.is_action_pressed("save_orientation"):
		$LabelSASState.text = "SAS state: saving"
		target_orientation = rov_orientation
	
	if Input.is_action_pressed("hold_orientation"):
		$LabelSASState.text = "SAS state: holding - "
		rotation_boost = Vector3.ZERO

		var x_displacement = rov_orientation.x.cross(target_orientation.x)
		var y_displacement = rov_orientation.y.cross(target_orientation.y)
		var z_displacement = rov_orientation.z.cross(target_orientation.z)

		$LabelSASState.text += "\nX ctrl: " + str(x_displacement)
		$LabelSASState.text += "\nY ctrl: " + str(y_displacement)
		$LabelSASState.text += "\nZ ctrl: " + str(z_displacement)
		
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

	if r >= 0.005:
		var mag = clamp((r - 0.005) / (1.0 - 0.005), 0.0, 1.0)
		var cmd = v / r * pow(mag, 3)

		motor_1 = cmd.x - cmd.y
		motor_2 = cmd.y + cmd.x

		var m = max(abs(motor_1), abs(motor_2))
		if m > 1.0:
			motor_1 /= m
			motor_2 /= m
	var left_gantry = gantry_pwm(motor_1)
	var right_gantry = gantry_pwm(motor_2)
	
	if mode_index == 1:
		if Input.is_action_pressed("button_b"):
			manipulator_pwm -= 150
		if Input.is_action_pressed("button_x"):
			manipulator_pwm += 150
	elif mode_index == 2:
		if Input.is_action_pressed("button_b"):
			manipulator_pwm -= 75
		if Input.is_action_pressed("button_x"):
			manipulator_pwm += 75
	elif mode_index == 3:
		if Input.is_action_pressed("button_b"):
			#yaw right
			rotation.z = -.2
		if Input.is_action_pressed("button_x"):
			#yaw left
			rotation.z = .2
	if Input.is_action_pressed("Capture_frame"):
		captureFrame = true
		print(captureFrame)
	else:
		captureFrame = false
	if Input.is_action_just_pressed("depth_hold"):
		depthHold = !depthHold
		print("Depth Hold: ", depthHold)
	rotation *= rotation.abs()
	translation.y *= abs(translation.y)
	translation.x *= -1.0
	
	$"%TranslationXValue".text = str("%0.3f" % translation.x)
	$"%TranslationYValue".text = str("%0.3f" % translation.y)
	$"%TranslationZValue".text = str("%0.3f" % translation.z)
	
	$"%RotationXValue".text = str("%0.3f" % rotation.x)
	$"%RotationYValue".text = str("%0.3f" % rotation.y)
	$"%RotationZValue".text = str("%0.3f" % rotation.z)
	if thrusters:
		$"%TopValue".text = str("%0.1f" %thrusters.top)
		$"%BottomValue".text = str("%0.1f" %thrusters.bottom)
		$"%RightUpValue".text = str("%0.1f" %thrusters.right_up)
		$"%LeftUpValue".text = str("%0.1f" %thrusters.left_up)
		$"%RightFwdValue".text = str("%0.1f" %thrusters.right_back)
		$"%LeftFwdValue".text = str("%0.1f" %thrusters.left_back)
	
	if motors:
		$"%GantryLeftValue".text = str("%0.1f" %motors.gantry_left)
		$"%GantryRightValue".text = str("%0.1f" %motors.gantry_right)
		$"%ManipulatorValue".text = str("%0.1f" %motors.manipulator)
		$"%ArmValue".text = str("%0.1f" %motors.buoyancy_arm)

	if gantry:
		$"%ConduitSimplified".gantry_x = gantry["x"]
		$"%ConduitSimplified".gantry_y = gantry["y"]

	if arm_angle:
		$"%ConduitSimplified".arm_angle = arm_angle
		$"%ArmValue".text = str("%0.1f" %arm_angle)
		
	$ServoCurrentPWMLabel.text = str("%0.1f" %manipulator_pwm)
	
	$InputLabel.text = str(translation)
	if ready:
		var data = {
			"type": "control_input",
			"translation": [-translation.x * 60, -translation.y * 80, -translation.z * 60],
			"rotation": [rotation.x, rotation.y, rotation.z * 50],
			"power_scale" : power_scale,
			"manipulator_pwm": int(manipulator_pwm),
			"left_gantry": left_gantry,
			"right_gantry": right_gantry,
			"capture_frame" : captureFrame,
			"depth_hold" : depthHold,
			"direct_motors" : $"%DirectMotorsButton".pressed,
			"override" : override}
		_client.get_peer(1).put_packet(JSON.print(data).to_ascii())
