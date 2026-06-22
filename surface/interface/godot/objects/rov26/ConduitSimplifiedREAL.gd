extends Spatial
export var gantry_x = 0.0
export var gantry_y = 0.0
export var arm_angle = 0.0
func _ready():
	pass # Replace with function body.

func _process(delta):
	$"%Crossbar".translation.y = gantry_y
	$"%Tool".translation.x = gantry_x
	$"%Arms".rotation_degrees = Vector3(0,90,arm_angle)
