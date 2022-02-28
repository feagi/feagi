extends Spatial


const PLAYER_MOVE_SPEED = 4

onready var Camera = $Camera

var rotation_speed = PI/2
var x_rotation = rotate_x(13.3)

func get_input_keyboard(delta):
	#print(x_rotation)
	var y_rotation = 0
	if Input.is_action_pressed("cam_right"):
		y_rotation += -1
	if Input.is_action_pressed("cam_left"):
		y_rotation += 1
	set_rotation(look_leftright_rotation(y_rotation * rotation_speed * delta))


	x_rotation = 0
	if Input.is_action_pressed("cam_up"):
		x_rotation += 1
	if Input.is_action_pressed("cam_down"):
		x_rotation += -1
	rotate_object_local(Vector3.RIGHT, x_rotation * rotation_speed * delta)
	
func look_leftright_rotation(rotation = 0):
	"""
	Returns a new Vector3 which contains only the y direction
	We'll use this vector to compute the final 3D rotation later
	"""
	return get_rotation() + Vector3(0, rotation, 0)

func _process(delta):
	get_input_keyboard(delta)



#	self.velocity = self.move_and_slide_with_snap(
#		self.velocity,
#		snap_vector,
#		Vector3(0, +1, 0),
#		true
#	)
