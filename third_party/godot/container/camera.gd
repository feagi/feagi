extends Spatial

export var forward_action = "ui_up"
export var backward_action = "ui_down"
export var left_action = "ui_left"
export var right_action = "ui_right"

var rotation_speed = PI/2
var _direction = Vector3(0.0, 0.0, 0.0)
var x = 0.312
var y = 44.943
var z = -5.133

func get_input_keyboard(delta):
	# Rotate outer gimbal around y axis

	var y_rotation = 0
	if Input.is_action_pressed("cam_right"):
		y_rotation += -1
	if Input.is_action_pressed("cam_left"):
		y_rotation += 1
	rotate_object_local(Vector3.UP, y_rotation * rotation_speed * delta)
	# Rotate inner gimbal around local x axis

	var x_rotation = 0
	if Input.is_action_pressed("cam_up"):
		x_rotation += 1
	if Input.is_action_pressed("cam_down"):
		x_rotation += -1
	$Camera.rotate_object_local(Vector3.RIGHT, x_rotation * rotation_speed * delta)
	
	if Input.is_action_pressed("ui_left"):
		x = x - 1
		$Camera.transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_right"):
		x = x + 1
		$Camera.transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_down"):
		z = z + 1
		$Camera.transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_up"):
		z = z - 1
		$Camera.transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_page_up"):
		y = y - 1
		$Camera.transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_page_down"):
		y = y + 1
		$Camera.transform.origin=Vector3(x,y,z)

func _process(delta):
	get_input_keyboard(delta)
	
