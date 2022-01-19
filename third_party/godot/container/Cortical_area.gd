extends CSGMesh


var location = Vector3()
var Gx = 0
var Gy = 0
var Gz = 0


func _on_Area_input_event(camera, event, position, normal, shape_idx):
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_LEFT and event.pressed == true:
			print(get_node("."))
			print("clicked")
