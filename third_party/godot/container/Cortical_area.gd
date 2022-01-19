extends CSGBox

onready var white = preload("res://white.material")
onready var deselected = preload("res://cortical_area.material")
onready var selected = preload("res://selected.material")


var location = Vector3()
var Gx = 0
var Gy = 0
var Gz = 0
var flag = false


func _on_Area_input_event(camera, event, position, normal, shape_idx):
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_LEFT and material == selected and event.pressed == true:
			material = deselected
			#print(material)
		elif event.button_index == BUTTON_LEFT and event.pressed == true:
			print(get_node("."))
			print("X: ", transform.origin.x, "Y: ", transform.origin.y, "Z: ", transform.origin.z)
			material = selected
			
			


func _on_Area_mouse_entered():
	if material == selected:
		material = selected
	else:
		material = white

func _on_Area_mouse_exited():
	if material == selected:
		material = selected
	else:
		material = deselected
