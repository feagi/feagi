extends GridMap

onready var selected =  preload("res://selected.meshlib")
onready var deselected = preload("res://Cortical_area_box.meshlib")
onready var block_outline = $BlockOutline
onready var full = $MeshInstance/Area/CollisionShape

var location = Vector3()
var Gx = 0
var Gy = 0
var Gz = 0


func _on_Area_input_event(camera, event, position, normal, shape_idx):
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_LEFT and event.pressed == true:
			print("Clicked")
			location =get_used_cells()
			print(location)
			mesh_library = selected
	else:
		mesh_library = deselected
		


#func _on_Area_mouse_entered():
#	block_outline.visible = true
#	block_outline.translation = Vector3(x,y,z)
