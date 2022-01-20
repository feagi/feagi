extends CSGBox

onready var white = preload("res://white.material")
onready var deselected = preload("res://cortical_area.material")
onready var selected = preload("res://selected.material")


var location = Vector3()
var Gx = 0
var Gy = 0
var Gz = 0
var flag = false

var udp := PacketPeerUDP.new()
var connected = false


func _on_Area_input_event(camera, event, position, normal, shape_idx):
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_LEFT and material == selected and event.pressed == true:
			material = deselected
			#print(material)
		elif event.button_index == BUTTON_LEFT and event.pressed == true:
			print(get_node("."))
			
			Gx = transform.origin.x
			Gy = transform.origin.y
			Gz = transform.origin.z
			location = Vector3(Gx, Gy, Gz)
			#print(location)
			material = selected
			udp.connect_to_host("127.0.0.1", 20002)


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

func _process(delta):
	if !connected:
		# Try to contact server
		udp.put_packet(String(location).to_utf8())

