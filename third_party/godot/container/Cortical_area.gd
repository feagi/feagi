extends CSGBox

onready var white = preload("res://white.material")
onready var deselected = preload("res://cortical_area.material")
onready var selected = preload("res://selected.material")


var location = Vector3()
var Gx = 0
var Gy = 0
var Gz = 0
var flag = false
var text = ""

var udp := PacketPeerUDP.new()
var connected = false


func _on_Area_input_event(camera, event, position, normal, shape_idx):
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_LEFT and material == selected and event.pressed == true:
			if material == selected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				udp.put_packet((text + "," + String(location)).to_utf8())
			material = deselected
			#print(material)
		elif event.button_index == BUTTON_LEFT and event.pressed == true:
			print(get_node("."))
			#print(location)
			udp.connect_to_host("127.0.0.1", 20002)
			if material == white:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				text = get_name().lstrip("@")
				print("The text is: " , text)
				udp.put_packet((text + "," + String(location)).to_utf8())
			if material == deselected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				udp.put_packet((text + "," + String(location)).to_utf8())
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

