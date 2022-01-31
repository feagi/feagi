# Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================
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
				udp.put_packet((text + "," + String(location)  + ",deselected").to_utf8())
			material = deselected
			#print(material)
		elif event.button_index == BUTTON_LEFT and event.pressed == true:
			#print(get_node("."))
			#print(location)
			udp.connect_to_host("127.0.0.1", 20002)
			if material == white:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				print(location)
				text = get_name().lstrip("@")
				#print("The text is: " , text)
				udp.put_packet((text + "," + String(location)).to_utf8())
			if material == deselected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				#print(location)
				#udp.put_packet((text + "," + String(location)).to_utf8())
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

func _input(event):
	if Input.is_action_just_pressed("ui_del"):
		material = deselected

