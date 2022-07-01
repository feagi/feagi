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
var cortical_area_name = ""
var connected = false


func _on_Area_input_event(_camera, event, _position, _normal, _shape_idx):
	if event is InputEventMouseButton and event.pressed:
		if event.button_index == BUTTON_LEFT and material == selected and event.pressed == true:
			if material == selected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				cortical_area_name = get_name().rsplit("@", true, 1)
				cortical_area_name = cortical_area_name[0].replace(" ", "")
				cortical_area_name = cortical_area_name.replace("@", "")
				cortical_area_name = "\'{s}\'".format({"s": cortical_area_name})
				for item in Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name]:
					if location == item:
						Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name].erase(item)
			material = deselected
		elif event.button_index == BUTTON_LEFT and event.pressed == true:
			if material == white:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				print(location)
				cortical_area_name = get_name().rsplit("@", true, 1)
				cortical_area_name = cortical_area_name[0].replace(" ", "")
				cortical_area_name = cortical_area_name.replace("@", "")
#				for x in range(10):
#					if cortical_area_name.find(x):
#						cortical_area_name = cortical_area_name.replace(x, "")
				cortical_area_name = "\'{s}\'".format({"s": cortical_area_name})
				print("UPDATED CORTICAL_AREA_NAME: ", cortical_area_name)
				if Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"].get(cortical_area_name):
					Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name].append(location)
				else:
					Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name] = []
					Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name].append(location)
				
			if material == deselected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z
				location = Vector3(Gx, Gy, Gz)
				cortical_area_name = get_name().lstrip("@")
				cortical_area_name = cortical_area_name.replace(" ", "")
				cortical_area_name = cortical_area_name.replace("@", "")
				for x in range(10):
					if cortical_area_name.find(x):
						cortical_area_name = cortical_area_name.replace(x, "")
				cortical_area_name = "\'{s}\'".format({"s": cortical_area_name})
				if Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"].get(cortical_area_name):
					Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name].append(location)
				else:
					Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name] = []
					Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][cortical_area_name].append(location)
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

func _input(_event):
	if Input.is_action_just_pressed("ui_del"):
		material = deselected

