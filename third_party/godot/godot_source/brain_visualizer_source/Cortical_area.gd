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
extends MeshInstance3D

var location = Vector3()
var Gx = 0
var Gy = 0
var Gz = 0
var flag = false
var cortical_area_name = ""
var connected = false
var mouse_in = false
var dragging = false
var screenpos

func _on_Area_input_event(_camera, event, _position, _normal, _shape_idx):
	if event is InputEventMouseButton and event.pressed and Input.is_action_pressed("shift"):
		cortical_area_name = get_name().rsplit("@", true, 1)
		cortical_area_name = cortical_area_name[0].replace(" ", "")
		cortical_area_name = cortical_area_name.replace("@", "")
		cortical_area_name = "{s}".format({"s": cortical_area_name})
		cortical_area_name = $"..".name_to_id(cortical_area_name)
		#select_cortical.selected.append(cortical_area_name)
		if event.button_index == 1 and get_surface_override_material(0) == global_material.selected and event.pressed == true:
			if get_surface_override_material(0) == global_material.selected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z * -1
				location = Vector3(Gx, Gy, Gz)
				for item in Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name]:
					if location == item:
						Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name].erase(item)
			set_surface_override_material(0, global_material.deselected)
		elif event.button_index == 1 == true:
			if get_surface_override_material(0) == global_material.white:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z * -1
				location = Vector3(Gx, Gy, Gz)
#				print(location)
#				print("UPDATED CORTICAL_AREA_NAME G4: ", cortical_area_name)
				if Godot_list.godot_list["data"]["direct_stimulation"].get(cortical_area_name):
					Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name].append(location)
				else:
					Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name] = []
					Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name].append(location)
				
			if get_surface_override_material(0) == global_material.deselected:
				Gx = transform.origin.x
				Gy = transform.origin.y
				Gz = transform.origin.z * -1
				location = Vector3(Gx, Gy, Gz)
				cortical_area_name = get_name().lstrip("@")
				cortical_area_name = cortical_area_name.replace(" ", "")
				cortical_area_name = cortical_area_name.replace("@", "")
				for x in range(10): # 0 to 9 to remove digits in string.
					if cortical_area_name.find(str(x)):
						cortical_area_name = cortical_area_name.replace(str(x), "")
				cortical_area_name = "{s}".format({"s": cortical_area_name})
				if Godot_list.godot_list["data"]["direct_stimulation"].get(cortical_area_name):
					Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name].append(location)
				else:
					Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name] = []
					Godot_list.godot_list["data"]["direct_stimulation"][cortical_area_name].append(location)
			set_surface_override_material(0, global_material.selected)
	elif event is InputEventMouseButton and event.pressed:
		cortical_area_name = get_name().rsplit("@", true, 1)
		cortical_area_name = cortical_area_name[0].replace(" ", "")
		cortical_area_name = cortical_area_name.replace("@", "")
		cortical_area_name = "{s}".format({"s": cortical_area_name})
		select_cortical.selected.append(cortical_area_name)
func _on_Area_mouse_entered():
#	mouse_in = true
	if get_surface_override_material(0) == global_material.selected:
		set_surface_override_material(0, global_material.selected)
	else:
		set_surface_override_material(0, global_material.white)

func _on_Area_mouse_exited():
	if get_surface_override_material(0) == global_material.selected:
		set_surface_override_material(0, global_material.selected)
	else:
		set_surface_override_material(0, global_material.deselected)

func _input(_event):
	if Input.is_action_just_pressed("ui_del"):
		set_surface_override_material(0, global_material.deselected)
