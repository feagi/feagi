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

extends Camera

export var forward_action = "ui_up"
export var backward_action = "ui_down"
export var left_action = "ui_left"
export var right_action = "ui_right"
export var spacebar = "ui_select"
export var reset = "reset"

var rotation_speed = PI
var _direction = Vector3(0.0, 0.0, 0.0)
var x = 47
var y = 26.323
var z = 25.711
var x_rotation = rotate_x(13.3)
var direction = Vector3(0, 0, 0)
var velocity = Vector3(0, 0, 0)
var flagged = false ## This allows space and del to be able to send data without being overwritten by spam "{}"
var is_not_typing = true

const CAMERA_TURN_SPEED = 200

func get_input_keyboard(_delta):
	# Rotate outer gimbal around y axis
	
	if Input.is_action_pressed("ui_left") and is_not_typing:
		x = x - 1
		translation=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_right") and is_not_typing:
		x = x + 1
		translation=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_down") and is_not_typing:
		z = z + 1
		translation=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_up") and is_not_typing:
		z = z - 1
		transform.origin=Vector3(x+rotation.x,(y+rotation.y),z)
	elif Input.is_action_pressed("ui_page_up") and is_not_typing:
		y = y - 1
		transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_page_down") and is_not_typing:
		y = y + 1
		transform.origin=Vector3(x,y,z)
	if Input.is_action_just_pressed("ui_select") and is_not_typing: ##It's actually spacebar
		flagged = true
		websocket.send(String(Godot_list.godot_list))
		print(Godot_list.godot_list)
	if Input.is_action_just_pressed("ui_del"):
		#print(Input.is_action_just_pressed("ui_del"))
		flagged = true
		websocket.send("refresh")
		for key in Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"]:
			Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][key] = []
		print(Godot_list.godot_list)
	if Input.is_action_just_pressed("reset"):
		x = 18
		y = 43.323002
		z = 6.711
		transform.origin=Vector3(x,y,z)
#		rotation_degrees = Vector3(-1.09, 0, 0)
	if flagged != true:
		websocket.send("{}")
	elif flagged:
		flagged = false
func _process(delta):
	get_input_keyboard(delta)
	
func look_leftright_rotation(rotation = 0):
	"""
	Returns a new Vector3 which contains only the y direction
	We'll use this vector to compute the final 3D rotation later
	"""
	return get_rotation() + Vector3(0, rotation, 0)
	
func move_forward_back(in_direction: int):
	"""
	Move the camera forward or backwards
	"""
	direction.z += in_direction
	velocity += get_transform().basis.z * in_direction * rotation_speed

func move_left_right(in_direction: int):
	"""
	Move the camera to the left or right
	"""
	direction.x += in_direction
	velocity += get_transform().basis.x * in_direction * rotation_speed
	


func _on_LineEdit_text_changed(new_text):
	is_not_typing = false

func _on_LineEdit_focus_exited():
	is_not_typing = true
