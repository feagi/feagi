"""
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

	http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""

extends Node3D


const PLAYER_MOVE_SPEED = 4

@onready var camera3d = $Camera3D

var rotation_speed = PI/2
var x_rotation = Vector3(13.3, 0.0, 0.0)
var is_not_typing = true

func get_input_keyboard(delta):
	if Godot_list.Node_2D_control:
		is_not_typing = false
	else:
		is_not_typing = true
	var y_rotation = 0
	if Input.is_action_pressed("cam_right") and is_not_typing:
		y_rotation += -1
	if Input.is_action_pressed("cam_left") and is_not_typing:
		y_rotation += 1
	set_rotation(look_leftright_rotation(y_rotation * rotation_speed * delta))
	if Input.is_action_just_pressed("reset") and is_not_typing:
		set_rotation(Vector3(13.3, 0, 0))


	x_rotation = 0
	if Input.is_action_pressed("cam_up") and is_not_typing:
		x_rotation += 1
	if Input.is_action_pressed("cam_down") and is_not_typing:
		x_rotation += -1
	rotate_object_local(Vector3.RIGHT, x_rotation * rotation_speed * delta)
	
func look_leftright_rotation(rotation_input = 0):
	"""
	Returns a new Vector3 which contains only the y direction
	We'll use this vector to compute the final 3D rotation later
	"""
	return get_rotation() + Vector3(0, rotation_input, 0)

func _process(delta):
	get_input_keyboard(delta)

