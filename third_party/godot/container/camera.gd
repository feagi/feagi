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

var rotation_speed = PI
var _direction = Vector3(0.0, 0.0, 0.0)
var x = 47
var y = 26.323
var z = 25.711
var x_rotation = rotate_x(13.3)
var udp := PacketPeerUDP.new()
var direction = Vector3(0, 0, 0)
var velocity = Vector3(0, 0, 0)

const CAMERA_TURN_SPEED = 200

func get_input_keyboard(delta):
	# Rotate outer gimbal around y axis
	
	if Input.is_action_pressed("ui_left"):
		x = x - 1
		translation=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_right"):
		x = x + 1
		translation=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_down"):
		z = z + 1
		translation=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_up"):
		z = z - 1
		transform.origin=Vector3(x+rotation.x,(y+rotation.y),z)
	elif Input.is_action_pressed("ui_page_up"):
		y = y - 1
		transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_page_down"):
		y = y + 1
		transform.origin=Vector3(x,y,z)
	if Input.is_action_just_pressed("ui_select"): ##It's actually spacebar
		udp.connect_to_host("127.0.0.1", 20002)
		udp.put_packet(String(Godot_list.godot_list).to_utf8())
		print(Godot_list.godot_list)
	if Input.is_action_just_pressed("ui_del"):
		udp.connect_to_host("127.0.0.1", 20002)
		udp.put_packet("refresh".to_utf8())
		for key in Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"]:
			Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][key] = []
		print(Godot_list.godot_list)

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
	
