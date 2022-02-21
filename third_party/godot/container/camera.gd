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

extends Spatial

export var forward_action = "ui_up"
export var backward_action = "ui_down"
export var left_action = "ui_left"
export var right_action = "ui_right"
export var spacebar = "ui_select"

var rotation_speed = PI/2
var _direction = Vector3(0.0, 0.0, 0.0)
var x = 47
var y = 26.323
var z = 25.711
var x_rotation = rotate_x(13.3)
var udp := PacketPeerUDP.new()

func get_input_keyboard(delta):
	# Rotate outer gimbal around y axis

	#print(x_rotation)
	var y_rotation = 0
	if Input.is_action_pressed("cam_right"):
		y_rotation += -1
	if Input.is_action_pressed("cam_left"):
		y_rotation += 1
	$Camera.rotate_object_local(Vector3.UP, y_rotation * rotation_speed * delta)
	# Rotate inner gimbal around local x axis

	var x_rotation = 0
	if Input.is_action_pressed("cam_up"):
		x_rotation += 1
	if Input.is_action_pressed("cam_down"):
		x_rotation += -1
	$Camera.rotate_object_local(Vector3.RIGHT, x_rotation * rotation_speed * delta)
	
	if Input.is_action_pressed("ui_left"):
		x = x - 1
		transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_right"):
		x = x + 1
		transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_down"):
		z = z + 1
		transform.origin=Vector3(x,y,z)
	elif Input.is_action_pressed("ui_up"):
		z = z - 1
		transform.origin=Vector3(x,y,z)
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
	
