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

extends Camera3D

@export var forward_action = "ui_up"
@export var backward_action = "ui_down"
@export var left_action = "ui_left"
@export var right_action = "ui_right"
@export var spacebar = "ui_select"
@export var reset = "reset"

var rotation_speed = PI

var x = transform.origin.x
var y = transform.origin.y
var z = transform.origin.z
var x_rotation = Vector3(13.3, 0.0, 0.0)
var direction = Vector3(0, 0, 0)
var velocity = Vector3(0, 0, 0)
var flagged = false ## This allows space and del to be able to send data without being overwritten by spam "{}"
var is_not_typing = true
var cortical_pointer = ""

const CAMERA_TURN_SPEED = 200

func get_input_keyboard(_delta):
	if Godot_list.Node_2D_control:
		is_not_typing = false
	else:
		is_not_typing = true
	# Rotate outer gimbal around y axis
	if Input.is_action_pressed("ui_left") and is_not_typing:
		x = x - 1
		position=Vector3(x,y,z)
	if Input.is_action_pressed("ui_right") and is_not_typing:
		x = x + 1
		position=Vector3(x,y,z)
	if Input.is_action_pressed("ui_down") and is_not_typing:
		z = z + 1
		position=Vector3(x,y,z)
	if Input.is_action_pressed("ui_up") and is_not_typing:
		z = z - 1
		transform.origin=Vector3(x+rotation.x,(y+rotation.y),z)
	if Input.is_action_pressed("ui_page_up") and is_not_typing:
		y = y - 1
		transform.origin=Vector3(x,y,z)
	if Input.is_action_pressed("ui_page_down") and is_not_typing:
		y = y + 1
		transform.origin=Vector3(x,y,z)
	if Input.is_action_just_pressed("ui_select") and is_not_typing: ##It's actually spacebar
		flagged = true
		network_setting.send(str(Godot_list.godot_list))
		print(Godot_list.godot_list)
	if Input.is_action_just_pressed("ui_del"):
		print(Input.is_action_just_pressed("ui_del"))
		$".."/".."/".."/".."/".."/Menu/Mapping_Properties.visible = false
		$".."/".."/".."/".."/".."/Menu/cortical_menu.visible = false
		$".."/".."/".."/".."/".."/Menu/insert_menu.visible = false
		$".."/".."/".."/".."/".."/Menu/cortical_mapping.visible = false
		$".."/".."/".."/".."/".."/Menu/cortical_menu.visible = false
		$".."/".."/".."/".."/".."/Menu/properties.visible = false
		$".."/".."/".."/".."/".."/Menu/button_choice.visible = false
		$".."/".."/".."/".."/".."/Menu/collapse_1.visible = false
		$".."/".."/".."/".."/".."/Menu/collapse_2.visible = false
		$".."/".."/".."/".."/".."/Menu/collapse_3.visible = false
		$".."/".."/".."/".."/".."/Menu/collapse_4.visible = false
		$".."/".."/".."/".."/".."/Menu/close_for_all.visible = false
		
		flagged = true
		network_setting.send("refresh")
		for key in Godot_list.godot_list["data"]["direct_stimulation"]:
			Godot_list.godot_list["data"]["direct_stimulation"][key] = []
		print(Godot_list.godot_list)
	if Input.is_action_just_pressed("reset"):
		x = 0
		y = 0
		z = 0
		transform.origin=Vector3(x,y,z)
#		rotation_degrees = Vector3(-1.09, 0, 0)
	if flagged != true:
		network_setting.send("{}")
	elif flagged:
		flagged = false
func _process(delta):
	get_input_keyboard(delta)

func look_leftright_rotation(rotation_input = 0):
	"""
	Returns a new Vector3 which contains only the y direction
	We'll use this vector to compute the final 3D rotation later
	"""
	return get_rotation() + Vector3(0, rotation_input, 0)
	
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

func _on_grab_location_of_cortical_request_completed(_result, _response_code, _headers, body):
		# Incomplete
		var test_json_conv = JSON.new()
		test_json_conv.parse(body.get_string_from_utf8())
		var api_data = test_json_conv.get_data()
		x = api_data[0]
		y = api_data[1]
		z = api_data[2]
		if api_data[0] > 0:
			x = x - 50
		else:
			x = x + 50
		if api_data[1] > 0:
			y = y + 35
		elif api_data[1] == 0:
			y = y - 20
		else:
			y = y - 35
		if api_data[2] > 0:
			z = z - 20
		else:
			z = z + 20
		transform.origin=Vector3(x, y, z)
		get_parent().get_parent().get_node("notification").generate_notification_message(api_data, _response_code, "_on_grab_location_of_cortical_request_completed", "/v1/feagi/genome/cortical_name_location")


func _on_menu_itemlist_item_selected(index):
	var names = $".."/".."/".."/".."/".."/Menu/information_menu/cortical_cam_label/menu_itemlist.get_item_text(index)
	if names != " " and cortical_pointer != names:
		var combine_url = "http://" + network_setting.api_ip_address + ":" + network_setting.api_port_address + "/v1/feagi/genome/cortical_name_location?cortical_name=" + names
		$".."/".."/".."/".."/".."/Menu/information_menu/cortical_cam_label/grab_location_of_cortical.request(combine_url)
		cortical_pointer = names
		$".."/".."/".."/".."/".."/Menu/information_menu/cortical_cam_label/menu.text = cortical_pointer
	$".."/".."/".."/".."/".."/Menu/information_menu/cortical_cam_label/menu_itemlist.visible = false
