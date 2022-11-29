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
extends Spatial


onready var textbox_display = get_node("Sprite3D")
onready var selected =  preload("res://selected.meshlib")
onready var deselected = preload("res://Cortical_area_box.meshlib")
onready var duplicate_model = get_node("Cortical_area")


var floor_size = 25
var grid_steps = 1000
var flag = 0
var test = ""
var data
var stored_value = ""
var current_pos = Vector3()
var total = []
var array_test = []
var number = 0
var voxel_history = 0
var history_total = 0
var Godot_list = {}
var x_increment = 0
var y_increment = 0
var z_increment = 0
var csv_flag = false
var connected = false
var stored_csv = ""
var genome_data = ""
var previous_genome_data = ""
var global_name_list = []
var last_cortical_selected
var start = 0 #for timer
var end = 0 #for timer
var increment_gridmap = 0


func _ready():
	set_physics_process(false)
	add_3D_indicator()


	while true:
#		if $Spatial/Camera/Menu/cortical_menu.visible:
		if cortical_is_clicked():
			$Spatial/Camera/Menu/cortical_menu.visible = true
		elif select_cortical.selected.empty() != true:
			select_cortical.selected.pop_front()
		_process(self)
		stored_value = data
		if "genome" in data:
			genome_data = parse_json(data)
			if str(genome_data) != str(previous_genome_data):
				_csv_generator()
				stored_value = ""
				previous_genome_data = genome_data
		elif str(genome_data) == "" or str(genome_data) == "{genome:{}}":
			websocket.send("empty")
#		print("data from python: ", data)
		start = OS.get_ticks_msec()## This will time the engine at start
		yield(get_tree().create_timer(0.01), "timeout")
		end = OS.get_ticks_msec()
		var time_total = end - start
		if time_total < 500: ## Generate voxels as long as you are on the tab
			generate_voxels()
		else:
			websocket.send("lagged")


func _process(_delta):
	#check_csv() ##Check if csv is changed
	data = websocket.one_frame
	
func generate_one_model(node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	var new = get_node("Cortical_area").duplicate()
	new.set_name(name_input)
	global_name_list.append({name_input.replace(" ", "") : [new, x_input, y_input, z_input, width_input, depth_input, height_input]})
	add_child(new)
	new.scale = Vector3(width_input, height_input, depth_input)
	new.transform.origin = Vector3(width_input/2 + int(x_input), height_input/2+ int(y_input), depth_input/2 + int(z_input))
	generate_textbox(node, x_input,height_input,z_input, name_input, y_input, width_input)
	
func convert_generate_one_model(_node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	var new = get_node("Cortical_area").duplicate()
	new.set_name(name_input)
	add_child(new)
	global_name_list.append({name_input.replace(" ", "") : [new, x_input, y_input, z_input, width_input, depth_input, height_input]})
	new.scale = Vector3(width_input, height_input, depth_input)
	new.transform.origin = Vector3(width_input/2 + int(x_input), height_input/2+ int(y_input), depth_input/2 + int(z_input))
	
func generate_model(node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	for x_gain in width_input:
		for y_gain in height_input:
			for z_gain in depth_input:
				if x_gain == 0 or x_gain == (int(width_input)-1) or y_gain == 0 or y_gain == (int(height_input) - 1) or z_gain == 0 or z_gain == (int(depth_input) - 1):
					var new = get_node("Cortical_area").duplicate()
					new.set_name(name_input)
					add_child(new)
					global_name_list.append({name_input.replace(" ", "") : [new, x_input, y_input, z_input, width_input, depth_input, height_input]})
					new.transform.origin = Vector3(x_gain+int(x_input), y_gain+int(y_input), z_gain+int(z_input))
					generate_textbox(node, x_input,height_input,z_input, name_input, y_input, width_input)

func generate_textbox(node, x_input,height_input,z_input, name_input, input_y, width_input):
	node.transform.origin = Vector3(int(x_input) + (width_input/1.5), int(int(input_y)+2 + (height_input)),z_input)
	node.get_node("Viewport/Label").set_text(str(name_input))
	node.get_node("Viewport").get_texture()
	global_name_list.append({name_input.replace(" ", ""): [node, x_input, 0, z_input, 0, 0, height_input]})

func install_voxel_inside(x_input,y_input,z_input):
	$GridMap.set_cell_item(x_input,y_input,z_input, 0)

func _csv_generator(): # After you are done with testing, change the name to genome_generator.
	_clear_node_name_list(global_name_list)
	for k in genome_data["genome"]:
		var CSV_data = genome_data["genome"][k]
		var x = CSV_data[4]; var y = CSV_data[5]; var z = CSV_data[6]; var width= int(CSV_data[7]) 
		var height = int(CSV_data[8]); var depth = int(CSV_data[9]); var name_input = CSV_data[0]
		$Floor_grid.set_cell_item(int(x),0,int(z),0)
		if sign(int(width)) > 0:
			x_increment = (int(width) / floor_size) + 1
			for i in x_increment:
				$Floor_grid.set_cell_item(int(x)+(i*floor_size),0,0,0)
		if sign(int(width)) < 0:
			x_increment = (int(width) / floor_size) - 1
			for i in range(0, x_increment):
				$Floor_grid.set_cell_item(int(x)+(-1*i*floor_size),0,0,0)
#				$Floor_grid.set_cell_item(0,0,int(z),0)
		if sign(int(depth)) > 0:
			z_increment = (int(depth) / floor_size) + 1
			for i in z_increment:
				$Floor_grid.set_cell_item(int(x),0,int(z)+(i*floor_size),0)
		if sign(int(width)) < 0:
			z_increment = (int(depth) / floor_size) - 1
			for i in range(0, z_increment):
				$Floor_grid.set_cell_item(int(x),0,int(z) + (-1*i*floor_size),0)
		var copy = duplicate_model.duplicate() 
		var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
		var viewport = create_textbox.get_node("Viewport")
		create_textbox.set_texture(viewport.get_texture())
		add_child(copy)
		global_name_list.append({name_input.replace(" ", "") : [copy, x, y, z, width, depth, height]})
		create_textbox.set_name(name_input.replace(" ", "") + "_textbox")
		add_child(create_textbox)#Copied the node to new node
		#global_name_list.append(create_textbox)
		create_textbox.scale = Vector3(1,1,1)
		if int(width) * int(depth) * int(height) < 999: # Prevent massive cortical area 
			generate_model(create_textbox, x,y,z,width, depth, height, name_input)
		else:
			generate_one_model(create_textbox, x,y,z,width, depth, height, name_input)

func _clear_node_name_list(node_name):
	var list = node_name
	if list.empty() != true:
		var list_size = global_name_list.size()
		for i in list_size:
			for iteration_name in global_name_list[i]:
				global_name_list[i][iteration_name][0].queue_free()
		global_name_list = []
	$Floor_grid.clear()

func _clear_single_cortical(node_name, node_list):
	var list = node_list
	if list.empty() != true:
		for search_name in list:
			if node_name == search_name:
				search_name.queue_free()
				global_name_list[search_name].clear()
	$Floor_grid.clear()

func generate_voxels():
	if stored_value != "" and stored_value != null:
		array_test = stored_value.replace("[", "")
		array_test = array_test.replace("]", "")
		test=array_test.split(",", true, '0')
		total = (test.size())
		$red_voxel.multimesh.instance_count = total
		$red_voxel.multimesh.visible_instance_count = total
		var key = 0
		var x
		var y
		var z
		flag=0
		while key < total:
			if flag == 0:
				flag = flag + 1
				x = int(test[key])
			elif flag == 1:
				flag = flag + 1
				y = int(test[key])
			elif flag == 2:
				flag = 0
				z = int(test[key])
				var position = Transform()
				position = position.translated(Vector3(int(x), int(y), int(z)))
				$red_voxel.multimesh.set_instance_transform(key, position)
			key+= 1
		flag = 0 # keep x,y,z in correct place

func cortical_is_clicked():
	if select_cortical.selected.empty() != true:
		var iteration_name = select_cortical.selected[0].replace("'","")
		var grab_id_cortical = ""
		for i in genome_data["genome"]:
			if genome_data["genome"][i][0] == iteration_name:
				grab_id_cortical = i
				break
		var combine_name = 'http://127.0.0.1:8000/v1/feagi/genome/cortical_properties?cortical_area=' + grab_id_cortical
		$Spatial/Camera/Menu/HTTPRequest.request(combine_name)
		select_cortical.selected.pop_front()
		return true
	return false

func _on_Update_pressed():
	var x = int($Spatial/Camera/Menu/cortical_menu/X.value);
	var y = int($Spatial/Camera/Menu/cortical_menu/Y.value);
	var z = int($Spatial/Camera/Menu/cortical_menu/Z.value);

	var width= int($Spatial/Camera/Menu/cortical_menu/properties/W.value)
	var height = int($Spatial/Camera/Menu/cortical_menu/properties/H.value);
	var depth = int($Spatial/Camera/Menu/cortical_menu/properties/D.value);

	var synaptic_attractivity = int($Spatial/Camera/Menu/cortical_menu/properties/syn.value);
	var post_synaptic_potential = float($Spatial/Camera/Menu/cortical_menu/properties/pst_syn.value);
	var post_synaptic_potential_max = float($Spatial/Camera/Menu/cortical_menu/properties/pst_syn_max.value);
	var plasticity_coef = float($Spatial/Camera/Menu/cortical_menu/properties/plst.value);
	var fire_threshold = float($Spatial/Camera/Menu/cortical_menu/properties/fire.value);
	var refractory_period = int($Spatial/Camera/Menu/cortical_menu/properties/refa.value);
	var leak_coefficient = float($Spatial/Camera/Menu/cortical_menu/properties/leak.value);
	var consecutive_fire_count = int($Spatial/Camera/Menu/cortical_menu/properties/cfr.value);
	var snooze_period = int($Spatial/Camera/Menu/cortical_menu/properties/snze.value);
	var degenerecy_coefficient = float($Spatial/Camera/Menu/cortical_menu/properties/dege.value);
	var psp_uniform_distribution = bool($Spatial/Camera/Menu/cortical_menu/properties/psud.toggle_mode)
	var name_input = $Spatial/Camera/Menu/cortical_menu/name_string.text
	var copy = duplicate_model.duplicate()
	var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox.get_node("Viewport")
	var store_global_data = []

	create_textbox.set_texture(viewport.get_texture())
	add_child(copy)
	global_name_list.append({name_input.replace(" ", "").replace(" ", "") : [copy, x, y, z, width, depth, height]})
	create_textbox.set_name(name_input + "_textbox")
	print("HERE: ", name_input)
	print("list: ", last_cortical_selected)
	add_child(create_textbox) # Copied the node to new node
	create_textbox.scale = Vector3(1,1,1)
	last_cortical_selected["cortical_name"] = name_input

	last_cortical_selected["cortical_coordinates"]["x"] = x
	last_cortical_selected["cortical_coordinates"]["y"] = y
	last_cortical_selected["cortical_coordinates"]["z"] = z

	last_cortical_selected["cortical_dimensions"]["x"] = width
	last_cortical_selected["cortical_dimensions"]["y"] = height
	last_cortical_selected["cortical_dimensions"]["z"] = depth

	last_cortical_selected["cortical_synaptic_attractivity"] = synaptic_attractivity
	last_cortical_selected["neuron_post_synaptic_potential"] = post_synaptic_potential
	last_cortical_selected["neuron_post_synaptic_potential_max"] = post_synaptic_potential_max
	last_cortical_selected["neuron_plasticity_constant"] = plasticity_coef
	last_cortical_selected["neuron_fire_threshold"] = fire_threshold
	last_cortical_selected["neuron_refractory_period"] = refractory_period
	last_cortical_selected["neuron_leak_coefficient"] = leak_coefficient
	last_cortical_selected["neuron_consecutive_fire_count"] = consecutive_fire_count
	last_cortical_selected["neuron_snooze_period"] = snooze_period
	last_cortical_selected["neuron_degeneracy_coefficient"] = degenerecy_coefficient
	last_cortical_selected["neuron_psp_uniform_distribution"] = psp_uniform_distribution

	_make_post_request('http://127.0.0.1:8000/v1/feagi/genome/cortical_properties',last_cortical_selected, false)

	var list_size = global_name_list.size()
	for i in list_size:
		var iteration_name = name_input
		if iteration_name in global_name_list[i]:
			global_name_list[i][iteration_name][0].queue_free()
		iteration_name = iteration_name + "_textbox"
		if iteration_name in global_name_list[i]:
			global_name_list[i][iteration_name][0].queue_free()
	for i in list_size:
		var iteration_name = name_input
		if iteration_name in global_name_list[i]:
			pass # Skip cortical area so global list can be updated
		else:
			store_global_data.append(global_name_list[i])
	global_name_list.clear()
	global_name_list = store_global_data
	
	if int(width) * int(depth) * int(height) < 999: # Prevent massive cortical area 
		generate_model(create_textbox, x,y,z,width, depth, height, name_input)
	else:
		generate_one_model(create_textbox, x,y,z,width, depth, height, name_input)

#func _on_reposition_pressed():
#	var get_name = $Spatial/Camera/Menu/move_cortical/cortical_menu/name.text
#	var get_x = $Spatial/Camera/Menu/move_cortical/cortical_menu/X.value
#	var get_y = $Spatial/Camera/Menu/move_cortical/cortical_menu/Y.value
#	var get_z = $Spatial/Camera/Menu/move_cortical/cortical_menu/Z.value
#	var get_w
#	var get_d
#	var get_h
#	var get_wdh = true
#	var list_size = global_name_list.size()
#	var store_global_data = []
#	for i in list_size:
#		var iteration_name = get_name
#		if iteration_name in global_name_list[i]:
#			if get_wdh:
#				if not "_textbox" in iteration_name:
#					get_wdh = false
#					get_w = global_name_list[i][iteration_name][4]
#					get_d = global_name_list[i][iteration_name][5]
#					get_h = global_name_list[i][iteration_name][6]
#			global_name_list[i][iteration_name][0].queue_free()
#		iteration_name = iteration_name + "_textbox"
#		if iteration_name in global_name_list[i]:
#			global_name_list[i][iteration_name][0].queue_free()
#	for i in list_size:
#		var iteration_name = get_name
#		if iteration_name in global_name_list[i]:
#			pass # Skip cortical area so global list can be updated
#		else:
#			store_global_data.append(global_name_list[i])
#	global_name_list.clear()
#	global_name_list = store_global_data
#
#	var get_id = ""
#	for i in genome_data['genome']:
#		if genome_data['genome'][i][0] == get_name:
#			get_id = i
#	var cortical_updated = {"\"cortical_name\"": str("\"", get_name, "\""), "\"cortical_id\"": str("\"", get_id, "\""), "\"cortical_coordinates\"": {"\"x\"": get_x, "\"y\"": get_y, "\"z\"": get_z}, "\"cortical_visibility\"": "\"true\""}
#	websocket.send(str(cortical_updated))
#
#
#	var copy = duplicate_model.duplicate() 
#	var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
#	var viewport = create_textbox.get_node("Viewport")
#	create_textbox.set_texture(viewport.get_texture())
#	add_child(copy)
##	global_name_list.append({name.replace(" ", "") : [copy, get_x, get_y, get_z, get_w, get_d, get_h]})
#	create_textbox.set_name(get_name.replace(" ", "") + "_textbox")
#	add_child(create_textbox)#Copied the node to new node
#	#global_name_list.append(create_textbox)
#	create_textbox.scale = Vector3(1,1,1)
#	if int(get_w) * int(get_d) * int(get_h) < 999: # Prevent massive cortical area 
#		generate_model(create_textbox, get_x,get_y,get_z, get_w, get_d, get_h, get_name)
#	else:
#		generate_one_model(create_textbox, get_x,get_y,get_z, get_w, get_d, get_h, get_name)
##	var x_input =  $Spatial/Camera/Menu/move_cortical/cortical_menu/X.value

func add_3D_indicator():
	for i in 6:
		$GridMap3.set_cell_item(i,0,0,0) ##set the arrow indicator of 3D
	var create_textbox_axis = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox_axis.get_node("Viewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("x_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(0.5,0.5,0.5)
	generate_textbox(create_textbox_axis, 5,0,0,"x", 1, 0)
	for j in 6:
		$GridMap3.set_cell_item(0,j,0,0)
	create_textbox_axis = create_textbox_axis.duplicate() #generate a new node to re-use the model
	viewport = create_textbox_axis.get_node("Viewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("y_textbox")
	add_child(create_textbox_axis) # Copied the node to new node
	create_textbox_axis.scale = Vector3(0.5,0.5,0.5)
	generate_textbox(create_textbox_axis, 0,5,0,"y", 1,0)
	for k in 6: 
		$GridMap3.set_cell_item(0,0,k,0)
	create_textbox_axis = textbox_display.duplicate() #generate a new node to re-use the model
	viewport = create_textbox_axis.get_node("Viewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("z_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(0.5,0.5,0.5)
	generate_textbox(create_textbox_axis, -2,0.5,6,"z", 1, 0)
	$GridMap.clear()


func _on_HTTPRequest_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var genome_properties = json.result
	# cortical_id missing
	$Spatial/Camera/Menu/cortical_menu/name_string.text = genome_properties["cortical_name"]
	# cortical_group missing
	# cortical_neuron_per_vox_count missing
	# cortical_visibility missing 
	$Spatial/Camera/Menu/cortical_menu/X.value = genome_properties["cortical_coordinates"]["x"]
	$Spatial/Camera/Menu/cortical_menu/Y.value = genome_properties["cortical_coordinates"]["y"]
	$Spatial/Camera/Menu/cortical_menu/Z.value = genome_properties["cortical_coordinates"]["z"]
	$Spatial/Camera/Menu/cortical_menu/properties/W.value = genome_properties["cortical_dimensions"]["x"]
	$Spatial/Camera/Menu/cortical_menu/properties/D.value = genome_properties["cortical_dimensions"]["z"]
	$Spatial/Camera/Menu/cortical_menu/properties/H.value = genome_properties["cortical_dimensions"]["y"]
	$Spatial/Camera/Menu/cortical_menu/properties/syn.value = genome_properties["cortical_synaptic_attractivity"]
	$Spatial/Camera/Menu/cortical_menu/properties/plst.value = genome_properties["neuron_post_synaptic_potential"]
	$Spatial/Camera/Menu/cortical_menu/properties/pst_syn_max.value = genome_properties["neuron_post_synaptic_potential_max"]
	# neuron_plasticity_constant missing
	$Spatial/Camera/Menu/cortical_menu/properties/fire.value = genome_properties["neuron_fire_threshold"]
	$Spatial/Camera/Menu/cortical_menu/properties/refa.value = genome_properties["neuron_refractory_period"]
	$Spatial/Camera/Menu/cortical_menu/properties/leak.value = genome_properties["neuron_leak_coefficient"]
	$Spatial/Camera/Menu/cortical_menu/properties/cfr.value = genome_properties["neuron_consecutive_fire_count"]
	$Spatial/Camera/Menu/cortical_menu/properties/snze.value = genome_properties["neuron_snooze_period"]
	$Spatial/Camera/Menu/cortical_menu/properties/dege.value = genome_properties["neuron_degeneracy_coefficient"]
	$Spatial/Camera/Menu/cortical_menu/properties/psud.text = str(genome_properties["neuron_psp_uniform_distribution"])
	last_cortical_selected = genome_properties


func _make_post_request(url, data_to_send, use_ssl):
	# Convert data to json string:
	var query = JSON.print(data_to_send)
	# Add 'Content-Type' header:
	var headers = ["Content-Type: application/json"]
	$Spatial/Camera/Menu/send_feagi.request(url, headers, use_ssl, HTTPClient.METHOD_POST, query)

func _on_send_feagi_request_completed(_result, _response_code, _headers, _body):
	pass


func _on_psud_toggled(button_pressed):
	print($Spatial/Camera/Menu/cortical_menu/properties/psud.toggle_mode)
	print(button_pressed)
