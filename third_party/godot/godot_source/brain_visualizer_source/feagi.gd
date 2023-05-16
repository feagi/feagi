"""
==============================================================================
Copyright 2016-2022 The FEAGI Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================
"""
extends Node3D


@onready var textbox_display = get_node("Sprite3D")
@onready var selected =  preload("res://brain_visualizer_source/selected.meshlib")
@onready var deselected = preload("res://brain_visualizer_source/Cortical_area_box.meshlib")
@onready var duplicate_model = get_node("Cortical_area")


var floor_size = 25
var grid_steps = 1000
var flag = 0
var test = ""
var data
var stored_value = ""
var total = []
var array_test = []
var x_increment = 0
var y_increment = 0
var z_increment = 0
var one_time_flag = true
var stored_csv = ""
var previous_genome_data = {}
var global_name_list = []
var last_cortical_selected = {}
var start = 0 #for timer
var end = 0 #for timer
var increment_gridmap = 0
var child_node_holder = []
var afferent_child_holder = []
var get_id_from_dst
var plus_node = []
var dst_data_holder
var ghost_morphology = []
var new_morphology_node = []
var latest_send_feagi = ""
var type = ""
var SEC

func _ready():
	# # # Initalize the bridge # # # 
	Autoload_variable.BV_Core = get_parent().get_parent()
	Autoload_variable.BV_UI = get_parent()
	await get_tree().create_timer(1.5).timeout
	SEC = 'HTTP://' + network_setting.api_ip_address
	set_physics_process(false)
	add_3D_indicator()
	Autoload_variable.BV_Core.Update_Dimensions() # Grab genome list
	Autoload_variable.BV_Core.Update_Morphology_type()
	while true:
		if Godot_list.genome_data["genome"] != previous_genome_data:
			previous_genome_data = Godot_list.genome_data["genome"].duplicate()
			_csv_generator()
		if $".."/".."/".."/Menu/box_loading.visible:
			$".."/".."/".."/Menu/box_loading.visible = false
		if cortical_is_clicked():
			$".."/".."/".."/Menu/cortical_menu.visible = true
			$".."/".."/".."/Menu/cortical_mapping.visible = true
			$".."/".."/".."/Menu/button_choice.visible = true
			$".."/".."/".."/Menu/properties.visible = true
			$".."/".."/".."/Menu/Mapping_Properties.visible = false
			$".."/".."/".."/Menu/collapse_4.visible = true
			$".."/".."/".."/Menu/cortical_menu/title.visible = true
		elif select_cortical.selected.is_empty() != true:
			select_cortical.selected.pop_front()
		_process(self)
#		print("FROM PYTHON: ", data)
		if data != null:
			if "update" in data:
				$".."/".."/".."/Menu/box_loading.visible = true
				Autoload_variable.BV_Core.Update_Dimensions()
				stored_value= ""
			else:
				stored_value = data
			start = Time.get_ticks_msec() # This will time the engine at start
			await get_tree().create_timer(0.01).timeout
			end = Time.get_ticks_msec()
			var time_total = end - start
			if time_total < 500: # Generate voxels as long as you are on the tab
				generate_voxels()
			else:
				network_setting.send("lagged")
		else:
			await get_tree().create_timer(0.01).timeout

func _process(_delta):
	data = network_setting.one_frame

func generate_one_model(node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	var new = get_node("Cortical_area").duplicate()
	new.set_name(name_input)
	global_name_list.append({name_input.replace(" ", "") : [new, x_input, y_input, z_input, width_input, depth_input, height_input]})
	add_child(new)
	new.visible = true
	new.scale = Vector3(width_input, height_input, depth_input)
	new.transform.origin = Vector3(width_input/2 + int(x_input), height_input/2+ int(y_input), -1 * (depth_input/2 + int(z_input)))
	generate_textbox(node, x_input,height_input,z_input, name_input, y_input, width_input, depth_input)

func convert_generate_one_model(_node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	var new = get_node("Cortical_area").duplicate()
	new.set_name(name_input)
	add_child(new)
	new.visible = true
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
					new.visible = true
					global_name_list.append({name_input.replace(" ", "") : [new, x_input, y_input, z_input, width_input, depth_input, height_input]})
					new.transform.origin = Vector3(x_gain+int(x_input), y_gain+int(y_input), -1 * (z_gain+int(z_input)))
					generate_textbox(node, x_input,height_input,z_input, name_input, y_input, width_input, depth_input)

func generate_textbox(node, x_input,height_input,z_input, name_input, input_y, width_input, depth_input):
	node.transform.origin = Vector3(int(x_input) + (width_input/1.5), int(int(input_y)+1 + (height_input)), -1 * depth_input - z_input)
	node.get_node("SubViewport/Label").set_text(str(name_input))
	node.get_node("SubViewport").get_texture()
	if not name_input in ["x", "y", "z"]:
		global_name_list.append({name_input.replace(" ", ""): [node, x_input, 0, z_input, 0, 0, height_input]})

func install_voxel_inside(x_input,y_input,z_input):
	$GridMap.set_cell_item( Vector3(x_input,y_input,z_input) ,0)

func _csv_generator(): # After you are done with testing, change the name to genome_generator.
	for key in Godot_list.godot_list["data"]["direct_stimulation"]:
		Godot_list.godot_list["data"]["direct_stimulation"][key] = []
	_clear_node_name_list(global_name_list)
	for k in Godot_list.genome_data["genome"]:
		var CSV_data = Godot_list.genome_data["genome"][k]
		var x = CSV_data[0]; var y = CSV_data[1]; var z = CSV_data[2]; var width= int(CSV_data[4]) 
		var height = int(CSV_data[5]); var depth = int(CSV_data[6]); var name_input = k
		$Floor_grid.set_cell_item( Vector3(int(x),0,int(z)) ,0)
		if sign(int(width)) > 0:
			x_increment = (width / floor_size) + 1
			for i in x_increment:
				$Floor_grid.set_cell_item( Vector3(int(x)+(i*floor_size),0,0) ,0)
		if sign(int(width)) < 0:
			x_increment = (int(width) / floor_size) - 1
			for i in range(0, x_increment):
				$Floor_grid.set_cell_item( Vector3(int(x)+(-1*i*floor_size),0,0) ,0)
#				$Floor_grid.set_cell_item( Vector3(0,0,int(z)) ,0)
		if sign(depth) > 0:
			z_increment = (depth / floor_size) + 1
			for i in z_increment:
				$Floor_grid.set_cell_item( Vector3(int(x),0,int(z)+(i*floor_size)) ,0)
		if sign(int(width)) < 0:
			z_increment = (depth / floor_size) - 1
			for i in range(0, z_increment):
				$Floor_grid.set_cell_item( Vector3(int(x),0,int(z) + (-1*i*floor_size)) ,0)
		var copy = duplicate_model.duplicate() 
		var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
		var viewport = create_textbox.get_node("SubViewport")
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
	Autoload_variable.BV_Core.Update_GenomeFileName()

func _clear_node_name_list(node_name):
	"""
	clear all cortical area along with the library list/dict
	"""
	var list = node_name
	if list.is_empty() != true:
		var list_size = global_name_list.size()
		for i in list_size:
			for iteration_name in global_name_list[i]:
				global_name_list[i][iteration_name][0].queue_free()
		global_name_list = []
	$Floor_grid.clear()

func _clear_single_cortical(cortical_name, node_list):
	var list = node_list
	if list.is_empty() != true:
		for search_name in list:
			if cortical_name in search_name:
				search_name[cortical_name][0].queue_free()
		erase_single_cortical_library(cortical_name)
		$Floor_grid.clear()

func generate_voxels():
	if stored_value != null:
		total = len(stored_value)
		$red_voxel.multimesh.instance_count = total
		$red_voxel.multimesh.visible_instance_count = total
		flag = 0
		for i in stored_value:
			var new_position = Transform3D().translated(Vector3(i[0], i[1], -i[2]))
			$red_voxel.multimesh.set_instance_transform(flag, new_position)
			flag += 1

func cortical_is_clicked():
	if select_cortical.selected.is_empty() != true:
		dst_data_holder = {}
		var iteration_name = select_cortical.selected[0].replace("'","")
		var grab_id_cortical = ""
		for i in Godot_list.genome_data["genome"]:
			if i == iteration_name:
				grab_id_cortical = Godot_list.genome_data["genome"][i][7]
				break
		update_cortical_map_name(grab_id_cortical)
		Autoload_variable.BV_Core.Update_Cortical_grab_id(grab_id_cortical)
		select_cortical.selected.pop_front()
		return true
	return false

func erase_single_cortical_library(name_input):
	var store_global_data = []
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

func generate_single_cortical(x,y,z,width, depth, height, name_input):
	var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox.get_node("SubViewport")
	var copy = duplicate_model.duplicate()
	
	# Generate title for this single cortical area
	create_textbox.set_texture(viewport.get_texture())
	add_child(copy)
	global_name_list.append({name_input.replace(" ", "").replace(" ", "") : [copy, x, y, z, width, depth, height]})
	create_textbox.set_name(name_input + "_textbox")
	add_child(create_textbox) # Copied the node to new node
	create_textbox.scale = Vector3(1,1,1)
	
	erase_single_cortical_library(name_input)
	
	if int(width) * int(depth) * int(height) < 999: # Prevent massive cortical area 
		generate_model(create_textbox, x,y,z,width, height, depth, name_input)
	else:
		generate_one_model(create_textbox, x,y,z,width, height, depth, name_input)

func _on_Update_pressed():
	var x = int($".."/".."/".."/Menu/cortical_menu/Control/X.value);
	var y = int($".."/".."/".."/Menu/cortical_menu/Control/Y.value);
	var z = int($".."/".."/".."/Menu/cortical_menu/Control/Z.value);
	var id_input = str($".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text)
	var width= int($".."/".."/".."/Menu/cortical_menu/Control/W.value)
	var height = int($".."/".."/".."/Menu/cortical_menu/Control/H.value);
	var depth = int($".."/".."/".."/Menu/cortical_menu/Control/D.value);
	var synaptic_attractivity = int($".."/".."/".."/Menu/properties/Control/syn.value);
	var post_synaptic_potential = $".."/".."/".."/Menu/properties/Control/pst_syn.value;
	var post_synaptic_potential_max = float($".."/".."/".."/Menu/properties/Control/pst_syn_max.value);
	var plasticity_coef = float($".."/".."/".."/Menu/properties/Control/plst.value);
	var fire_threshold = float($".."/".."/".."/Menu/properties/Control/fire.value);
	var fire_threshold_limit = int($".."/".."/".."/Menu/properties/Control/Threshold_Sensitivity_text.value)
	var refractory_period = int($".."/".."/".."/Menu/properties/Control/refa.value);
	var leak_coefficient = float($".."/".."/".."/Menu/properties/Control/leak.text);
	var leak_variability = float($".."/".."/".."/Menu/properties/Control/leak_Vtext.text);
	var fire_threshold_increment = $".."/".."/".."/Menu/properties/Control/fireshold_increment.text
	var consecutive_fire_count = int($".."/".."/".."/Menu/properties/Control/cfr.value);
	var snooze_period = int($".."/".."/".."/Menu/properties/Control/snze.value);
	var degenerecy_coefficient = float($".."/".."/".."/Menu/properties/Control/dege.value);
	var psp_uniform_distribution = $".."/".."/".."/Menu/properties/Control/psud.is_pressed()
	var MP_accumulation = $".."/".."/".."/Menu/properties/Control/MP.is_pressed()
	var name_input = $".."/".."/".."/Menu/cortical_menu/Control/name_string.text
	var copy = duplicate_model.duplicate()
	var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox.get_node("SubViewport")
	var store_global_data = []

	create_textbox.set_texture(viewport.get_texture())
	add_child(copy)
	global_name_list.append({name_input.replace(" ", "").replace(" ", "") : [copy, x, y, z, width, depth, height]})
	create_textbox.set_name(name_input + "_textbox")
	add_child(create_textbox) # Copied the node to new node
	create_textbox.scale = Vector3(1,1,1)



	last_cortical_selected["cortical_coordinates"] = []
	last_cortical_selected["cortical_destinations"] = {}
	last_cortical_selected["cortical_dimensions"] = []
	var cortical_name = $".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_item_text($".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_selected_id())
	var get_id = name_to_id(cortical_name)
	for i in child_node_holder:
		if i.text == "":
			if $".."/".."/".."/Menu/Mapping_Properties.visible:
				if $".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_selected_id() != 0:
					i.text = $".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_item_text($".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_selected_id())
		else:
			if i.text != cortical_name:
				last_cortical_selected["cortical_destinations"][name_to_id(i.text)] = dst_data_holder[name_to_id(i.text)] # This fixed
	for p in plus_node:
		var dst = {}
		dst["morphology_id"] = p.get_child(0).get_item_text(p.get_child(0).get_selected_id())
		dst["morphology_scalar"] = []
		dst["morphology_scalar"].append(p.get_child(1).value)
		dst["morphology_scalar"].append(p.get_child(2).value)
		dst["morphology_scalar"].append(p.get_child(3).value) 
		dst["postSynapticCurrent_multiplier"] = int(p.get_child(4).text)
		dst["plasticity_flag"] = p.get_child(5).is_pressed()
		if last_cortical_selected["cortical_destinations"].has(get_id):
			last_cortical_selected["cortical_destinations"][get_id].append(dst)
		else:
			last_cortical_selected["cortical_destinations"][get_id] = []
			last_cortical_selected["cortical_destinations"][get_id].append(dst)
	if id_input == "":
		last_cortical_selected["cortical_id"]= 'new_id'
	else:
		last_cortical_selected["cortical_id"]= id_input
	last_cortical_selected["cortical_name"] = name_input
	last_cortical_selected["cortical_group"] = last_cortical_selected["cortical_group"]
	last_cortical_selected["cortical_neuron_per_vox_count"] = $".."/".."/".."/Menu/properties/Control/neuron_count.value
	last_cortical_selected["cortical_coordinates"].append(x)
	last_cortical_selected["cortical_coordinates"].append(y)
	last_cortical_selected["cortical_coordinates"].append(z)
	last_cortical_selected["cortical_dimensions"].append(width)
	last_cortical_selected["cortical_dimensions"].append(height)
	last_cortical_selected["cortical_dimensions"].append(depth)
	last_cortical_selected["cortical_synaptic_attractivity"] = synaptic_attractivity
	last_cortical_selected["neuron_post_synaptic_potential"] = post_synaptic_potential
	last_cortical_selected["neuron_post_synaptic_potential_max"] = post_synaptic_potential_max
	last_cortical_selected["neuron_plasticity_constant"] = plasticity_coef
	last_cortical_selected["neuron_fire_threshold"] = fire_threshold
	last_cortical_selected["neuron_fire_threshold_increment"] = fire_threshold_increment
	last_cortical_selected["neuron_firing_threshold_limit"] = fire_threshold_limit
	last_cortical_selected["neuron_refractory_period"] = refractory_period
	last_cortical_selected["neuron_leak_coefficient"] = float(leak_coefficient)
	last_cortical_selected["neuron_leak_variability"] = float(leak_variability)
	last_cortical_selected["neuron_consecutive_fire_count"] = consecutive_fire_count
	last_cortical_selected["neuron_snooze_period"] = snooze_period
	last_cortical_selected["neuron_degeneracy_coefficient"] = degenerecy_coefficient
	last_cortical_selected["neuron_psp_uniform_distribution"] = psp_uniform_distribution
	last_cortical_selected["neuron_mp_charge_accumulation"] = bool(MP_accumulation)
	Autoload_variable.BV_Core.Update_Genome_CorticalArea(last_cortical_selected)
	$".."/".."/".."/Menu/cortical_menu/Control/Update.release_focus()
	$".."/".."/".."/Menu/properties/Control/Update.release_focus()

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

	$".."/".."/".."/Menu/Mapping_Properties.visible = false
	$".."/".."/".."/Menu/cortical_menu/Control/Update.release_focus()

func add_3D_indicator():
	for i in 6:
		$GridMap3.set_cell_item( Vector3(i,0,0) ,0) #set the arrow indicator of 3D
	var create_textbox_axis = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox_axis.get_node("SubViewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("x_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(1, 1, 1)
	generate_textbox(create_textbox_axis, 10,0,0,"x", 1, 0, 0)
	for j in 6:
		$GridMap3.set_cell_item( Vector3(0,j,0) ,0)
	create_textbox_axis = create_textbox_axis.duplicate() #generate a new node to re-use the model
	viewport = create_textbox_axis.get_node("SubViewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("y_textbox")
	add_child(create_textbox_axis) # Copied the node to new node
	create_textbox_axis.scale = Vector3(1, 1, 1)
	generate_textbox(create_textbox_axis, 5,5,0,"y", 1,0, 0)
	for k in 6: 
		$GridMap3.set_cell_item( Vector3(0,0,k) ,0)
	create_textbox_axis = textbox_display.duplicate() #generate a new node to re-use the model
	viewport = create_textbox_axis.get_node("SubViewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("z_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(1, 1, 1)
	generate_textbox(create_textbox_axis, 4,0,-6,"z", 1, 0, 0)
	$GridMap.clear()

func _on_HTTPRequest_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var genome_properties = test_json_conv.get_data()
	if _response_code == 200:
		$".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text = genome_properties["cortical_id"]
		$".."/".."/".."/Menu/cortical_menu/title.text = genome_properties["cortical_name"]
		$".."/".."/".."/Menu/cortical_menu/Control/name_string.text = $".."/".."/".."/Menu/cortical_menu/title.text
		$".."/".."/".."/Menu/properties/Control/neuron_count.value = genome_properties["cortical_neuron_per_vox_count"]
#		Autoload_variable.BV_UI.SpawnLeftBar()
		# I just want to populate the data using the new menu without modify anything significantly
#		Autoload_variable.BV_UI.dataUp.emit(genome_properties["cortical_neuron_per_vox_count"])
		$".."/".."/".."/Menu/cortical_menu/Control/X.value = genome_properties["cortical_coordinates"][0]
		$".."/".."/".."/Menu/cortical_menu/Control/Y.value = genome_properties["cortical_coordinates"][1]
		$".."/".."/".."/Menu/cortical_menu/Control/Z.value = genome_properties["cortical_coordinates"][2]
		$".."/".."/".."/Menu/cortical_menu/Control/W.value = genome_properties["cortical_dimensions"][0]
		$".."/".."/".."/Menu/cortical_menu/Control/D.value = genome_properties["cortical_dimensions"][2]
		$".."/".."/".."/Menu/cortical_menu/Control/H.value = genome_properties["cortical_dimensions"][1]
		$".."/".."/".."/Menu/properties/Control/syn.value = genome_properties["cortical_synaptic_attractivity"]
		$".."/".."/".."/Menu/properties/Control/pst_syn.value = genome_properties["neuron_post_synaptic_potential"]
		$".."/".."/".."/Menu/properties/Control/pst_syn_max.value = float(genome_properties["neuron_post_synaptic_potential_max"])
		$".."/".."/".."/Menu/properties/Control/plst.value = genome_properties["neuron_plasticity_constant"]
		$".."/".."/".."/Menu/properties/Control/fire.value = genome_properties["neuron_fire_threshold"]
		$".."/".."/".."/Menu/properties/Control/Threshold_Sensitivity_text.value = int(genome_properties["neuron_firing_threshold_limit"])
		$".."/".."/".."/Menu/properties/Control/fireshold_increment.text = str(genome_properties["neuron_fire_threshold_increment"])
		$".."/".."/".."/Menu/properties/Control/refa.value = genome_properties["neuron_refractory_period"]
		$".."/".."/".."/Menu/properties/Control/leak.text = str(float(genome_properties["neuron_leak_coefficient"]))
		$".."/".."/".."/Menu/properties/Control/leak_Vtext.text = str((genome_properties["neuron_leak_variability"]))
		$".."/".."/".."/Menu/properties/Control/cfr.value = genome_properties["neuron_consecutive_fire_count"]
		$".."/".."/".."/Menu/properties/Control/snze.value = genome_properties["neuron_snooze_period"]
		$".."/".."/".."/Menu/properties/Control/dege.value = genome_properties["neuron_degeneracy_coefficient"]
		$".."/".."/".."/Menu/properties/Control/psud.set_pressed(genome_properties["neuron_psp_uniform_distribution"])
		if genome_properties["neuron_mp_charge_accumulation"] != null:
			$".."/".."/".."/Menu/properties/Control/MP.set_pressed(genome_properties["neuron_mp_charge_accumulation"])
		else:
			$".."/".."/".."/Menu/properties/Control/MP.set_pressed(false)
		last_cortical_selected = genome_properties
		Autoload_variable.BV_Core.Update_Afferent_list(genome_properties["cortical_id"])
	$notification.generate_notification_message(genome_properties, _response_code, "_on_HTTPRequest_request_completed", "/v1/feagi/genome/cortical_area")

func _on_send_feagi_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	$notification.generate_notification_message(api_data, _response_code, "_on_send_feagi_request_completed", latest_send_feagi, type)

func _on_info_pressed():
	for i in plus_node:
		i.queue_free()
	plus_node.clear()
	$".."/".."/".."/Menu/Mapping_Properties.visible = false
	$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu.size.y = 107
	$".."/".."/".."/Menu/Mapping_Properties.visible = true

func update_cortical_map_name(name_input):
	Autoload_variable.BV_Core.Update_Efferent_information(name_input)

func _on_information_button_request_completed(_result, _response_code, _headers, body):
	# Do not touch here. THis is for information biteration_nameutton only and will dedicate
	# to the information button
	# Clear duplicate cortical maps name up
	child_holder_clear() 
	# Obtain the data from API and convert it into json/string
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	if _response_code == 200 and not api_data.has("Request failed..."):
		var new_name = ""
		var counter = 0 # To increase the height between two different duplicated nodes
	#	$".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.visible = false
		var cap = 120 # Keep nodes inside the white rectangle
		for i in api_data:
			var new_node = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.duplicate()
			$".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.add_child(new_node)
			child_node_holder.append(new_node)
			new_name = id_to_name(i)
			new_node.visible = true
			new_node.text = new_name
			new_node.position.x = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.position.x
			new_node.position.y = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.position.y + (counter * 30)
			if new_node.position.y > cap:
	#			$".."/".."/".."/Menu/button_choice/white_background.size.y += 30
				cap += 30 
	#		$".."/".."/".."/Menu/button_choice.position.y = $".."/".."/".."/Menu/button_choice.position.y + (counter * 5)
			new_node.size.x = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.size.x 
			new_node.size.y = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.size.y
			new_node.visible = true
			new_node.get_child(1).connect("pressed",Callable(self,"dst_remove_pressed").bind(new_node))
			new_node.get_child(0).connect("pressed",Callable(self,"info_pressed").bind(new_node))
			counter += 1
#		map_colorful()
	#	$".."/".."/".."/Menu/cortical_menu/Control/Update.position.y = 10 + $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.size.y + $".."/".."/".."/Menu/cortical_mapping.position.y
	else:
		$notification.generate_notification_message(api_data, _response_code, "_on_information_button_request_completed", "/v1/feagi/genome/cortical_mappings/efferents")

func child_holder_clear():
	# Clear duplicate cortical maps name up
	if child_node_holder:
		for i in child_node_holder:
			i.queue_free()
		child_node_holder = []
	if ghost_morphology:
		ghost_morphology = []

func _on_get_genome_name_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	if api_data != null:
		$".."/".."/".."/Menu/information_menu/genome_string.text = api_data
		Autoload_variable.BV_Core.Update_Refresh_Rate()
	$notification.generate_notification_message(api_data, _response_code, "_on_get_genome_name_request_completed", "/v1/feagi/genome/file_name")


func _on_get_burst_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	$".."/".."/".."/Menu/information_menu/burst_duration_label/burst_value.text = str(1/float(api_data))
	$notification.generate_notification_message(api_data, _response_code, "_on_get_burst_request_completed", "/v1/feagi/feagi/burst_engine/stimulation_period")

func _on_download_pressed():
	_clear_node_name_list(global_name_list)
	Godot_list.genome_data["genome"] = {}
	previous_genome_data = {}

func _on_add_pressed():
	var json_data = {}
	var flag_boolean = false
	if $".."/".."/".."/Menu/addition_menu/OptionButton.selected == 1 or $".."/".."/".."/Menu/addition_menu/OptionButton.selected == 2:
		json_data["cortical_type"] = $".."/".."/".."/Menu/addition_menu/OptionButton.get_item_text($".."/".."/".."/Menu/addition_menu/OptionButton.selected)
		json_data["cortical_name"] = $".."/".."/".."/Menu/addition_menu/cortical_name_label/type.get_item_text($".."/".."/".."/Menu/addition_menu/cortical_name_label/type.selected)
		json_data["cortical_coordinates"] = []
		json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value)
		json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value)
		json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value)
		json_data["channel_count"] = $".."/".."/".."/Menu/addition_menu/count/count_spinbox.value
		Autoload_variable.BV_Core.Update_cortical_area(json_data)
	if $".."/".."/".."/Menu/addition_menu/OptionButton.selected == 3:
		if $".."/".."/".."/Menu/addition_menu/cortical_name_textbox/type.text != "" and $".."/".."/".."/Menu/addition_menu/cortical_name_textbox/type.text != " ":
			json_data["cortical_type"] = "CUSTOM"
			json_data["cortical_name"] = $".."/".."/".."/Menu/addition_menu/cortical_name_textbox/type.text
			json_data["cortical_coordinates"] = []
			json_data["cortical_dimensions"] = []
			json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value)
			json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value)
			json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value)
			json_data["cortical_dimensions"].append($".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value)
			json_data["cortical_dimensions"].append($".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value)
			json_data["cortical_dimensions"].append($".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value)
			json_data["channel_count"] = $".."/".."/".."/Menu/custom_cortical/count_spinbox.value
			generate_single_cortical(json_data["cortical_coordinates"][0], json_data["cortical_coordinates"][1], json_data["cortical_coordinates"][2], json_data["cortical_dimensions"][0], json_data["cortical_dimensions"][1], json_data["cortical_dimensions"][2], json_data["cortical_name"])
			Autoload_variable.BV_Core.Update_custom_cortical_area(json_data)
			$".."/".."/".."/Menu/addition_menu/add.release_focus()
			$Node3D/Camera3D.transform.origin=Vector3(json_data["cortical_coordinates"][0]-20,json_data["cortical_coordinates"][1],json_data["cortical_coordinates"][2]+20)
		else:
			flag_boolean = true
	if flag_boolean != true:
		$".."/".."/".."/Menu/addition_menu.visible = false
		$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.load_options()
	$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.load_options()

func _on_remove_pressed():
	var get_name_data = $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text
	_clear_single_cortical($".."/".."/".."/Menu/cortical_menu/title.text, global_name_list)
	Autoload_variable.BV_Core.Delete_cortical_area(get_name_data)
	$".."/".."/".."/Menu/cortical_menu/Control/remove.release_focus()

func _on_update_destination_info_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	plus_node_clear()
	ghost_morphology_clear()
	if api_data != null:
		if api_data.has("Request failed..."):
			pass
		else:
			for i in range(len(api_data)):
				var new_node = $".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control.duplicate()
				$".."/".."/".."/Menu/"Mapping_Properties"/inside_mapping_menu.add_child(new_node)
				new_node.position.y = (50 * (plus_node.size()))
				plus_node.append(new_node)
				$".."/".."/".."/Menu/"Mapping_Properties"/inside_mapping_menu.size.y += (30 * plus_node.size())
				new_node.get_child(0).connect("pressed",Callable(self,"_on_Mapping_def_pressed"))
				new_node.get_child(4).text_changed.connect(_on_text_changed.bind(new_node.get_child(4)))
				ghost_morphology.append(new_node.get_child(0))

				for x in new_node.get_child(0).get_item_count():
					if new_node.get_child(0).get_item_text(x) == api_data[i]["morphology_id"]:
						new_node.get_child(0).selected = x
						$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.selected = x
				new_node.visible = true
				new_node.get_child(1).value = api_data[i]["morphology_scalar"][0]
				new_node.get_child(2).value = api_data[i]["morphology_scalar"][1]
				new_node.get_child(3).value = api_data[i]["morphology_scalar"][2]
				new_node.get_child(4).text = str(api_data[i]["postSynapticCurrent_multiplier"])
				new_node.get_child(5).set_pressed(api_data[i]["plasticity_flag"])
				new_node.get_child(6).connect("pressed",Callable(self,"map_info_pressed").bind(new_node))
				new_node.get_child(7).connect("pressed",Callable(self,"remove_button_inside_dst").bind(new_node))
	$notification.generate_notification_message(api_data, _response_code, "_on_update_destination_info_request_completed", "/v1/feagi/genome/mapping_properties")

func _on_genome_data_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	var create_json = {}
	if api_data != null:
		for i in api_data:
			create_json[i] = api_data[i]
		Godot_list.genome_data["genome"] = create_json
		_csv_generator()

func _on_remove_cortical_request_completed(_result, _response_code, _headers, _body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(_body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	$notification.generate_notification_message(api_data, _response_code, "_on_remove_cortical_request_completed")

func map_info_pressed(node_duplicated):
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.selected = node_duplicated.get_child(0).get_selected_id()
	Autoload_variable.BV_Core.Get_Morphology_information($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text(node_duplicated.get_child(0).get_selected_id()))

func remove_button_inside_dst(node_duplicated):
	var number_holder = []
	var rule_to_delete = node_duplicated.get_child(0).text
	var morphology_scalar_x = node_duplicated.get_child(1).value
	var morphology_scalar_y = node_duplicated.get_child(2).value
	var morphology_scalar_z = node_duplicated.get_child(3).value
	var plasticity_flag = node_duplicated.get_child(5).is_pressed()
	var postSynapticCurrent_multiplier = float(node_duplicated.get_child(4).text)

	var get_id = name_to_id($".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_item_text($".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_selected_id()))
	var inner_counter = 0
	for i in range(dst_data_holder[get_id].size()):
		if rule_to_delete in dst_data_holder[get_id][i-inner_counter]["morphology_id"] and morphology_scalar_x == dst_data_holder[get_id][i-inner_counter]["morphology_scalar"][0] and morphology_scalar_y == dst_data_holder[get_id][i-inner_counter]["morphology_scalar"][1] and morphology_scalar_z == dst_data_holder[get_id][i-inner_counter]["morphology_scalar"][2] and plasticity_flag == dst_data_holder[get_id][i-inner_counter]["plasticity_flag"] and postSynapticCurrent_multiplier == dst_data_holder[get_id][i-inner_counter]["postSynapticCurrent_multiplier"]:
			dst_data_holder[get_id].pop_at(i - inner_counter)
			inner_counter += 1
	for i in range(plus_node.size()):
		if node_duplicated == plus_node[i]:
			plus_node[i].queue_free()
			number_holder.append(i)
	var counter = 0
	for i in number_holder:
		plus_node.pop_at(i - counter)
		counter += 1
func info_pressed(duplicated_node_lineedit):
	var get_id = $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text
	if duplicated_node_lineedit.text != " " and duplicated_node_lineedit.text != "":
		get_id_from_dst = Godot_list.genome_data["genome"][duplicated_node_lineedit.text][7]
		var combine_url = '#&dst_cortical_area=$'.replace("#", get_id)
		combine_url= combine_url.replace("$", get_id_from_dst)
		Autoload_variable.BV_Core.Update_destination(combine_url)
	for i in $".."/".."/".."/Menu/Mapping_Properties/source_dropdown.get_item_count():
		if $".."/".."/".."/Menu/cortical_menu/Control/name_string.text == $".."/".."/".."/Menu/Mapping_Properties/source_dropdown.get_item_text(i):
			$".."/".."/".."/Menu/Mapping_Properties/source_dropdown.selected = i
	for i in $".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_item_count():
		if duplicated_node_lineedit.text == $".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_item_text(i):
			$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.selected = i

func dst_remove_pressed(duplicated_node_lineedit):
	_on_info_pressed() # leveraging the same function to clear all infos on the box
	$".."/".."/".."/Menu/Mapping_Properties.visible = false
	$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.select(0)
	var dst_id = name_to_id(duplicated_node_lineedit.text)
	var grab_id = $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text
	var combine_url = '?src_cortical_area=#&dst_cortical_area=$'.replace("#", grab_id)
	if dst_id != null:
		combine_url= combine_url.replace("$", dst_id)
		var number_holder = []
		for i in range(child_node_holder.size()):
			if child_node_holder[i].text == duplicated_node_lineedit.text:
				child_node_holder[i].queue_free()
				number_holder.append(i)
		var counter = 0
		for x in number_holder:
			child_node_holder.pop_at(x - counter)
			counter += 1
		Autoload_variable.BV_Core.Update_Mapping_Properties([],combine_url)
		$".."/".."/".."/Menu/cortical_menu/Control/Update.position.y = 10 + $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.size.y + $".."/".."/".."/Menu/cortical_mapping.position.y 
		$".."/".."/".."/Menu/cortical_mapping.position.y = $".."/".."/".."/Menu/cortical_mapping.position.y - (number_holder.size() * 5)

func delete_morphology(input_node):
	input_node.queue_free()
	new_morphology_node.erase(input_node)

func _on_mapping_def_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	if api_data != null and not api_data.has("Request failed..."):
		$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.clear()
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.clear()
		$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(" ")
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.add_item(" ")
		for i in api_data:
			$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(i)
			$".."/".."/".."/Menu/rule_properties/mapping_rule_options.add_item(i)
	$notification.generate_notification_message(api_data, _response_code, "_on_mapping_def_request_completed", "/v1/feagi/genome/morphology_list")

func _on_morphology_types_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	$".."/".."/".."/Menu/rule_properties/rules/rule_type_options.add_item(" ")
	if api_data != null:
		for i in api_data:
			$".."/".."/".."/Menu/rule_properties/rules/rule_type_options.add_item(i)
	$notification.generate_notification_message(api_data, _response_code, "_on_morphology_types_request_completed", "/v1/feagi/genome/morphology_types")


func _on_plus_add_pressed():
	var new_node = $".."/".."/".."/Menu/"Mapping_Properties"/inside_mapping_menu/Control.duplicate()
	$".."/".."/".."/Menu/"Mapping_Properties"/inside_mapping_menu.add_child(new_node)
	plus_node.append(new_node)
	new_node.get_child(0).connect("pressed",Callable(self,"_on_Mapping_def_pressed"))
	new_node.get_child(4).text_changed.connect(_on_text_changed.bind(new_node.get_child(4)))
	ghost_morphology.append(new_node.get_child(0))
	new_node.visible = true
	new_node.get_child(1).value = 1
	new_node.get_child(2).value = 1
	new_node.get_child(3).value = 1
	new_node.get_child(4).text = str(1)
	new_node.get_child(6).connect("pressed",Callable(self,"map_info_pressed").bind(new_node))
	new_node.get_child(7).connect("pressed",Callable(self,"remove_button_inside_dst").bind(new_node))
	new_node.position.y = (50 * (plus_node.size()))
	$".."/".."/".."/Menu/"Mapping_Properties"/inside_mapping_menu.size.y += (30 * plus_node.size())



func _on_type_rules_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	for i in $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_count():
		for x in api_data:
			if $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_text(i) == x:
				$".."/".."/".."/Menu/rule_properties/rules/rule_type_options.select(i)
	$notification.generate_notification_message(api_data, _response_code, "_on_type_rules_request_completed", "/v1/feagi/genome/morphology")

func _on_save_pressed():
	var json_data = {}
	var new_name = $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id())
	var new_type = $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_text($".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_selected_id())
	if new_type == "patterns":
		json_data["patterns"] = []
		var string_input = []
		var empty_array1 = []
		var empty_array2 = []
		var full_array = []
		var empty_flag = 0
		for i in new_morphology_node:
			empty_flag = 0
			full_array = []
			empty_array1 = []
			empty_array2 = []
			for _x in range(6):
				if not "?" in i.get_child(empty_flag).text and not "*" in i.get_child(empty_flag).text:
					if empty_flag < 3:
						empty_array1.append(int(i.get_child(empty_flag).text))
					elif empty_flag >= 3:
						empty_array2.append(int(i.get_child(empty_flag).text))
				else:
					if empty_flag < 3:
						empty_array1.append(str(i.get_child(empty_flag).text))
					elif empty_flag >= 3:
						empty_array2.append(str(i.get_child(empty_flag).text))
				empty_flag += 1
			full_array.append(empty_array1)
			full_array.append(empty_array2)
			string_input.append(full_array)
		json_data["patterns"] = string_input
		$".."/".."/".."/Menu/rule_properties.visible = false
		new_morphology_clear()
	if new_type == "vectors":
		json_data["vectors"] = []
		var empty_array1 = []
		for i in new_morphology_node:
			var temp_array = []
			temp_array = [i.get_child(0).value, i.get_child(1).value, i.get_child(2).value]
			empty_array1.append(temp_array)
		json_data["vectors"] = empty_array1
		new_morphology_clear()
	if new_type== "composite":
		json_data['src_pattern'] = []
		json_data['src_pattern'].append([$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/X_box/C.value, $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/X_box/S.value])
		json_data['src_pattern'].append([$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Y_box/C.value, $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Y_box/S.value])
		json_data['src_pattern'].append([$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Z_box/C.value, $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Z_box/S.value])
		json_data['src_seed'] = [$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/x.value, $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/y.value, $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/z.value]
		json_data["mapper_morphology"] = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.get_item_text($".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.selected)
	var combine_url = '/v1/feagi/genome/morphology?morphology_name=' + symbols_checker_for_api(new_name) + '&morphology_type=' + new_type
	Autoload_variable.BV_Core.PUT_Request_Brain_visualizer(SEC + combine_url, json_data)
	$".."/".."/".."/Menu/rule_properties.visible = false

#func _on_update_morphology_request_completed(_result, response_code, headers, body):
#	pass # Replace with function body.

func _on_delete_pressed():
	var grab_name_rule = $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id())
	grab_name_rule = symbols_checker_for_api(grab_name_rule)
	var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + grab_name_rule
	Autoload_variable.BV_Core.DELETE_Request_Brain_visualizer(combine_url)

func _on_get_cortical_dst_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	if _response_code == 200:
		var dst_data = {}
		for i in api_data["cortical_destinations"]:
			for x in Godot_list.genome_data["genome"]:
				for b in child_node_holder:
					if x == b.text:
						if i in Godot_list.genome_data["genome"][x][7]:
							dst_data[i] = api_data["cortical_destinations"][i]
		dst_data_holder = dst_data.duplicate()
		$".."/".."/".."/Menu/cortical_menu/Control/Update.position.y = 749 
		if $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text != "":
			var get_id = $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text
			Autoload_variable.BV_Core.Get_mem_data(get_id)
	$notification.generate_notification_message(api_data, _response_code, "_on_get_cortical_dst_request_completed", "/v1/feagi/genome/cortical_area")

func _on_cortical_mapping_add_pressed():
	_on_info_pressed() # leveraging the same function to clear all infos on the box
	$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.select(0)
	var add_flag = true
	for i in child_node_holder:
		if i.text == "":
			add_flag = false
	if add_flag:
		generate_cortical_mapping()


func generate_cortical_mapping():
	for i in $".."/".."/".."/Menu/Mapping_Properties/source_dropdown.get_item_count():
		if $".."/".."/".."/Menu/cortical_menu/Control/name_string.text == $".."/".."/".."/Menu/Mapping_Properties/source_dropdown.get_item_text(i):
			$".."/".."/".."/Menu/Mapping_Properties/source_dropdown.selected = i
	var counter = child_node_holder.size()
	var new_node = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.duplicate()
	$".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.add_child(new_node)
	child_node_holder.append(new_node)
	new_node.visible = true
#	new_node.text = new_name
	new_node.position.x = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.position.x
	new_node.position.y = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.position.y + (counter * 30)
	new_node.size.x = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.size.x 
	new_node.size.y = $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.size.y
	new_node.visible = true
	new_node.editable = true
	new_node.text = ""
	new_node.get_child(0).connect("pressed",Callable(self,"dst_remove_pressed").bind(new_node))
	new_node.get_child(1).connect("pressed",Callable(self,"info_pressed").bind(new_node))
	counter += 1

func name_to_id(name_input):
	var iteration_name = name_input
	var grab_id_cortical = ""
	for i in Godot_list.genome_data["genome"]:
		if i == iteration_name:
			grab_id_cortical = Godot_list.genome_data["genome"][i][7]
			return grab_id_cortical
			
func id_to_name(name_input):
	for x in Godot_list.genome_data["genome"]:
		if Godot_list.genome_data["genome"][x][7] == name_input:
			return x

func _on_menu_pressed():
	$".."/".."/".."/Menu/information_menu/cortical_cam_label/menu.text = ""
	if $".."/".."/".."/Menu/information_menu/cortical_cam_label/menu_itemlist.visible:
		$".."/".."/".."/Menu/information_menu/cortical_cam_label/menu_itemlist.visible = false
	else:
		$".."/".."/".."/Menu/information_menu/cortical_cam_label/menu_itemlist.visible = true
		$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.load_options()
		$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.release_focus()

func _on_cortical_dropdown_pressed():
	$".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.load_options()

func _on_burst_value_text_entered(new_text):
	var json = {}
	if new_text == "0" or new_text == "":
		json["burst_duration"] = float(1/float(1))
	else:
		json["burst_duration"] = float(1/float(new_text))
	Autoload_variable.BV_Core.Update_BurstRate(json["burst_duration"])
	$".."/".."/".."/Menu/information_menu/burst_duration_label/burst_value.release_focus()

func _on_burst_value_focus_exited():
	var json = {}
	var new_text = $".."/".."/".."/Menu/information_menu/burst_duration_label/burst_value.text
	if new_text == "0" or new_text == "":
		json["burst_duration"] = float(1/float(1))
	else:
		json["burst_duration"] = float(1/float(new_text))
	Autoload_variable.BV_Core.Update_BurstRate(json["burst_duration"])
	$".."/".."/".."/Menu/information_menu/burst_duration_label/burst_value.release_focus()

func _on_Neuron_morphologies_button_pressed():
	if $".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.visible:
		$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.visible = false
	else:
		$".."/".."/".."/Menu/rule_properties.visible = false
		$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.visible = true
		Autoload_variable.BV_Core.Update_MorphologyList()

func _on_morphology_list_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.clear()
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.clear()
	$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.clear()
	$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.clear()
	$".."/".."/".."/Menu/Control/inner_box/box_of_composite/mapper_composite.clear()
	$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.add_item(" ")
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.add_item(" ")
	$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.add_item(" ")
	$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(" ")
	$".."/".."/".."/Menu/Control/inner_box/box_of_composite/mapper_composite.add_item(" ")
	for i in api_data:
		$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(i)
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.add_item(i)
		$".."/".."/".."/Menu/Control/inner_box/box_of_composite/mapper_composite.add_item(i)
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.add_item(i)
		$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.add_item(i, null, true)
	$notification.generate_notification_message(api_data, _response_code, "_on_morphology_list_request_completed", "/v1/feagi/genome/morphology_list")

func _on_Neuron_morphologies_item_selected(index):
	if index != 0:
		$".."/".."/".."/Menu/rule_properties.visible = true
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.selected = index
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.emit_signal("item_selected", index)
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.release_focus()

func _on_Button_pressed():
	$".."/".."/".."/Menu/Control/inner_box/morphology_type.clear()
	for i in $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_count():
		if $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_text(i) != "functions":
			$".."/".."/".."/Menu/Control/inner_box/morphology_type.add_item($".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_text(i))
	$".."/".."/".."/Menu/Control.visible = true
	if $".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.get_item_count() == 0:
		Autoload_variable.BV_Core.Update_MorphologyList()

func _on_create_pressed():
	if $".."/".."/".."/Menu/Control/inner_box/morphology_name.text != "":
		if $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.selected) == "patterns":
			var json = {}
			var new_name = symbols_checker_for_api($".."/".."/".."/Menu/Control/inner_box/morphology_name.text)
			var new_type = $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.get_selected_id())
			var string_input = []
			var empty_array1 = []
			var empty_array2 = []
			var full_array = []
			var empty_flag = 0
			for i in new_morphology_node:
				empty_flag = 0
				full_array = []
				empty_array1 = []
				empty_array2 = []
				for _x in range(6):
					if not "?" in i.get_child(empty_flag).text and not "*" in i.get_child(empty_flag).text:
						if empty_flag < 3:
							empty_array1.append(int(i.get_child(empty_flag).text))
						elif empty_flag >= 3:
							empty_array2.append(int(i.get_child(empty_flag).text))
					else:
						if empty_flag < 3:
							empty_array1.append(str(i.get_child(empty_flag).text))
						elif empty_flag >= 3:
							empty_array2.append(str(i.get_child(empty_flag).text))
					empty_flag += 1
				full_array.append(empty_array1)
				full_array.append(empty_array2)
				string_input.append(full_array)
			json["patterns"] = string_input
			var combine_url = '/v1/feagi/genome/morphology?morphology_name=' + new_name + '&morphology_type=' + new_type
			Autoload_variable.BV_Core.POST_Request_Brain_visualizer(SEC+combine_url, json)
			$".."/".."/".."/Menu/Control.visible = false
			new_morphology_clear()
			$".."/".."/".."/Menu/Control/inner_box/morphology_name.text = ""
		if $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.selected) == "vectors":
			var json = {}
			var new_name = symbols_checker_for_api($".."/".."/".."/Menu/Control/inner_box/morphology_name.text)
			var new_type = $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.get_selected_id())
			var empty_array1 = []
			for i in new_morphology_node:
				var temp_array = []
				temp_array.append(i.get_child(0).value)
				temp_array.append(i.get_child(1).value)
				temp_array.append(i.get_child(2).value)
				empty_array1.append(temp_array)
			json["vectors"] = empty_array1
			var combine_url = '/v1/feagi/genome/morphology?morphology_name=' + new_name + '&morphology_type=' + new_type
			Autoload_variable.BV_Core.POST_Request_Brain_visualizer(SEC+combine_url, json)
			$".."/".."/".."/Menu/Control.visible = false
			new_morphology_clear()
			$".."/".."/".."/Menu/Control/inner_box/morphology_name.text = ""
		if $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.selected) == "composite":
			var json = {}
			var new_name = symbols_checker_for_api($".."/".."/".."/Menu/Control/inner_box/morphology_name.text)
			var new_type = $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.get_selected_id())
			json["src_seed"] = [$".."/".."/".."/Menu/Control/inner_box/box_of_composite/X.value, $".."/".."/".."/Menu/Control/inner_box/box_of_composite/Y.value, $".."/".."/".."/Menu/Control/inner_box/box_of_composite/Z.value]
			json["src_pattern"] = [[$".."/".."/".."/Menu/Control/inner_box/box_of_composite/X_box/C.value, $".."/".."/".."/Menu/Control/inner_box/box_of_composite/X_box/S.value], [$".."/".."/".."/Menu/Control/inner_box/box_of_composite/Y_box/C.value, $".."/".."/".."/Menu/Control/inner_box/box_of_composite/Y_box/S.value], [$".."/".."/".."/Menu/Control/inner_box/box_of_composite/Z_box/C.value, $".."/".."/".."/Menu/Control/inner_box/box_of_composite/Z_box/S.value]]
			json["mapper_morphology"] = $".."/".."/".."/Menu/Control/inner_box/box_of_composite/mapper_composite.get_item_text($".."/".."/".."/Menu/Control/inner_box/box_of_composite/mapper_composite.selected)	
			var combine_url = '/v1/feagi/genome/morphology' + '?morphology_name=' + new_name + '&morphology_type=' + new_type
			Autoload_variable.BV_Core.POST_Request_Brain_visualizer(SEC+combine_url, json)
			$".."/".."/".."/Menu/Control.visible = false
			new_morphology_clear()
			$".."/".."/".."/Menu/Control/inner_box/morphology_name.text = ""
func _on_X_inside_inner_box_pressed():
	$".."/".."/".."/Menu/Control.visible = false
	new_morphology_clear()
	$".."/".."/".."/Menu/Control/inner_box/morphology_name.text = ""

func _on_afferent_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	var counter = 0
	afferent_holder_clear()
	for i in api_data:
		var new_node = $".."/".."/".."/Menu/cortical_mapping/Control/afferent/VBoxContainer/LineEdit.duplicate()
		$".."/".."/".."/Menu/cortical_mapping/Control/afferent/VBoxContainer.add_child(new_node)
		afferent_child_holder.append(new_node)
		new_node.visible = true
		new_node.text = id_to_name(i)
		new_node.visible = true
		new_node.position.x = $".."/".."/".."/Menu/cortical_mapping/Control/afferent/VBoxContainer.position.x + 5
		new_node.position.y = $".."/".."/".."/Menu/cortical_mapping/Control/afferent/VBoxContainer.position.y + (counter * 20)
		counter += 1
	$notification.generate_notification_message(api_data, _response_code, "_on_afferent_request_completed", "/v1/feagi/genome/cortical_mappings/afferents")

func afferent_holder_clear():
	# Clear duplicate cortical maps name up
	if afferent_child_holder:
		for i in afferent_child_holder:
			i.queue_free()
		afferent_child_holder = []

func new_morphology_clear():
	if new_morphology_node:
		for i in new_morphology_node:
			i.queue_free()
		new_morphology_node = []
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/Label2.position.y = 160
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.position.y = 180
		$".."/".."/".."/Menu/rule_properties/rules.size.y = 436
		$".."/".."/".."/Menu/rule_properties.size.y = 608
		$".."/".."/".."/Menu/rule_properties/rules/save.position.y = 477.001
		$".."/".."/".."/Menu/rule_properties/rules/delete.position.y = 477.001
		$".."/".."/".."/Menu/Control/inner_box/grey_bg.size.y = 283
		$".."/".."/".."/Menu/Control/ColorRect.size.y = 394
		$".."/".."/".."/Menu/Control/update.position.y = 330
		$".."/".."/".."/Menu/Control/inner_box/Button.disabled = false

func plus_node_clear():
	for i in plus_node:
		i.queue_free()
	plus_node.clear()

func ghost_morphology_clear():
	for i in ghost_morphology:
		if is_instance_valid(i):
			i.queue_free()
	ghost_morphology.clear()

func _on_cortical_dropdown_item_selected(index):
	if index != 0:
		if $".."/".."/".."/Menu/Mapping_Properties/source_dropdown.selected != 0:
			var combine_url = '#&dst_cortical_area=$'.replace("#", name_to_id($".."/".."/".."/Menu/Mapping_Properties/source_dropdown.get_item_text($".."/".."/".."/Menu/Mapping_Properties/source_dropdown.get_selected_id())))
			combine_url= combine_url.replace("$", name_to_id($".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.get_item_text(index)))
			Autoload_variable.BV_Core.Update_destination(combine_url)

func _on_update_inside_map_pressed():
	var combine_url = '?src_cortical_area=#&dst_cortical_area=$'
	var get_id = name_to_id($".."/".."/".."/Menu/Mapping_Properties/source_dropdown.text)
	var get_dst_data = name_to_id($".."/".."/".."/Menu/Mapping_Properties/cortical_dropdown.text)
	var dst_data = {}
	dst_data["cortical_destinations"] = {}
	combine_url = combine_url.replace("#", get_id)
	combine_url = combine_url.replace("$", get_dst_data)
	for p in plus_node:
		var dst = {}
		dst["morphology_id"] = p.get_child(0).get_item_text(p.get_child(0).get_selected_id())
		dst["morphology_scalar"] = []
		dst["morphology_scalar"].append(p.get_child(1).value)
		dst["morphology_scalar"].append(p.get_child(2).value)
		dst["morphology_scalar"].append(p.get_child(3).value)
		dst["postSynapticCurrent_multiplier"] = float(p.get_child(4).text)
		dst["plasticity_flag"] = p.get_child(5).is_pressed()
		if dst_data["cortical_destinations"].has(get_id):
			dst_data["cortical_destinations"][get_id].append(dst)
		else:
			dst_data["cortical_destinations"][get_id] = []
			dst_data["cortical_destinations"][get_id].append(dst)
	if dst_data["cortical_destinations"].has(get_id):
		Autoload_variable.BV_Core.Update_Mapping_Properties(dst_data["cortical_destinations"][get_id],combine_url)
	else:
		Autoload_variable.BV_Core.Update_Mapping_Properties([],combine_url)

#func map_colorful():
#	pass
#	for i in child_node_holder:
#		for x in global_name_list:
#			if i.text in x:
#				if not(i.text) in x[0]:
##					print(i.text + "_textbox")
#					print(x)
#			if i.text == global_name_list[x]:
#				print("MATCHED!")
#				print("here: ", x)

func _on_mem_pressed():
	var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/monitoring/neuron/membrane_potential?cortical_area=' + $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text + '&state=' + str($".."/".."/".."/Menu/button_choice/Control/mem.is_pressed())
	Autoload_variable.BV_Core.POST_Request_Brain_visualizer(combine_url,$".."/".."/".."/Menu/button_choice/Control/mem.is_pressed())

func _on_mem_request_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	var get_id = $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text
	$".."/".."/".."/Menu/button_choice/Control/mem.set_pressed(api_data)
	Autoload_variable.BV_Core.Get_syn_data(get_id)
	$notification.generate_notification_message(api_data, _response_code, "_on_mem_request_request_completed", "/v1/feagi/monitoring/neuron/membrane_potential")

func _on_syn_request_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	$".."/".."/".."/Menu/button_choice/Control/syn.set_pressed(api_data)
	$notification.generate_notification_message(api_data, _response_code, "_on_syn_request_request_completed", "/v1/feagi/monitoring/neuron/synaptic_potential")

func _on_syn_pressed():
	var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/monitoring/neuron/synaptic_potential?cortical_area=' + $".."/".."/".."/Menu/cortical_menu/Control/cortical_id.text + '&state=' + str($".."/".."/".."/Menu/button_choice/Control/syn.is_pressed())
	Autoload_variable.BV_Core.POST_Request_Brain_visualizer(combine_url,$".."/".."/".."/Menu/button_choice/Control/syn.is_pressed())
	
func _on_insert_button_pressed():
	$".."/".."/".."/Menu/insert_menu/inner_box.visible = true
	var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/append?circuit_name=' + $".."/".."/".."/Menu/insert_menu/inner_box/name_text.text + "&circuit_origin_x=" + str($".."/".."/".."/Menu/insert_menu/x_spinbox.value) + "&circuit_origin_y=" + str($".."/".."/".."/Menu/insert_menu/y_spinbox.value) + "&circuit_origin_z=" + str($".."/".."/".."/Menu/insert_menu/z_spinbox.value)
	var new_data = ["placeholder"]
	Autoload_variable.BV_Core.POST_Request_Brain_visualizer(combine_url, new_data)

func _on_circuit_request_request_completed(_result, _response_code, _headers, body):
	$".."/".."/".."/Menu/insert_menu/insert_button/ItemList.clear()
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	for i in api_data:
		$".."/".."/".."/Menu/insert_menu/insert_button/ItemList.add_item(i, null, true)
	$notification.generate_notification_message(api_data, _response_code, "_on_circuit_request_request_completed", "/v1/feagi/genome/circuits")

func _on_import_pressed():
	var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/circuits'
	Autoload_variable.BV_Core.Get_circuit_list()

func _on_ItemList_item_selected(index):
	var name_text = $".."/".."/".."/Menu/insert_menu/insert_button/ItemList.get_item_text(index)
	name_text = symbols_checker_for_api(name_text)
	$".."/".."/".."/Menu/insert_menu/inner_box.visible = true
	$".."/".."/".."/Menu/insert_menu/inner_box/name_text.text = name_text
	var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/circuit_size?circuit_name=' + name_text
	$HTTP_node/circuit_size.request(combine_url)

func _on_circuit_size_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	$".."/".."/".."/Menu/insert_menu/inner_box/W_spinbox.value = api_data[0]
	$".."/".."/".."/Menu/insert_menu/inner_box/D_spinbox.value = api_data[1]
	$".."/".."/".."/Menu/insert_menu/inner_box/H_spinbox.value = api_data[2]
	generate_single_cortical($".."/".."/".."/Menu/insert_menu/x_spinbox.value, $".."/".."/".."/Menu/insert_menu/y_spinbox.value, $".."/".."/".."/Menu/insert_menu/z_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/W_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/D_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/H_spinbox.value, "example")
	$notification.generate_notification_message(api_data, _response_code, "_on_circuit_size_request_completed", "/v1/feagi/genome/circuit_size")

func symbols_checker_for_api(string_data):
	if " " in string_data:
		string_data = string_data.replace(" ", "%20")
	if "(" in string_data:
		string_data = string_data.replace("(", "%28")
	if ")" in string_data:
		string_data = string_data.replace(")", "%29")
	if "+" in string_data:
		string_data = string_data.replace("+", "%2B")
	return string_data

func _on_x_spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/insert_menu/x_spinbox.value, $".."/".."/".."/Menu/insert_menu/y_spinbox.value, $".."/".."/".."/Menu/insert_menu/z_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/W_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/D_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/H_spinbox.value, "example")

func _on_y_spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/insert_menu/x_spinbox.value, $".."/".."/".."/Menu/insert_menu/y_spinbox.value, $".."/".."/".."/Menu/insert_menu/z_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/W_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/D_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/H_spinbox.value, "example")

func _on_z_spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/insert_menu/x_spinbox.value, $".."/".."/".."/Menu/insert_menu/y_spinbox.value, $".."/".."/".."/Menu/insert_menu/z_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/W_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/D_spinbox.value, $".."/".."/".."/Menu/insert_menu/inner_box/H_spinbox.value, "example")


func _on_Neuron_morphologies_item_item_selected(index):
	$".."/".."/".."/Menu/rule_properties.visible = true
	$".."/".."/".."/Menu/information_menu/Neuron_morphologies_button.text = $".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.get_item_text(index)
	$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.visible = false
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.selected = index
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.emit_signal("item_selected", index)
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.release_focus()

func _on_Mapping_def_pressed():
	Autoload_variable.BV_Core.Update_MorphologyList()

func _on_ghost_morphology_list_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.clear()
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.clear()
	$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.clear()
	$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.add_item(" ")
	$".."/".."/".."/Menu/rule_properties/mapping_rule_options.add_item(" ")
	$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(" ")
	for i in api_data:
		$".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(i)
		$".."/".."/".."/Menu/rule_properties/mapping_rule_options.add_item(i)
		$".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.add_item(i, null, true)
	if ghost_morphology:
		for a in ghost_morphology:
			var node_ghost = a
			if node_ghost.get_item_count() != $".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.get_item_count():
				node_ghost.clear()
				for i in $".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.get_item_count():
					node_ghost.add_item($".."/".."/".."/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.get_item_text(i), i)
	$notification.generate_notification_message(api_data, _response_code, "_on_circuit_size_request_completed", "/v1/feagi/genome/circuit_size")

func _on_text_changed(new_text, node_input):
	Godot_list.Node_2D_control = true
	if new_text != "":
		if new_text.is_valid_int():
			node_input.value = int(new_text)
		elif new_text == "-":
			node_input.value = int(new_text)
		else:
			node_input.delete_char_at_caret()

func _on_get_morphology_usuage_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	var string_list = ""
	$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.text = ""
	if api_data != null:
		for i in api_data:
			string_list = string_list + str(id_to_name(i[0]), " > ", id_to_name(i[1])) + "\n"
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.text += str(string_list)
	$notification.generate_notification_message(api_data, _response_code, "_on_get_morphology_usuage_request_completed", "/v1/feagi/genome/morphology")

func _morphology_button_pressed():
	var counter = 0
	counter = len(new_morphology_node)
	if $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.selected) == "patterns":
		var new_node = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.duplicate()
		$".."/".."/".."/Menu/Control/inner_box/box_of_pattern.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(7).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.position.x = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.position.x + $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.size.x
		new_node.position.y = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.position.y + (30 * counter)
	elif $".."/".."/".."/Menu/Control/inner_box/morphology_type.get_item_text($".."/".."/".."/Menu/Control/inner_box/morphology_type.selected) == "vectors":
		var new_node = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.duplicate()
		new_morphology_node.append(new_node)
		$".."/".."/".."/Menu/Control/inner_box/box_of_vectors.add_child(new_node)
		new_node.visible = true
		new_node.get_child(3).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.size = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.size
		new_node.position.x = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.position.x
		new_node.position.y = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.position.y + (30 * counter)
	if counter > 4:
		$".."/".."/".."/Menu/Control/inner_box/grey_bg.size.y += 35
		$".."/".."/".."/Menu/Control/create.position.y += 35
		$".."/".."/".."/Menu/Control/ColorRect.size.y += 35
	else:
		$".."/".."/".."/Menu/Control/create.position = Vector2(152, 330)
		$".."/".."/".."/Menu/Control/ColorRect.size = Vector2(519, 394)


func _on_morphology_name_focus_exited():
	new_morphology_clear()
	for i in $".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.get_item_count():
		var name_morphology = $".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.get_item_text(i) 
		if $".."/".."/".."/Menu/information_menu/Neuron_morphologies_item.get_item_text(i) == $".."/".."/".."/Menu/Control/inner_box/morphology_name.text:
			if "+" in name_morphology:
				name_morphology = name_morphology.replace("+", "%2B")
			if "[" in name_morphology:
				name_morphology = name_morphology.replace("[", "%5B")
			if "]" in name_morphology:
				name_morphology = name_morphology.replace("]", "%5D")
			if ", " in name_morphology:
				name_morphology = name_morphology.replace(", ", "%2C%20")
			var combine_url = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + name_morphology
			Autoload_variable.BV_Core.Get_Morphology_information(name_morphology)

func _on_get_morphology_request_completed(_result, _response_code, _headers, body):
	new_morphology_clear()
	flag = false
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8()) # Every http data, it's done in poolbytearray
	var api_data = test_json_conv.get_data()
	var new_name = ""
	var type_name = " "
	var counter = 0
	for i in api_data:
		if i == "type":
			for x in api_data["parameters"]:
				new_name = str(api_data[i])
				if x == "patterns":
					type_name = x
					for a in api_data["parameters"]["patterns"]:
						counter = len(new_morphology_node)
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = true
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = false
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label.visible = false
						var new_node = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.duplicate()
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition.add_child(new_node)
						new_morphology_node.append(new_node)
						new_node.visible = true
						new_node.get_child(6).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
						new_node.position.x = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.position.x
						new_node.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.position.y + (25 * counter)
						if len(a) == 2:
							new_node.get_child(0).text = str(a[0][0])
							new_node.get_child(1).text = str(a[0][1])
							new_node.get_child(2).text = str(a[0][2])
							new_node.get_child(3).text = str(a[1][0])
							new_node.get_child(4).text = str(a[1][1])
							new_node.get_child(5).text = str(a[1][2])
						else:
							print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
							print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id()))
							break
				elif x == "vectors":
					type_name = x
					for a in api_data["parameters"][x]:
						counter = len(new_morphology_node)
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label.visible = false
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = false
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = true
						var new_node = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.duplicate()
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label.add_child(new_node)
						new_morphology_node.append(new_node)
						new_node.visible = true
						new_node.get_child(3).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
						new_node.size = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.size
						new_node.position.x = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.position.x
						new_node.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.position.y + (30 * counter)
						if typeof(a) == 28:
							if len(a) == 3:
								new_node.get_child(0).value = int(a[0])
								new_node.get_child(1).value = int(a[1])
								new_node.get_child(2).value = int(a[2])
							else:
								print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
								print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id()))
								break
						else:
							print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
							print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id()))
							break
					if len(new_morphology_node) > 4:
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/Label2.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/Label2.position.y + (4 * counter)
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.position.y + (4 * counter)
						$".."/".."/".."/Menu/rule_properties/rules.size.y += (counter * 4)
						$".."/".."/".."/Menu/rule_properties.size.y += (counter * 4)
				elif x == "src_seed": # composite
					type_name = api_data[i]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = false
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = false
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label.visible = true
					for a in $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.get_item_count():
						if api_data["parameters"]["mapper_morphology"] == $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.get_item_text(a):
							$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.select(a)
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/X_box/C.value = api_data['parameters']['src_pattern'][0][0]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/X_box/S.value = api_data['parameters']['src_pattern'][0][1]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Y_box/C.value = api_data['parameters']['src_pattern'][1][0]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Y_box/S.value = api_data['parameters']['src_pattern'][1][1]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Z_box/C.value = api_data['parameters']['src_pattern'][2][0]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/Z_box/S.value = api_data['parameters']['src_pattern'][2][1]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/x.value = api_data['parameters']['src_seed'][0]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/y.value = api_data['parameters']['src_seed'][1]
					$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/composite_label/z.value = api_data['parameters']['src_seed'][2]
			counter = 0
			if $".."/".."/".."/Menu/Control.visible:
				for x in api_data[i]:
					if i == "patterns":
						counter += 1
						var new_node = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.duplicate()
						$".."/".."/".."/Menu/Control/inner_box/box_of_pattern.add_child(new_node)
						new_morphology_node.append(new_node)
						new_node.visible = true
						new_node.get_child(7).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
						new_node.position.x = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.position.x + $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.size.x
						new_node.position.y = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.position.y + (30 * counter)
						if len(x) == 2:
							new_node.get_child(0).text = str(x[0][0])
							new_node.get_child(1).text = str(x[0][1])
							new_node.get_child(2).text = str(x[0][2])
							new_node.get_child(3).text = str(x[1][0])
							new_node.get_child(4).text = str(x[1][1])
							new_node.get_child(5).text = str(x[1][2])
						else:
							print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
							print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/Control/inner_box/morphology_name.text)
							break
					elif i == "vectors":
						counter += 1
						var new_node = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.duplicate()
						$".."/".."/".."/Menu/Control/inner_box/box_of_vectors.add_child(new_node)
						new_morphology_node.append(new_node)
						new_node.visible = true
						new_node.get_child(3).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
						new_node.position.x = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.position.x
						new_node.position.y = $".."/".."/".."/Menu/Control/inner_box/box_of_vectors/Control.position.y + (30 * counter)
						new_node.get_child(0).value = int(x[0])
						new_node.get_child(1).value = int(x[1])
						new_node.get_child(2).value = int(x[2])
						if len(new_morphology_node) > 4:
							$".."/".."/".."/Menu/Control/inner_box/grey_bg.size.y += (4 * counter)
							$".."/".."/".."/Menu/Control/ColorRect.size.y += (4 * counter)
							$".."/".."/".."/Menu/Control/update.position.y += (4 * counter)
						$".."/".."/".."/Menu/Control/inner_box/Button.disabled = true

			for x in $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_count():
				if $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_text(x) == type_name:
					$".."/".."/".."/Menu/rule_properties/rules/rule_type_options.selected = x
					if "*" in new_name:
						new_name = new_name.replace("*", "\""+"*"+"\"")
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/morphology_def.text = new_name
						flag = true
					if "?" in new_name:
						flag = true
						new_name = new_name.replace("?", "\""+"?"+"\"")
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/morphology_def.text = new_name
					if flag == false:
						$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/morphology_def.text = new_name
	$notification.generate_notification_message(api_data, _response_code, "_on_get_morphology_request_completed", "/v1/feagi/monitoring/neuron/synaptic_potential")


func _morphology_button_inside_red():
	var i = $".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_item_text($".."/".."/".."/Menu/rule_properties/rules/rule_type_options.get_selected_id())
	var counter = 0
	if i == "patterns":
		counter = len(new_morphology_node)
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = true
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = false
		var new_node = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.duplicate()
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(6).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.position.x = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.position.x
		new_node.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.position.y + (25 * counter)
		new_node.get_child(0).text = ""
		new_node.get_child(1).text = ""
		new_node.get_child(2).text = ""
		new_node.get_child(3).text = ""
		new_node.get_child(4).text = ""
		new_node.get_child(5).text = ""
	elif i == "vectors":
		counter = len(new_morphology_node)
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = false
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = true
		var new_node = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.duplicate()
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(3).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.size = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.size
		new_node.position.x = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.position.x
		new_node.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.position.y + (30 * counter)
		new_node.get_child(0).value = 0
		new_node.get_child(1).value = 0
		new_node.get_child(2).value = 0
	if counter > 4:
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/Label2.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/Label2.position.y + (3 * counter)
		$".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.position.y = $".."/".."/".."/Menu/rule_properties/rules/morphology_definition/associations_data.position.y + (3 * counter)
		$".."/".."/".."/Menu/rule_properties/rules.size.y += (counter * 3)
		$".."/".."/".."/Menu/rule_properties.size.y += (counter * 3)
		$".."/".."/".."/Menu/rule_properties/rules/save.position.y -= 2
		$".."/".."/".."/Menu/rule_properties/rules/delete.position.y -=2

func _on_close_pressed_def():
	plus_node_clear()
	ghost_morphology_clear()


func _on_morphology_type_item_selected(_index):
	if len(new_morphology_node) > 0:
		new_morphology_clear()
		_morphology_button_pressed()
	else:
		_morphology_button_pressed()


func _on_menu_itemlist_item_selected(index):
	$Node3D/Camera3D._on_menu_itemlist_item_selected(index)


func _on_grab_location_of_cortical_request_completed(result, response_code, headers, body):
	$Node3D/Camera3D._on_grab_location_of_cortical_request_completed(result, response_code, headers, body)

func _on_X_SpinBox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_W_Spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_H_Spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_D_Spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_Y_Spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_Z_Spinbox_value_changed(_value):
	generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func demo_new_cortical():
	"""
	This is for add new cortical area so the name will be updated when you move it around. This is designed to use
	the duplicated node called "example", so if it has no name, it will display as "example" but if 
	it has a letter or name, it will display as the user typed.
	"""
	for i in len(global_name_list):
		if "example" in global_name_list[i]:
			if global_name_list[i]["example"][0].get_child(0).get_class() == "Viewport":
				if $".."/".."/".."/Menu/addition_menu/addition_menu/cortical_name_textbox/type.text == "":
					global_name_list[i]["example"][0].get_child(0).get_child(0).text = "example"
				else:
					global_name_list[i]["example"][0].get_child(0).get_child(0).text = $".."/".."/".."/Menu/addition_menu/addition_menu/cortical_name_textbox/type.text
