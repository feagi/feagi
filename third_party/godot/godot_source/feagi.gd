"""
==============================================================================
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
var total = []
var array_test = []
var x_increment = 0
var y_increment = 0
var z_increment = 0
var one_time_flag = true
var stored_csv = ""
var genome_data = {}
var previous_genome_data = ""
var global_name_list = []
var last_cortical_selected
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



func _ready():
	set_physics_process(false)
	add_3D_indicator()
	genome_data["genome"] = {}
	$HTTP_node/morphology_types.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_types')
	$HTTP_node/genome_data.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/connectome/properties/dimensions')
	while true:
		if $Spatial/Camera/Menu/box_loading.visible:
			$Spatial/Camera/Menu/box_loading.visible = false
		if $Spatial/Camera/Menu/insert_menu.visible == false:
			_clear_single_cortical("example", global_name_list)
#		if not($Spatial/Camera/Menu/cortical_menu.visible) and child_node_holder:
#			child_holder_clear() # Is this needed now?
		if cortical_is_clicked():
			$Spatial/Camera/Menu/cortical_menu.visible = true
			$Spatial/Camera/Menu/cortical_mapping.visible = true
			$Spatial/Camera/Menu/button_choice.visible = true
			$Spatial/Camera/Menu/properties.visible = true
			$Spatial/Camera/Menu/Mapping_Properties.visible = false
#			$Spatial/Camera/Menu/collapse_1.visible = true
#			$Spatial/Camera/Menu/collapse_2.visible = true
#			$Spatial/Camera/Menu/collapse_3.visible = true
			$Spatial/Camera/Menu/collapse_4.visible = true
			$Spatial/Camera/Menu/cortical_menu/title.visible = true
		elif select_cortical.selected.empty() != true:
			select_cortical.selected.pop_front()
		_process(self)
#		print("FROM PYTHON: ", data)
		if "update" in data:
			$Spatial/Camera/Menu/box_loading.visible = true
			if timer_api.bool_flag:
				timer_api.trigger_api_timer()
				$HTTP_node/genome_data.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/connectome/properties/dimensions')
				stored_value= ""
		else:
			stored_value = data
		start = OS.get_ticks_msec()## This will time the engine at start
		yield(get_tree().create_timer(0.01), "timeout")
		end = OS.get_ticks_msec()
		var time_total = end - start
		if time_total < 500: ## Generate voxels as long as you are on the tab
			generate_voxels()
		else:
			network_setting.send("lagged")

func _process(_delta):
	data = network_setting.one_frame

func generate_one_model(node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	var new = get_node("Cortical_area").duplicate()
	new.set_name(name_input)
	global_name_list.append({name_input.replace(" ", "") : [new, x_input, y_input, z_input, width_input, depth_input, height_input]})
	add_child(new)
	new.visible = true
	new.scale = Vector3(width_input, height_input, depth_input)
	new.transform.origin = Vector3(width_input/2 + int(x_input), height_input/2+ int(y_input), depth_input/2 + int(z_input))
	generate_textbox(node, x_input,height_input,z_input, name_input, y_input, width_input)

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
	for key in Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"]:
		Godot_list.godot_list["\'data\'"]["\'direct_stimulation\'"][key] = []
	_clear_node_name_list(global_name_list)
	for k in genome_data["genome"]:
		var CSV_data = genome_data["genome"][k]
		var x = CSV_data[0]; var y = CSV_data[1]; var z = CSV_data[2]; var width= int(CSV_data[4]) 
		var height = int(CSV_data[5]); var depth = int(CSV_data[6]); var name_input = k
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
	$HTTP_node/get_genome_name.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/file_name')

func _clear_node_name_list(node_name):
	"""
	clear all cortical area along with the library list/dict
	"""
	var list = node_name
	if list.empty() != true:
		var list_size = global_name_list.size()
		for i in list_size:
			for iteration_name in global_name_list[i]:
				global_name_list[i][iteration_name][0].queue_free()
		global_name_list = []
	$Floor_grid.clear()

func _clear_single_cortical(cortical_name, node_list):
	var list = node_list
	if list.empty() != true:
		for search_name in list:
			if cortical_name in search_name:
				search_name[cortical_name][0].queue_free()
		erase_single_cortical_library(cortical_name)
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
		dst_data_holder = {}
		var iteration_name = select_cortical.selected[0].replace("'","")
		var grab_id_cortical = ""
		for i in genome_data["genome"]:
			if i == iteration_name:
				grab_id_cortical = genome_data["genome"][i][7]
				break
		update_cortical_map_name(grab_id_cortical)
		var combine_name = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area?cortical_area=' + grab_id_cortical
		$HTTP_node/HTTPRequest.request(combine_name)
		$HTTP_node/get_cortical_dst.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area?cortical_area=' + grab_id_cortical)
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
	var viewport = create_textbox.get_node("Viewport")
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
	var x = int($Spatial/Camera/Menu/cortical_menu/Control/X.value);
	var y = int($Spatial/Camera/Menu/cortical_menu/Control/Y.value);
	var z = int($Spatial/Camera/Menu/cortical_menu/Control/Z.value);
	var id_input = str($Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text)
	var width= int($Spatial/Camera/Menu/cortical_menu/Control/W.value)
	var height = int($Spatial/Camera/Menu/cortical_menu/Control/H.value);
	var depth = int($Spatial/Camera/Menu/cortical_menu/Control/D.value);

	var synaptic_attractivity = int($Spatial/Camera/Menu/properties/Control/syn.value);
	var post_synaptic_potential = $Spatial/Camera/Menu/properties/Control/pst_syn.value;
	var post_synaptic_potential_max = float($Spatial/Camera/Menu/properties/Control/pst_syn_max.value);
	var plasticity_coef = float($Spatial/Camera/Menu/properties/Control/plst.value);
	var fire_threshold = float($Spatial/Camera/Menu/properties/Control/fire.value);
	var refractory_period = int($Spatial/Camera/Menu/properties/Control/refa.value);
	var leak_coefficient = float($Spatial/Camera/Menu/properties/Control/leak.text);
	var leak_variability = float($Spatial/Camera/Menu/properties/Control/leak_Vtext.text);
	var consecutive_fire_count = int($Spatial/Camera/Menu/properties/Control/cfr.value);
	var snooze_period = int($Spatial/Camera/Menu/properties/Control/snze.value);
	var degenerecy_coefficient = float($Spatial/Camera/Menu/properties/Control/dege.value);
	var psp_uniform_distribution = $Spatial/Camera/Menu/properties/Control/psud.is_pressed()
	var name_input = $Spatial/Camera/Menu/cortical_menu/Control/name_string.text
	var copy = duplicate_model.duplicate()
	var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox.get_node("Viewport")
	var store_global_data = []

	create_textbox.set_texture(viewport.get_texture())
	add_child(copy)
	global_name_list.append({name_input.replace(" ", "").replace(" ", "") : [copy, x, y, z, width, depth, height]})
	create_textbox.set_name(name_input + "_textbox")
	add_child(create_textbox) # Copied the node to new node
	create_textbox.scale = Vector3(1,1,1)
	
	

	last_cortical_selected["cortical_destinations"] = {}
	var cortical_name = $Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_text($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_selected_id())
	var get_id = name_to_id(cortical_name)
	for i in child_node_holder:
		if i.text == "":
			if $Spatial/Camera/Menu/Mapping_Properties.visible:
				if $Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_selected_id() != 0:
					i.text = $Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_text($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_selected_id())
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
	last_cortical_selected["cortical_neuron_per_vox_count"] = $Spatial/Camera/Menu/properties/Control/neuron_count.value
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
	last_cortical_selected["neuron_leak_coefficient"] = float(leak_coefficient)
	last_cortical_selected["neuron_leak_variability"] = float(leak_variability)
	last_cortical_selected["neuron_consecutive_fire_count"] = consecutive_fire_count
	last_cortical_selected["neuron_snooze_period"] = snooze_period
	last_cortical_selected["neuron_degeneracy_coefficient"] = degenerecy_coefficient
	last_cortical_selected["neuron_psp_uniform_distribution"] = psp_uniform_distribution
	_make_put_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area',last_cortical_selected, false)
	$Spatial/Camera/Menu/cortical_menu/Control/Update.release_focus()
	$Spatial/Camera/Menu/properties/Control/Update.release_focus()
	
	
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
		
	$Spatial/Camera/Menu/Mapping_Properties.visible = false
	$Spatial/Camera/Menu/cortical_menu/Control/Update.release_focus()

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
	$Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text = genome_properties["cortical_id"]
	$Spatial/Camera/Menu/cortical_menu/title.text = genome_properties["cortical_name"]
	$Spatial/Camera/Menu/cortical_menu/Control/name_string.text = $Spatial/Camera/Menu/cortical_menu/title.text
#	$Spatial/Camera/Menu/cortical_menu/Control/OptionButton.selected(1) = genome_properties["cortical_group"]
	$Spatial/Camera/Menu/properties/Control/neuron_count.value = genome_properties["cortical_neuron_per_vox_count"]
	$Spatial/Camera/Menu/cortical_menu/Control/X.value = genome_properties["cortical_coordinates"]["x"]
	$Spatial/Camera/Menu/cortical_menu/Control/Y.value = genome_properties["cortical_coordinates"]["y"]
	$Spatial/Camera/Menu/cortical_menu/Control/Z.value = genome_properties["cortical_coordinates"]["z"]
	$Spatial/Camera/Menu/cortical_menu/Control/W.value = genome_properties["cortical_dimensions"]["x"]
	$Spatial/Camera/Menu/cortical_menu/Control/D.value = genome_properties["cortical_dimensions"]["z"]
	$Spatial/Camera/Menu/cortical_menu/Control/H.value = genome_properties["cortical_dimensions"]["y"]
	$Spatial/Camera/Menu/properties/Control/syn.value = genome_properties["cortical_synaptic_attractivity"]
	$Spatial/Camera/Menu/properties/Control/pst_syn.value = genome_properties["neuron_post_synaptic_potential"]
	$Spatial/Camera/Menu/properties/Control/pst_syn_max.value = float(genome_properties["neuron_post_synaptic_potential_max"])
	$Spatial/Camera/Menu/properties/Control/plst.value = genome_properties["neuron_plasticity_constant"]
	$Spatial/Camera/Menu/properties/Control/fire.value = genome_properties["neuron_fire_threshold"]
	$Spatial/Camera/Menu/properties/Control/refa.value = genome_properties["neuron_refractory_period"]
	$Spatial/Camera/Menu/properties/Control/leak.text = str(float(genome_properties["neuron_leak_coefficient"]))
	$Spatial/Camera/Menu/properties/Control/leak_Vtext.text = str((genome_properties["neuron_leak_variability"]))
	$Spatial/Camera/Menu/properties/Control/cfr.value = genome_properties["neuron_consecutive_fire_count"]
	$Spatial/Camera/Menu/properties/Control/snze.value = genome_properties["neuron_snooze_period"]
	$Spatial/Camera/Menu/properties/Control/dege.value = genome_properties["neuron_degeneracy_coefficient"]
	$Spatial/Camera/Menu/properties/Control/psud.set_pressed(genome_properties["neuron_psp_uniform_distribution"])
	last_cortical_selected = genome_properties
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_mappings/afferents?cortical_area=' + genome_properties["cortical_id"]
	$HTTP_node/afferent.request(combine_url)

func _make_post_request(url, use_ssl, data_to_send):
	# Convert data to json string:
	var query = JSON.print(data_to_send)
	# Add 'Content-Type' header:
	var headers = ["Content-Type: application/json"]
	$HTTP_node/send_feagi.request(url, headers, use_ssl, HTTPClient.METHOD_POST, query)

func _make_put_request(url, data_to_send, use_ssl):
	# Convert data to json string:
	var query = JSON.print(data_to_send)
	# Add 'Content-Type' header:
	var headers = ["Content-Type: application/json"]
	$HTTP_node/send_feagi.request(url, headers, use_ssl, HTTPClient.METHOD_PUT, query)
	
func _make_delete_request(url, use_ssl):
	# Convert data to json string:
	# Add 'Content-Type' header:
	var headers = ["Content-Type: application/json"]
	$HTTP_node/send_feagi.request(url, headers, use_ssl, HTTPClient.METHOD_DELETE)

func _on_send_feagi_request_completed(_result, _response_code, _headers, _body):
	pass

func _on_info_pressed():
	for i in plus_node:
		i.queue_free()
	plus_node.clear()
	$Spatial/Camera/Menu/Mapping_Properties.visible = false
#	$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu.rect_size.y = 107
	$Spatial/Camera/Menu/Mapping_Properties.visible = true

func update_cortical_map_name(name):
	var combine_name = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_mappings/efferents?cortical_area=' + name
	$HTTP_node/information_button.request(combine_name)

func _on_information_button_request_completed(_result, _response_code, _headers, body):
	# Do not touch here. THis is for information biteration_nameutton only and will dedicate
	# to the information button
	# Clear duplicate cortical maps name up
	child_holder_clear() 
	# Obtain the data from API and convert it into json/string
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var new_name = ""
	var counter = 0 # To increase the height between two different duplicated nodes
#	$Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.visible = false
	var cap = 120 # Keep nodes inside the white rectangle
	for i in api_data:
		var new_node = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.duplicate()
		$Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.add_child(new_node)
		child_node_holder.append(new_node)
		new_name = id_to_name(i)
		new_node.visible = true
		new_node.text = new_name
		new_node.rect_position.x = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_position.x
		new_node.rect_position.y = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_position.y + (counter * 30)
		if new_node.rect_position.y > cap:
#			$Spatial/Camera/Menu/button_choice/white_background.rect_size.y += 30
			cap += 30 
#		$Spatial/Camera/Menu/button_choice.rect_position.y = $Spatial/Camera/Menu/button_choice.rect_position.y + (counter * 5)
		new_node.rect_size.x = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_size.x 
		new_node.rect_size.y = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_size.y
		new_node.visible = true
		new_node.get_child(3).connect("pressed", self, "dst_remove_pressed", [new_node])
		new_node.get_child(2).connect("pressed", self, "info_pressed", [new_node])
		counter += 1
	map_colorful()
#	$Spatial/Camera/Menu/cortical_menu/Control/Update.rect_position.y = 10 + $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.rect_size.y + $Spatial/Camera/Menu/cortical_mapping.rect_position.y


func child_holder_clear():
	# Clear duplicate cortical maps name up
	if child_node_holder:
		for i in child_node_holder:
			i.queue_free()
		child_node_holder = []
	if ghost_morphology:
		ghost_morphology = []

func _on_get_genome_name_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	if api_data != null:
		$Spatial/Camera/Menu/information_menu/genome_string.text = api_data
		$HTTP_node/get_burst.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/feagi/burst_engine/stimulation_period')

func _on_get_burst_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$Spatial/Camera/Menu/information_menu/burst_duration_label/burst_value.text = str(1/float(api_data))

func _on_download_pressed():
	_clear_node_name_list(global_name_list)
	genome_data = ""
	previous_genome_data = ""

func _on_add_pressed():
	var json_data = {}
	var flag_boolean = false
	if $"Spatial/Camera/Menu/addition_menu/OptionButton".selected == 1 or $"Spatial/Camera/Menu/addition_menu/OptionButton".selected == 2:
		json_data["cortical_type"] = $"Spatial/Camera/Menu/addition_menu/OptionButton".get_item_text($"Spatial/Camera/Menu/addition_menu/OptionButton".selected)
		json_data["cortical_name"] = $Spatial/Camera/Menu/addition_menu/cortical_name_label/type.get_item_text($Spatial/Camera/Menu/addition_menu/cortical_name_label/type.selected)
		json_data["cortical_coordinates"] = {}
		json_data["cortical_coordinates"]["x"] = $Spatial/Camera/Menu/addition_menu/xyz/X_SpinBox.value
		json_data["cortical_coordinates"]["y"] = $Spatial/Camera/Menu/addition_menu/xyz/Y_Spinbox.value
		json_data["cortical_coordinates"]["z"] = $Spatial/Camera/Menu/addition_menu/xyz/Z_Spinbox.value
		json_data["channel_count"] = $Spatial/Camera/Menu/addition_menu/count/count_spinbox.value

		_make_post_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area', false, json_data)

	if $"Spatial/Camera/Menu/addition_menu/OptionButton".selected == 3:
		if $Spatial/Camera/Menu/addition_menu/cortical_name_textbox/type.text != "" and $Spatial/Camera/Menu/addition_menu/cortical_name_textbox/type.text != " ":
			json_data["cortical_type"] = "CUSTOM"
			json_data["cortical_name"] = $Spatial/Camera/Menu/addition_menu/cortical_name_textbox/type.text
			json_data["cortical_coordinates"] = {}
			json_data["cortical_dimensions"] = {}
			json_data["cortical_coordinates"]["x"] = $Spatial/Camera/Menu/addition_menu/xyz/X_SpinBox.value
			json_data["cortical_coordinates"]["y"] = $Spatial/Camera/Menu/addition_menu/xyz/Y_Spinbox.value
			json_data["cortical_coordinates"]["z"] = $Spatial/Camera/Menu/addition_menu/xyz/Z_Spinbox.value
			json_data["cortical_dimensions"]["x"] = $Spatial/Camera/Menu/addition_menu/wdh/W_Spinbox.value
			json_data["cortical_dimensions"]["y"] = $Spatial/Camera/Menu/addition_menu/wdh/H_Spinbox.value
			json_data["cortical_dimensions"]["z"] = $Spatial/Camera/Menu/addition_menu/wdh/D_Spinbox.value
			json_data["channel_count"] = $Spatial/Camera/Menu/custom_cortical/count_spinbox.value
			
			generate_single_cortical(json_data["cortical_coordinates"]["x"], json_data["cortical_coordinates"]["y"], json_data["cortical_coordinates"]["z"], json_data["cortical_dimensions"]["x"], json_data["cortical_dimensions"]["y"], json_data["cortical_dimensions"]["z"], json_data["cortical_name"])
			
			_make_post_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/custom_cortical_area', false, json_data)
			$Spatial/Camera/Menu/addition_menu/add.release_focus()
			$Spatial/Camera.transform.origin=Vector3(json_data["cortical_coordinates"]["x"]-20,json_data["cortical_coordinates"]["y"],json_data["cortical_coordinates"]["z"]+20)
		else:
			flag_boolean = true
	if flag_boolean != true:
		$Spatial/Camera/Menu/addition_menu.visible = false
#	$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.timer = true
#		network_setting.send("updated")

func _on_remove_pressed():
	var get_name = $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area?cortical_area_name=' + get_name
	_clear_single_cortical($Spatial/Camera/Menu/cortical_menu/title.text, global_name_list)
#	$HTTP_node/remove_cortical.request(combine_url)
	_make_delete_request(combine_url, false)
	$Spatial/Camera/Menu/cortical_menu/Control/remove.release_focus()

func _on_update_destination_info_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	plus_node_clear()
	ghost_morphology_clear()
	if api_data != null:
		if api_data.has("Request failed..."):
			pass
		else:
			for i in range(len(api_data)):
				var new_node = $Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control.duplicate()
				$Spatial/Camera/Menu/"Mapping_Properties"/inside_mapping_menu.add_child(new_node)
				new_node.rect_position.y = (50 * (plus_node.size()))
				plus_node.append(new_node)
				$Spatial/Camera/Menu/"Mapping_Properties"/inside_mapping_menu.rect_size.y += (30 * plus_node.size())
				new_node.get_child(0).connect("pressed", self, "_on_Mapping_def_pressed")
				new_node.get_child(4).connect("text_changed", self, "_on_text_changed")
				ghost_morphology.append(new_node.get_child(0))
				
				for x in new_node.get_child(0).get_item_count():
					if new_node.get_child(0).get_item_text(x) == api_data[i]["morphology_id"]:
						new_node.get_child(0).selected = x
						$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.selected = x
				new_node.visible = true
				new_node.get_child(1).value = api_data[i]["morphology_scalar"][0]
				new_node.get_child(2).value = api_data[i]["morphology_scalar"][1]
				new_node.get_child(3).value = api_data[i]["morphology_scalar"][2]
				new_node.get_child(4).text = str(api_data[i]["postSynapticCurrent_multiplier"])
				new_node.get_child(5).set_pressed(api_data[i]["plasticity_flag"])
				new_node.get_child(6).connect("pressed", self, "map_info_pressed", [new_node])
				new_node.get_child(7).connect("pressed", self, "remove_button_inside_dst", [new_node])

func _on_genome_data_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var create_json = {}
	for i in api_data:
		create_json[i] = api_data[i]
	genome_data["genome"] = create_json
	_csv_generator()
#	if one_time_flag:
#		for i in api_data:
#			create_json[i] = api_data[i]
#		genome_data["genome"] = create_json
#		_csv_generator()
#		one_time_flag = false
#	else:
##		print("api data: ", api_data.size())
#		for i in api_data:
#			if not(i) in genome_data["genome"]:
#				generate_single_cortical(api_data[i][0], api_data[i][1], api_data[i][2], api_data[i][4], api_data[i][5], api_data[i][6], api_data[i][7])
#			elif i in genome_data["genome"]:
#				if api_data[i].hash() != genome_data["genome"][i].hash():
#					_clear_single_cortical(i, global_name_list)
#					generate_single_cortical(api_data[i][0], api_data[i][1], api_data[i][2], api_data[i][4], api_data[i][5], api_data[i][6], api_data[i][7])


func _on_remove_cortical_request_completed(_result, _response_code, _headers, _body):
	pass # Replace with function body.

func map_info_pressed(node_duplicated):
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.selected = node_duplicated.get_child(0).get_selected_id()
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + $Spatial/Camera/Menu/rule_properties/mapping_rule_options.get_item_text(node_duplicated.get_child(0).get_selected_id())
	$HTTP_node/type_rules.request(combine_url)
	
func remove_button_inside_dst(node_duplicated):
	var number_holder = []
	var rule_to_delete = node_duplicated.get_child(0).text
	var morphology_scalar_x = node_duplicated.get_child(1).value
	var morphology_scalar_y = node_duplicated.get_child(2).value
	var morphology_scalar_z = node_duplicated.get_child(3).value
	var plasticity_flag = node_duplicated.get_child(5).is_pressed()
	var postSynapticCurrent_multiplier = float(node_duplicated.get_child(4).text)
	
	var get_id = name_to_id($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_text($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_selected_id()))
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
	var get_id = $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text
	if duplicated_node_lineedit.text != " " and duplicated_node_lineedit.text != "":
		get_id_from_dst = genome_data["genome"][duplicated_node_lineedit.text][7]
		var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/mapping_properties?src_cortical_area=#&dst_cortical_area=$'.replace("#", get_id)
		combine_url= combine_url.replace("$", get_id_from_dst)
		$HTTP_node/update_destination_info.request(combine_url)
	for i in $Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_item_count():
		if $Spatial/Camera/Menu/cortical_menu/Control/name_string.text == $Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_item_text(i):
			$Spatial/Camera/Menu/Mapping_Properties/source_dropdown.selected = i
	for i in $Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_count():
		if duplicated_node_lineedit.text == $Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_text(i):
			$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.selected = i
	

func dst_remove_pressed(duplicated_node_lineedit):
	_on_info_pressed() # leveraging the same function to clear all infos on the box
	$Spatial/Camera/Menu/Mapping_Properties.visible = false
	$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.select(0)
	var dst_id = name_to_id(duplicated_node_lineedit.text)
	var grab_id = $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/mapping_properties?src_cortical_area=#&dst_cortical_area=$'.replace("#", grab_id)
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
		_make_put_request(combine_url,[], false)
#		$Spatial/Camera/Menu/cortical_menu/Control/Update.rect_position.y = 10 + $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.rect_size.y + $Spatial/Camera/Menu/cortical_mapping.rect_position.y 
#		$Spatial/Camera/Menu/cortical_mapping.rect_position.y = $Spatial/Camera/Menu/cortical_mapping.rect_position.y - (number_holder.size() * 5)

func delete_morphology(input_node):
	input_node.queue_free()
	new_morphology_node.erase(input_node)

func _on_mapping_def_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$"Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def".clear()
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.clear()
	$"Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def".add_item(" ")
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.add_item(" ")
	for i in api_data:
		$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(i)
		$Spatial/Camera/Menu/rule_properties/mapping_rule_options.add_item(i)

func _on_morphology_types_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$Spatial/Camera/Menu/rule_properties/rules/rule_type_options.add_item(" ")
	for i in api_data:
		$Spatial/Camera/Menu/rule_properties/rules/rule_type_options.add_item(i)

func _on_plus_add_pressed():
	var new_node = $Spatial/Camera/Menu/"Mapping_Properties"/inside_mapping_menu/Control.duplicate()
	$Spatial/Camera/Menu/"Mapping_Properties"/inside_mapping_menu.add_child(new_node)
	plus_node.append(new_node)
	new_node.get_child(0).connect("pressed", self, "_on_Mapping_def_pressed")
	new_node.get_child(4).connect("text_changed", self, "_on_text_changed")
	ghost_morphology.append(new_node.get_child(0))
	new_node.visible = true
	new_node.get_child(1).value = 1
	new_node.get_child(2).value = 1
	new_node.get_child(3).value = 1
	new_node.get_child(4).text = str(1)
	new_node.rect_position.y = (50 * (plus_node.size()))
	$Spatial/Camera/Menu/"Mapping_Properties"/inside_mapping_menu.rect_size.y += (30 * plus_node.size())



func _on_type_rules_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	for i in $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_count():
		for x in api_data:
			if $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_text(i) == x:
				$Spatial/Camera/Menu/rule_properties/rules/rule_type_options.select(i)


func _on_save_pressed():
	var json_data = {}
	json_data["name"] = $Spatial/Camera/Menu/rule_properties/mapping_rule_options.get_item_text($Spatial/Camera/Menu/rule_properties/mapping_rule_options.get_selected_id())
	json_data["type"] = $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_text($Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_selected_id())
	if json_data["type"] == "patterns":
		json_data["morphology"] = []
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
			for _x in range(7):
				if not "?" in i.get_child(empty_flag).text and not "*" in i.get_child(empty_flag).text:
					if empty_flag < 3:
						empty_array1.append(int(i.get_child(empty_flag).text))
					elif empty_flag > 3:
						empty_array2.append(int(i.get_child(empty_flag).text))
				else:
					if empty_flag < 3:
						empty_array1.append(str(i.get_child(empty_flag).text))
					elif empty_flag > 3:
						empty_array2.append(str(i.get_child(empty_flag).text))
				empty_flag += 1
			full_array.append(empty_array1)
			full_array.append(empty_array2)
			string_input.append(full_array)
		json_data["morphology"] = string_input
		$Spatial/Camera/Menu/rule_properties.visible = false
		new_morphology_clear()
	if json_data["type"] == "vectors":
		json_data["morphology"] = []
		var empty_array1 = []
		for i in new_morphology_node:
			var temp_array = []
			temp_array = [i.get_child(0).value, i.get_child(1).value, i.get_child(2).value]
			empty_array1.append(temp_array)
		json_data["morphology"] = empty_array1
		new_morphology_clear()
#	if json_data["type"] == "composite":
#
	_make_put_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology',json_data, false)
	$Spatial/Camera/Menu/rule_properties.visible = false



#func _on_update_morphology_request_completed(result, response_code, headers, body):
#	pass # Replace with function body.


func _on_delete_pressed():
	var grab_name_rule = $Spatial/Camera/Menu/rule_properties/mapping_rule_options.get_item_text($Spatial/Camera/Menu/rule_properties/mapping_rule_options.get_selected_id())
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + grab_name_rule
	_make_delete_request(combine_url, false)

func _on_get_cortical_dst_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var dst_data = {}
	for i in api_data["cortical_destinations"]:
		for x in genome_data["genome"]:
			for b in child_node_holder:
				if x == b.text:
					if i in genome_data["genome"][x][7]:
						dst_data[i] = api_data["cortical_destinations"][i]
	dst_data_holder = dst_data.duplicate()
	$Spatial/Camera/Menu/cortical_menu/Control/Update.rect_position.y = 749 
	if $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text != "":
		var get_id = $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text
		var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/monitoring/neuron/membrane_potential?cortical_area=' + get_id
		$HTTP_node/mem_request.request(combine_url)

func _on_cortical_mapping_add_pressed():
	_on_info_pressed() # leveraging the same function to clear all infos on the box
	$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.select(0)
	var add_flag = true
	for i in child_node_holder:
		if i.text == "":
			add_flag = false
	if add_flag:
		generate_cortical_mapping()


func generate_cortical_mapping():
	for i in $Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_item_count():
		if $Spatial/Camera/Menu/cortical_menu/Control/name_string.text == $Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_item_text(i):
			$Spatial/Camera/Menu/Mapping_Properties/source_dropdown.selected = i
	var counter = child_node_holder.size()
	var new_node = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.duplicate()
	$Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.add_child(new_node)
	child_node_holder.append(new_node)
	new_node.visible = true
#	new_node.text = new_name
	new_node.rect_position.x = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_position.x
	new_node.rect_position.y = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_position.y + (counter * 30)
#	if new_node.rect_position.y > cap:
##		$Spatial/Camera/Menu/cortical_mapping/Control/white_background.rect_size.y += 30
#		cap += 30
#	$Spatial/Camera/Menu/cortical_menu/Control/Update.rect_position.y = 10 + $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.rect_size.y + $Spatial/Camera/Menu/button_choice.rect_position.y 
#	$Spatial/Camera/Menu/cortical_mapping.rect_position.y = $Spatial/Camera/Menu/cortical_mapping.rect_position.y + (counter * 5)
	new_node.rect_size.x = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_size.x 
	new_node.rect_size.y = $Spatial/Camera/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer/cortical_map_name.rect_size.y
	new_node.visible = true
	new_node.editable = true
	new_node.text = ""
	new_node.get_child(3).connect("pressed", self, "dst_remove_pressed", [new_node])
	new_node.get_child(2).connect("pressed", self, "info_pressed", [new_node])
	counter += 1
	
func name_to_id(name):
	var iteration_name = name
	var grab_id_cortical = ""
	for i in genome_data["genome"]:
		if i == iteration_name:
			grab_id_cortical = genome_data["genome"][i][7]
			return grab_id_cortical
			
func id_to_name(name):
	for x in genome_data["genome"]:
		if genome_data["genome"][x][7] == name:
			return x

func _on_menu_pressed():
	$Spatial/Camera/Menu/information_menu/cortical_cam_label/menu.text = ""
	if $Spatial/Camera/Menu/information_menu/cortical_cam_label/menu_itemlist.visible:
		$Spatial/Camera/Menu/information_menu/cortical_cam_label/menu_itemlist.visible = false
	else:
		$Spatial/Camera/Menu/information_menu/cortical_cam_label/menu_itemlist.visible = true
		$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.load_options()
		$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.release_focus()

func _on_cortical_dropdown_pressed():
	$Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.load_options()

func _on_burst_value_text_entered(new_text):
	var json = {}
	if new_text == "0" or new_text == "":
		json["burst_duration"] = float(1/float(1))
	else:
		json["burst_duration"] = float(1/float(new_text))
	_make_post_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/feagi/burst_engine', false, json)
	$Spatial/Camera/Menu/information_menu/burst_duration_label/burst_value.release_focus()

func _on_burst_value_focus_exited():
	var json = {}
	var new_text = $Spatial/Camera/Menu/information_menu/burst_duration_label/burst_value.text
	if new_text == "0" or new_text == "":
		json["burst_duration"] = float(1/float(1))
	else:
		json["burst_duration"] = float(1/float(new_text))
	_make_post_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/feagi/burst_engine', false, json)
	$Spatial/Camera/Menu/information_menu/burst_duration_label/burst_value.release_focus()

func _on_Neuron_morphologies_button_pressed():
	if $Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.visible:
		$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.visible = false
	else:
		$Spatial/Camera/Menu/rule_properties.visible = false
		$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.visible = true
		$HTTP_node/morphology_list.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_list')


func _on_morphology_list_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.clear()
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.clear()
	$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.clear()
	$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.clear()
	$Spatial/Camera/Menu/Control/inner_box/box_of_composite/mapper_composite.clear()
	$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.add_item(" ")
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.add_item(" ")
	$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.add_item(" ")
	$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(" ")
	$Spatial/Camera/Menu/Control/inner_box/box_of_composite/mapper_composite.add_item(" ")
	for i in api_data:
		$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(i)
		$Spatial/Camera/Menu/rule_properties/mapping_rule_options.add_item(i)
		$Spatial/Camera/Menu/Control/inner_box/box_of_composite/mapper_composite.add_item(i)
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.add_item(i)
#		$Spatial/Camera/Menu/information_menu/Neuron_morphologies.add_item(i)
		$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.add_item(i, null, true)


func _on_Neuron_morphologies_item_selected(index):
	if index != 0:
		$Spatial/Camera/Menu/rule_properties.visible = true
		$Spatial/Camera/Menu/rule_properties/mapping_rule_options.selected = index
		$Spatial/Camera/Menu/rule_properties/mapping_rule_options.emit_signal("item_selected", index)
		$Spatial/Camera/Menu/rule_properties/mapping_rule_options.release_focus()


func _on_Button_pressed():
	$Spatial/Camera/Menu/Control/inner_box/morphology_type.clear()
	for i in $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_count():
		if $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_text(i) != "functions":
			$Spatial/Camera/Menu/Control/inner_box/morphology_type.add_item($Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_text(i))
	$Spatial/Camera/Menu/Control.visible = true
	if $Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.get_item_count() == 0:
		$HTTP_node/morphology_list.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_list')

func _on_create_pressed():
	if $Spatial/Camera/Menu/Control/inner_box/morphology_name.text != "":
		if $Spatial/Camera/Menu/Control/inner_box/morphology_type.get_item_text($Spatial/Camera/Menu/Control/inner_box/morphology_type.selected) == "patterns":
			var json = {}
			json["name"] = $Spatial/Camera/Menu/Control/inner_box/morphology_name.text
			json["type"] = $Spatial/Camera/Menu/Control/inner_box/morphology_type.get_item_text($Spatial/Camera/Menu/Control/inner_box/morphology_type.get_selected_id())
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
				for _x in range(7):
					if not "?" in i.get_child(empty_flag).text and not "*" in i.get_child(empty_flag).text:
						if empty_flag < 3:
							empty_array1.append(int(i.get_child(empty_flag).text))
						elif empty_flag > 3:
							empty_array2.append(int(i.get_child(empty_flag).text))
					else:
						if empty_flag < 3:
							empty_array1.append(str(i.get_child(empty_flag).text))
						elif empty_flag > 3:
							empty_array2.append(str(i.get_child(empty_flag).text))
					empty_flag += 1
				full_array.append(empty_array1)
				full_array.append(empty_array2)
				string_input.append(full_array)
			json["morphology"] = string_input
			_make_post_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology', false, json)
			$Spatial/Camera/Menu/Control.visible = false
			new_morphology_clear()
			$Spatial/Camera/Menu/Control/inner_box/morphology_name.text = ""
	if $Spatial/Camera/Menu/Control/inner_box/morphology_type.get_item_text($Spatial/Camera/Menu/Control/inner_box/morphology_type.selected) == "vectors":
		var json = {}
		json["name"] = $Spatial/Camera/Menu/Control/inner_box/morphology_name.text
		json["type"] = $Spatial/Camera/Menu/Control/inner_box/morphology_type.get_item_text($Spatial/Camera/Menu/Control/inner_box/morphology_type.get_selected_id())
		var empty_array1 = []
		for i in new_morphology_node:
			var temp_array = []
			temp_array.append(i.get_child(0).value)
			temp_array.append(i.get_child(1).value)
			temp_array.append(i.get_child(2).value)
			empty_array1.append(temp_array)
		json["morphology"] = empty_array1
		_make_post_request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology', false, json)
		$Spatial/Camera/Menu/Control.visible = false
		new_morphology_clear()
		$Spatial/Camera/Menu/Control/inner_box/morphology_name.text = ""

func _on_X_inside_inner_box_pressed():
	$Spatial/Camera/Menu/Control.visible = false
	new_morphology_clear()

func _on_afferent_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var counter = 0
	afferent_holder_clear()
	for i in api_data:
		var new_node = $Spatial/Camera/Menu/cortical_mapping/Control/afferent/VBoxContainer/LineEdit.duplicate()
		$Spatial/Camera/Menu/cortical_mapping/Control/afferent/VBoxContainer.add_child(new_node)
		afferent_child_holder.append(new_node)
		new_node.visible = true
		new_node.text = id_to_name(i)
		new_node.visible = true
		new_node.rect_position.x = $Spatial/Camera/Menu/cortical_mapping/Control/afferent/VBoxContainer.rect_position.x + 5
		new_node.rect_position.y = $Spatial/Camera/Menu/cortical_mapping/Control/afferent/VBoxContainer.rect_position.y + (counter * 20)
		counter += 1
	
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
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/Label2.rect_position.y = 160
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.rect_position.y = 180
		$Spatial/Camera/Menu/rule_properties/rules.rect_size.y = 436
		$Spatial/Camera/Menu/rule_properties.rect_size.y = 608
		$Spatial/Camera/Menu/rule_properties/rules/save.rect_position.y = 477.001
		$Spatial/Camera/Menu/rule_properties/rules/delete.rect_position.y = 477.001
		$Spatial/Camera/Menu/Control/inner_box/grey_bg.rect_size.y = 283
		$Spatial/Camera/Menu/Control/ColorRect.rect_size.y = 394
		$Spatial/Camera/Menu/Control/update.rect_position.y = 330
		$Spatial/Camera/Menu/Control/inner_box/Button.disabled = false

func _on_source_dropdown_item_selected(index):
	if index != 0:
		if $Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.selected != 0:
			var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/mapping_properties?src_cortical_area=#&dst_cortical_area=$'.replace("#", name_to_id($Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_item_text(index)))
			combine_url= combine_url.replace("$", name_to_id($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_text($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_selected_id())))
			$HTTP_node/update_destination_info.request(combine_url)

func plus_node_clear():
	for i in plus_node:
		i.queue_free()
	plus_node.clear()

func ghost_morphology_clear():
	for i in ghost_morphology:
		i.queue_free()
	ghost_morphology.clear()

func _on_cortical_dropdown_item_selected(index):
	if index != 0:
		if $Spatial/Camera/Menu/Mapping_Properties/source_dropdown.selected != 0:
			var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/mapping_properties?src_cortical_area=#&dst_cortical_area=$'.replace("#", name_to_id($Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_item_text($Spatial/Camera/Menu/Mapping_Properties/source_dropdown.get_selected_id())))
			combine_url= combine_url.replace("$", name_to_id($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.get_item_text(index)))
			$HTTP_node/update_destination_info.request(combine_url)

func _on_update_inside_map_pressed():
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/mapping_properties?src_cortical_area=#&dst_cortical_area=$'
	var get_id = name_to_id($Spatial/Camera/Menu/Mapping_Properties/source_dropdown.text)
	var get_dst_data = name_to_id($Spatial/Camera/Menu/Mapping_Properties/cortical_dropdown.text)
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
		_make_put_request(combine_url,dst_data["cortical_destinations"][get_id], false)
	else:
		_make_put_request(combine_url,[], false)
		
func map_colorful():
	pass
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
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/monitoring/neuron/membrane_potential?cortical_area=' + $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text + '&state=' + str($Spatial/Camera/Menu/button_choice/Control/mem.is_pressed())
	_make_post_request(combine_url, false, $Spatial/Camera/Menu/button_choice/Control/mem.is_pressed())

func _on_mem_request_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var get_id = $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text
	$Spatial/Camera/Menu/button_choice/Control/mem.set_pressed(api_data)
	var combnine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/monitoring/neuron/synaptic_potential?cortical_area=' + get_id
	$HTTP_node/syn_request.request(combnine_url)

func _on_syn_request_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$Spatial/Camera/Menu/button_choice/Control/syn.set_pressed(api_data)

func _on_syn_pressed():
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/monitoring/neuron/synaptic_potential?cortical_area=' + $Spatial/Camera/Menu/cortical_menu/Control/cortical_id.text + '&state=' + str($Spatial/Camera/Menu/button_choice/Control/syn.is_pressed())
	_make_post_request(combine_url, false, $Spatial/Camera/Menu/button_choice/Control/syn.is_pressed())

func _on_insert_button_pressed():
	$Spatial/Camera/Menu/insert_menu/inner_box.visible = true
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/append?circuit_name=' + $Spatial/Camera/Menu/insert_menu/inner_box/name_text.text
	var new_data = ["placeholder"]
	_make_post_request(combine_url, false, new_data)


func _on_circuit_request_request_completed(_result, _response_code, _headers, body):
	$Spatial/Camera/Menu/insert_menu/insert_button/ItemList.clear()
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	for i in api_data:
		$Spatial/Camera/Menu/insert_menu/insert_button/ItemList.add_item(i, null, true)


func _on_import_pressed():
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/circuits'
	$HTTP_node/circuit_request.request(combine_url)


func _on_ItemList_item_selected(index):
	var name_text = $Spatial/Camera/Menu/insert_menu/insert_button/ItemList.get_item_text(index)
	name_text = symbols_checker_for_api(name_text)
	$Spatial/Camera/Menu/insert_menu/inner_box.visible = true
	$Spatial/Camera/Menu/insert_menu/inner_box/name_text.text = name_text
	var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/circuit_size?circuit_name=' + name_text
	$HTTP_node/circuit_size.request(combine_url)


func _on_circuit_size_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$Spatial/Camera/Menu/insert_menu/inner_box/W_spinbox.value = api_data[0]
	$Spatial/Camera/Menu/insert_menu/inner_box/D_spinbox.value = api_data[1]
	$Spatial/Camera/Menu/insert_menu/inner_box/H_spinbox.value = api_data[2]
	generate_single_cortical($Spatial/Camera/Menu/insert_menu/x_spinbox.value, $Spatial/Camera/Menu/insert_menu/y_spinbox.value, $Spatial/Camera/Menu/insert_menu/z_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/W_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/D_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/H_spinbox.value, "example")
	
func symbols_checker_for_api(string_data):
	if " " in string_data:
		string_data = string_data.replace(" ", "%20")
	if "(" in string_data:
		string_data = string_data.replace("(", "%28")
	if ")" in string_data:
		string_data = string_data.replace(")", "%29")
	return string_data

func _on_x_spinbox_value_changed(_value):
	generate_single_cortical($Spatial/Camera/Menu/insert_menu/x_spinbox.value, $Spatial/Camera/Menu/insert_menu/y_spinbox.value, $Spatial/Camera/Menu/insert_menu/z_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/W_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/D_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/H_spinbox.value, "example")

func _on_y_spinbox_value_changed(_value):
	generate_single_cortical($Spatial/Camera/Menu/insert_menu/x_spinbox.value, $Spatial/Camera/Menu/insert_menu/y_spinbox.value, $Spatial/Camera/Menu/insert_menu/z_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/W_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/D_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/H_spinbox.value, "example")

func _on_z_spinbox_value_changed(_value):
	generate_single_cortical($Spatial/Camera/Menu/insert_menu/x_spinbox.value, $Spatial/Camera/Menu/insert_menu/y_spinbox.value, $Spatial/Camera/Menu/insert_menu/z_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/W_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/D_spinbox.value, $Spatial/Camera/Menu/insert_menu/inner_box/H_spinbox.value, "example")


func _on_Neuron_morphologies_item_item_selected(index):
	$Spatial/Camera/Menu/rule_properties.visible = true
	$Spatial/Camera/Menu/information_menu/Neuron_morphologies_button.text = $Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.get_item_text(index)
	$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.visible = false
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.selected = index
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.emit_signal("item_selected", index)
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.release_focus()

func _on_Mapping_def_pressed():
	$HTTP_node/ghost_morphology_list.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_list')
	
func _on_ghost_morphology_list_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.clear()
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.clear()
	$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.clear()
	$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.add_item(" ")
	$Spatial/Camera/Menu/rule_properties/mapping_rule_options.add_item(" ")
	$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(" ")
	for i in api_data:
		$Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.add_item(i)
		$Spatial/Camera/Menu/rule_properties/mapping_rule_options.add_item(i)
#		$Spatial/Camera/Menu/information_menu/Neuron_morphologies.add_item(i)
		$Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.add_item(i, null, true)
	if ghost_morphology:
		for a in ghost_morphology:
			var node_ghost = a
			if node_ghost.get_item_count() != $Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.get_item_count():
				node_ghost.clear()
				for i in $Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.get_item_count():
					node_ghost.add_item($Spatial/Camera/Menu/Mapping_Properties/inside_mapping_menu/Control/Mapping_def.get_item_text(i), i)


func _on_text_changed(new_text):
	Godot_list.Node_2D_control = true
	if new_text != "":
		if new_text.is_valid_float():
			self.value = float(new_text)

func _on_get_morphology_usuage_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var string_list = ""
	$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.text = ""
	for i in api_data:
		string_list = string_list + str(id_to_name(i[0]), " > ", id_to_name(i[1])) + "\n"
	$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.text += str(string_list)

func _morphology_button_pressed():
	var counter = 0
	counter = len(new_morphology_node)
	if $Spatial/Camera/Menu/Control/inner_box/morphology_type.get_item_text($Spatial/Camera/Menu/Control/inner_box/morphology_type.selected) == "patterns":
		var new_node = $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.duplicate()
		$Spatial/Camera/Menu/Control/inner_box/box_of_pattern.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(7).connect("pressed", self, "delete_morphology", [new_node])
		new_node.rect_position.x = $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.rect_position.x + $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.rect_size.x
		new_node.rect_position.y = $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.rect_position.y + (30 * counter)
	elif $Spatial/Camera/Menu/Control/inner_box/morphology_type.get_item_text($Spatial/Camera/Menu/Control/inner_box/morphology_type.selected) == "vectors":
		var new_node = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.duplicate()
		new_morphology_node.append(new_node)
		$Spatial/Camera/Menu/Control/inner_box/box_of_vectors.add_child(new_node)
		new_node.visible = true
		new_node.get_child(3).connect("pressed", self, "delete_morphology", [new_node])
		new_node.rect_size = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.rect_size
		new_node.rect_position.x = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.rect_position.x
		new_node.rect_position.y = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.rect_position.y + (30 * counter)
	

func _on_morphology_name_focus_exited():
	new_morphology_clear()
	for i in $Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.get_item_count():
		var name_morphology = $Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.get_item_text(i) 
		if $Spatial/Camera/Menu/information_menu/Neuron_morphologies_item.get_item_text(i) == $Spatial/Camera/Menu/Control/inner_box/morphology_name.text:
			if "+" in name_morphology:
				name_morphology = name_morphology.replace("+", "%2B")
			if "[" in name_morphology:
				name_morphology = name_morphology.replace("[", "%5B")
			if "]" in name_morphology:
				name_morphology = name_morphology.replace("]", "%5D")
			if ", " in name_morphology:
				name_morphology = name_morphology.replace(", ", "%2C%20")
			var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + name_morphology
			$Spatial/Camera/Menu/rule_properties/get_morphology.request(combine_url)
			
func _on_get_morphology_request_completed(_result, _response_code, _headers, body):
	new_morphology_clear()
	flag = false
	var json = JSON.parse(body.get_string_from_utf8()) # Every http data, it's done in poolbytearray
	var api_data = json.result
	var new_name = ""
	var counter = 0
	print("api: ", api_data)
	for i in api_data:
		if i != "functions":
			new_name = str(api_data[i])
			for x in api_data[i]:
				if i == "patterns":
					counter = len(new_morphology_node)
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = true
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = false
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label.visible = false
					var new_node = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.duplicate()
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition.add_child(new_node)
					new_morphology_node.append(new_node)
					new_node.visible = true
					new_node.get_child(6).connect("pressed", self, "delete_morphology", [new_node])
					new_node.rect_position.x = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.rect_position.x
					new_node.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.rect_position.y + (25 * counter)
					if len(x) == 2:
						new_node.get_child(0).text = str(x[0][0])
						new_node.get_child(1).text = str(x[0][1])
						new_node.get_child(2).text = str(x[0][2])
						new_node.get_child(3).text = str(x[1][0])
						new_node.get_child(4).text = str(x[1][1])
						new_node.get_child(5).text = str(x[1][2])
					else:
						print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
						print("The last array: ", x, " and the name of morphology: ", $Spatial/Camera/Menu/Control/inner_box/morphology_name.text)
						break
				elif i == "vectors":
					counter = len(new_morphology_node)
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label.visible = false
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = false
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = true
					var new_node = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.duplicate()
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/.add_child(new_node)
					new_morphology_node.append(new_node)
					new_node.visible = true
					new_node.get_child(3).connect("pressed", self, "delete_morphology", [new_node])
					new_node.rect_size = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.rect_size
					new_node.rect_position.x = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.rect_position.x
					new_node.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.rect_position.y + (30 * counter)
					new_node.get_child(0).value = int(x[0])
					new_node.get_child(1).value = int(x[1])
					new_node.get_child(2).value = int(x[2])
				if len(new_morphology_node) > 4:
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/Label2.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/Label2.rect_position.y + (4 * counter)
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.rect_position.y + (4 * counter)
					$Spatial/Camera/Menu/rule_properties/rules.rect_size.y += (counter * 4)
					$Spatial/Camera/Menu/rule_properties.rect_size.y += (counter * 4)
				elif i == "composite":
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = false
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = false
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label.visible = true
					for a in $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.get_item_count():
#						print(a)
						if api_data[i]['mapper_morphology'] == $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.get_item_text(a):
							$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/morphology_name.select(a)
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/X_box/C.value = api_data[i]['parameters']['src_pattern'][0][0]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/X_box/S.value = api_data[i]['parameters']['src_pattern'][0][1]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/Y_box/C.value = api_data[i]['parameters']['src_pattern'][1][0]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/Y_box/S.value = api_data[i]['parameters']['src_pattern'][1][1]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/Z_box/C.value = api_data[i]['parameters']['src_pattern'][2][0]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/Z_box/S.value = api_data[i]['parameters']['src_pattern'][2][1]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/x.value = api_data[i]['parameters']['src_seed'][0]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/y.value = api_data[i]['parameters']['src_seed'][1]
					$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/composite_label/z.value = api_data[i]['parameters']['src_seed'][2]
			counter = 0
			if $Spatial/Camera/Menu/Control.visible:
				for x in api_data[i]:
					if i == "patterns":
						counter += 1
						var new_node = $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.duplicate()
						$Spatial/Camera/Menu/Control/inner_box/box_of_pattern.add_child(new_node)
						new_morphology_node.append(new_node)
						new_node.visible = true
						new_node.get_child(7).connect("pressed", self, "delete_morphology", [new_node])
						new_node.rect_position.x = $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.rect_position.x + $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.rect_size.x
						new_node.rect_position.y = $Spatial/Camera/Menu/Control/inner_box/box_of_pattern/Control.rect_position.y + (30 * counter)
						if len(x) == 2:
							new_node.get_child(0).text = str(x[0][0])
							new_node.get_child(1).text = str(x[0][1])
							new_node.get_child(2).text = str(x[0][2])
							new_node.get_child(3).text = str(x[1][0])
							new_node.get_child(4).text = str(x[1][1])
							new_node.get_child(5).text = str(x[1][2])
						else:
							print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
							print("The last array: ", x, " and the name of morphology: ", $Spatial/Camera/Menu/Control/inner_box/morphology_name.text)
							break
					elif i == "vectors":
#						$Spatial/Camera/Menu/Control/create.visible = false
#						$Spatial/Camera/Menu/Control/update.visible = true
						counter += 1
						var new_node = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.duplicate()
						$Spatial/Camera/Menu/Control/inner_box/box_of_vectors.add_child(new_node)
						new_morphology_node.append(new_node)
						new_node.visible = true
						new_node.get_child(3).connect("pressed", self, "delete_morphology", [new_node])
						new_node.rect_position.x = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.rect_position.x
						new_node.rect_position.y = $Spatial/Camera/Menu/Control/inner_box/box_of_vectors/Control.rect_position.y + (30 * counter)
						new_node.get_child(0).value = int(x[0])
						new_node.get_child(1).value = int(x[1])
						new_node.get_child(2).value = int(x[2])
						if len(new_morphology_node) > 4:
							$Spatial/Camera/Menu/Control/inner_box/grey_bg.rect_size.y += (4 * counter)
							$Spatial/Camera/Menu/Control/ColorRect.rect_size.y += (4 * counter)
							$Spatial/Camera/Menu/Control/update.rect_position.y += (4 * counter)
						$Spatial/Camera/Menu/Control/inner_box/Button.disabled = true

			for x in $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_count():
				if $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_text(x) == i:
					$Spatial/Camera/Menu/rule_properties/rules/rule_type_options.selected = x
					if "*" in new_name:
						new_name = new_name.replace("*", "\""+"*"+"\"")
						$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/morphology_def.text = new_name
						flag = true
					if "?" in new_name:
						flag = true
						new_name = new_name.replace("?", "\""+"?"+"\"")
						$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/morphology_def.text = new_name
					if flag == false:
						$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/morphology_def.text = new_name


func _morphology_button_inside_red():
	var i = $Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_item_text($Spatial/Camera/Menu/rule_properties/rules/rule_type_options.get_selected_id())
	var counter = 0
	if i == "patterns":
		counter = len(new_morphology_node)
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = true
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = false
		var new_node = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.duplicate()
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(6).connect("pressed", self, "delete_morphology", [new_node])
		new_node.rect_position.x = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.rect_position.x
		new_node.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label/Control.rect_position.y + (25 * counter)
		new_node.get_child(0).text = ""
		new_node.get_child(1).text = ""
		new_node.get_child(2).text = ""
		new_node.get_child(3).text = ""
		new_node.get_child(4).text = ""
		new_node.get_child(5).text = ""
	elif i == "vectors":
		counter = len(new_morphology_node)
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/pattern_label.visible = false
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/labels.visible = true
		var new_node = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.duplicate()
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(3).connect("pressed", self, "delete_morphology", [new_node])
		new_node.rect_size = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.rect_size
		new_node.rect_position.x = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.rect_position.x
		new_node.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/vectors_label/Control.rect_position.y + (30 * counter)
		new_node.get_child(0).value = 0
		new_node.get_child(1).value = 0
		new_node.get_child(2).value = 0
	if counter > 4:
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/Label2.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/Label2.rect_position.y + (3 * counter)
		$Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.rect_position.y = $Spatial/Camera/Menu/rule_properties/rules/morphology_definition/associations_data.rect_position.y + (3 * counter)
		$Spatial/Camera/Menu/rule_properties/rules.rect_size.y += (counter * 3)
		$Spatial/Camera/Menu/rule_properties.rect_size.y += (counter * 3)
		$Spatial/Camera/Menu/rule_properties/rules/save.rect_position.y -= 2
		$Spatial/Camera/Menu/rule_properties/rules/delete.rect_position.y -=2
