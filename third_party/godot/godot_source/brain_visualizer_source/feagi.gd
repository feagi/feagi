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
	SEC = 'HTTP://' + network_setting.api_ip_address + ':' + network_setting.api_port_address
	set_physics_process(false)
#	add_3D_indicator()
	Autoload_variable.BV_Core.Update_Dimensions() # Grab genome list
	Autoload_variable.BV_Core.Update_Morphology_type()

	while true:
		if Godot_list.genome_data["genome"] != previous_genome_data:
			previous_genome_data = Godot_list.genome_data["genome"].duplicate()
			_csv_generator()
		if $".."/".."/".."/Menu/box_loading.visible:
			$".."/".."/".."/Menu/box_loading.visible = false
		if cortical_is_clicked():
			pass
		elif select_cortical.selected.is_empty() != true:
			select_cortical.selected.pop_front()
		_process(self)
#		print("FROM PYTHON: ", data)
		if data != null:
			if "update" in data:
				$".."/".."/".."/Menu/box_loading.visible = true
				Autoload_variable.BV_Core.Update_Dimensions()
				Autoload_variable.BV_Core.GET_health_status()
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
		grab_id_cortical = name_to_id(iteration_name)
		update_cortical_map_name(grab_id_cortical)
		var LeftBarDict = HelperFuncs.GenerateDefinedUnitDict("LEFTBAR", $"..".currentLanguageISO)
		$"..".SpawnLeftBar(grab_id_cortical, LeftBarDict)
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

func _on_Update_pressed(data_input):
	var x = data_input.GetReferenceByID("XYZ").get_node("counter_Pos_X").get_node("counter_Pos_X").value;
	var y = data_input.GetReferenceByID("XYZ").get_node("counter_Pos_Y").get_node("counter_Pos_Y").value;
	var z = data_input.GetReferenceByID("XYZ").get_node("counter_Pos_Z").get_node("counter_Pos_Z").value;
	var id_input = str(data_input.GetReferenceByID("CorticalID").get_node("sideLabel_CorticalID").text);
	var width= data_input.GetReferenceByID("WHD").get_node("counter_W").get_node("counter_W").value;
	var height = data_input.GetReferenceByID("WHD").get_node("counter_H").get_node("counter_H").value;
	var depth = data_input.GetReferenceByID("WHD").get_node("counter_D").get_node("counter_D").value;
	var cortical_neuron_per_vox_count = data_input.GetReferenceByID("VoxelNeuronDensity").get_node("counter_VoxelNeuronDensity").value;
	var synaptic_attractivity = data_input.GetReferenceByID("SynapticAttractivity").get_node("counter_SynapticAttractivity").value;
	var post_synaptic_potential = data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("PostSynapticPotential").get_node("floatField_PostSynapticPotential").value;
	var post_synaptic_potential_max = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("PSPMax").get_node("floatField_PSPMax").value);
	var plasticity_coef = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("PlasticityConstant").get_node("floatField_PlasticityConstant").value);
	var fire_threshold = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("FireThreshold").get_node("floatField_FireThreshold").value);
	var fire_threshold_limit = data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("Thresholdlimit").get_node("counter_Thresholdlimit").value;
	var refractory_period = data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("RefactoryPeriod").get_node("counter_RefactoryPeriod").value;
	var leak_coefficient = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("LeakConstant").get_node("floatField_LeakConstant").value);
	var leak_variability = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("LeakVaribility").get_node("floatField_LeakVaribility").value);
	var fire_threshold_increment = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("ThresholdINC").get_node("floatField_ThresholdINC").value);
	var consecutive_fire_count = data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("ConsecutiveFireCount").get_node("counter_ConsecutiveFireCount").value;
	var snooze_period = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("SnoozePeriod").get_node("floatField_SnoozePeriod").value);
	var degenerecy_coefficient = float(data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("DegeneracyConstant").get_node("floatField_DegeneracyConstant").value);
	var psp_uniform_distribution = data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("PSPUNI").get_node("checkButton_PSPUNI").is_pressed()
	var MP_accumulation = data_input.GetReferenceByID("NeuronParametersSection").GetReferenceByID("ChargeACC").get_node("checkButton_ChargeACC").is_pressed()
	var name_input = data_input.GetReferenceByID("CorticalName").get_node("sideLabel_CorticalName").text
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
#	last_cortical_selected["cortical_destinations"] = {}
	last_cortical_selected["cortical_dimensions"] = []

	last_cortical_selected["cortical_id"]= id_input
	last_cortical_selected["cortical_name"] = name_input
#	last_cortical_selected["cortical_group"] = last_cortical_selected["cortical_group"]
	last_cortical_selected["cortical_neuron_per_vox_count"] = cortical_neuron_per_vox_count
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
	create_textbox_axis.scale = Vector3(3, 3, 3)
	generate_textbox(create_textbox_axis, 18,0,0,"x", 0, 0, 0)
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
	generate_textbox(create_textbox_axis, 0,0,-6,"z", 1, 0, 0)
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
		print("STILL USING")
		Autoload_variable.BV_Core.Update_Afferent_list(genome_properties["cortical_id"])
#	$"..".SpawnLeftBar()


func _on_send_feagi_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	$notification.generate_notification_message(api_data, _response_code, "_on_send_feagi_request_completed", latest_send_feagi, type)

func _on_info_pressed():
	for i in plus_node:
		i.queue_free()
	plus_node.clear()

func update_cortical_map_name(name_input):
	Autoload_variable.BV_Core.Update_Efferent_information(name_input)

func _on_information_button_request_completed(_result, _response_code, _headers, body):
	# Do not touch here. THis is for information iteration_namebutton only and will dedicate
	# to the information button
	# Clear duplicate cortical maps name up
	child_holder_clear()
	# Obtain the data from API and convert it into json/string
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	
	if _response_code == 200 and not api_data.has("Request failed..."):
		var new_name = ""
		var UI_LeftBar = $"..".UI_LeftBar
		for i in api_data:
			var new_node = UI_LeftBar.GetReferenceByID("blank_efferent").get_node("button_blank_efferent").duplicate()
			UI_LeftBar.GetReferenceByID("blank_efferent").add_child(new_node)
			child_node_holder.append(new_node)
			new_name = id_to_name(i)
			new_node.text = new_name
			new_node.connect("pressed",Callable($"..","mapping_definition_button").bind(new_node))
#		map_colorful()
	#	$".."/".."/".."/Menu/cortical_menu/Control/Update.position.y = 10 + $".."/".."/".."/Menu/cortical_mapping/Control/ScrollContainer/VBoxContainer.size.y + $".."/".."/".."/Menu/cortical_mapping.position.y
	else:
		$notification.generate_notification_message(api_data, _response_code, "_on_information_button_request_completed", "/v1/feagi/genome/cortical_mappings/efferents")

func child_holder_clear():
	# Clear duplicate cortical maps name up
	if child_node_holder:
		for i in child_node_holder:
			if i == null:
				pass
			else:
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

func _on_add_pressed(node=[]):
	print("HERE!")
	var json_data = {}
	if node == []:
		if $".."/".."/".."/Menu/addition_menu/OptionButton.selected == 1 or $".."/".."/".."/Menu/addition_menu/OptionButton.selected == 2:
			json_data["cortical_type"] = $".."/".."/".."/Menu/addition_menu/OptionButton.get_item_text($".."/".."/".."/Menu/addition_menu/OptionButton.selected)
			json_data["cortical_name"] = $".."/".."/".."/Menu/addition_menu/cortical_name_label/type.get_item_text($".."/".."/".."/Menu/addition_menu/cortical_name_label/type.selected)
			json_data["cortical_coordinates"] = []
			json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value)
			json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value)
			json_data["cortical_coordinates"].append($".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value)
			json_data["channel_count"] = $".."/".."/".."/Menu/addition_menu/count/count_spinbox.value
			Autoload_variable.BV_Core.Update_cortical_area(json_data)
	else:
		if node[7].selected == 3:
#			if $".."/".."/".."/Menu/addition_menu/cortical_name_textbox/type.text != "" and $".."/".."/".."/Menu/addition_menu/cortical_name_textbox/type.text != " ":
			json_data["cortical_type"] = "CUSTOM"
			json_data["cortical_name"] = node[6].text
			json_data["cortical_coordinates"] = []
			json_data["cortical_dimensions"] = []
			json_data["cortical_coordinates"].append(node[3].value)
			json_data["cortical_coordinates"].append(node[4].value)
			json_data["cortical_coordinates"].append(node[5].value)
			json_data["cortical_dimensions"].append(node[0].value)
			json_data["cortical_dimensions"].append(node[1].value)
			json_data["cortical_dimensions"].append(node[2].value)
			generate_single_cortical(json_data["cortical_coordinates"][0], json_data["cortical_coordinates"][1], json_data["cortical_coordinates"][2], json_data["cortical_dimensions"][0], json_data["cortical_dimensions"][1], json_data["cortical_dimensions"][2], json_data["cortical_name"])
			Autoload_variable.BV_Core.Update_custom_cortical_area(json_data)
			node[8].release_focus()
			$Node3D/Camera3D.transform.origin=Vector3(json_data["cortical_coordinates"][0]-20,json_data["cortical_coordinates"][1],json_data["cortical_coordinates"][2]+20)
			$"..".UI_createcorticalBar.queue_free()
	Autoload_variable.BV_Core.Update_CorticalAreaNameList()

func _on_remove_pressed(node):
	var get_name_data = node.get_node("sideLabel_CorticalID").text
	_clear_single_cortical(id_to_name(get_name_data), global_name_list)
	Autoload_variable.BV_Core.Delete_cortical_area(get_name_data)
#	$".."/".."/".."/Menu/cortical_menu/Control/remove.release_focus()

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
				#INCOMPLETE
				var UI_MappingDefinition = $"..".UI_MappingDefinition
				var new_node = UI_MappingDefinition.GetReferenceByID("third_box").duplicate()
				new_node.set_name("Unit_third_box" + str(i))
				UI_MappingDefinition.add_child(new_node)
				plus_node.append(new_node)
				# Waiting on toggle
#				new_node.get_child(0).connect("pressed",Callable(self,"_on_Mapping_def_pressed"))
#				new_node.get_child(4).text_changed.connect(_on_text_changed.bind(new_node.get_child(4)))
				ghost_morphology.append(new_node)
				var dropdown = new_node.get_node("dropdown_mappingdefinitions").get_node("dropDown_mappingdefinitions")
				for x in dropdown.get_item_count():
					if dropdown.get_item_text(x) == api_data[i]["morphology_id"]:
						dropdown.selected = x
				new_node.visible = true
				new_node.get_node("box_overwritescalar").get_node("box_XYZ").get_child(0).get_child(1).value = api_data[i]["morphology_scalar"][0]
				new_node.get_node("box_overwritescalar").get_node("box_XYZ").get_child(1).get_child(1).value = api_data[i]["morphology_scalar"][1]
				new_node.get_node("box_overwritescalar").get_node("box_XYZ").get_child(2).get_child(1).value = api_data[i]["morphology_scalar"][2]
				new_node.get_node("floatfield_PSPMULTIPLER").get_node("floatField_PSPMULTIPLER").text = str(api_data[i]["postSynapticCurrent_multiplier"])
				new_node.get_node("box_third_box1").get_node("checkbutton_PSPTOGGLE").get_node("checkButton_PSPTOGGLE").set_pressed(api_data[i]["plasticity_flag"])
				new_node.get_node("button_mappingdefbuttonminus").get_node("button_mappingdefbuttonminus").connect("pressed",Callable(self,"map_info_pressed").bind(new_node))
				new_node.get_node("button_mappingdefbuttonminus").get_node("sideButton_mappingdefbuttonminus").connect("pressed",Callable(self,"remove_button_inside_dst").bind(new_node))
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
	var name_selected = node_duplicated.get_node("dropdown_mappingdefinitions").get_node("dropDown_mappingdefinitions").get_item_text(node_duplicated.get_node("dropdown_mappingdefinitions").get_node("dropDown_mappingdefinitions").selected)
	Autoload_variable.BV_Core.Get_Morphology_information(name_selected)

func remove_button_inside_dst(node_duplicated):
	plus_node.erase(node_duplicated)
	node_duplicated.queue_free()

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
	print("erasing: ", input_node)
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
	var UI_MappingDefinition = $"..".UI_MappingDefinition
	var new_node = UI_MappingDefinition.GetReferenceByID("third_box").duplicate()
	var counter = 0
	for i in UI_MappingDefinition.get_children():
		if "Unit_third_box" in i.get_name():
			counter += 1
	new_node.set_name("Unit_third_box" + str(counter))
	UI_MappingDefinition.add_child(new_node)
	plus_node.append(new_node)
	# Waiting on toggle
#				new_node.get_child(0).connect("pressed",Callable(self,"_on_Mapping_def_pressed"))
#				new_node.get_child(4).text_changed.connect(_on_text_changed.bind(new_node.get_child(4)))
	ghost_morphology.append(new_node)
	new_node.visible = true
	new_node.get_node("box_overwritescalar").get_node("box_XYZ").get_child(0).get_child(1).value = 1
	new_node.get_node("box_overwritescalar").get_node("box_XYZ").get_child(1).get_child(1).value = 1
	new_node.get_node("box_overwritescalar").get_node("box_XYZ").get_child(2).get_child(1).value = 1
	new_node.get_node("floatfield_PSPMULTIPLER").get_node("floatField_PSPMULTIPLER").text = str(1)
	new_node.get_node("box_third_box1").get_node("checkbutton_PSPTOGGLE").get_node("checkButton_PSPTOGGLE").set_pressed(false)
	new_node.get_node("button_mappingdefbuttonminus").get_node("button_mappingdefbuttonminus").connect("pressed",Callable(self,"map_info_pressed").bind(new_node))
	new_node.get_node("button_mappingdefbuttonminus").get_node("sideButton_mappingdefbuttonminus").connect("pressed",Callable(self,"remove_button_inside_dst").bind(new_node))

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

func _on_cortical_mapping_add_pressed(name_input):
	var mappingdefinitiongenerated = HelperFuncs.GenerateDefinedUnitDict("MAPPING_DEFINITION", $"..".currentLanguageISO)
	$"..".SpawnMappingDefinition(id_to_name(name_input), "", mappingdefinitiongenerated)

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
	if name_input != "":
		for i in Godot_list.genome_data["genome"]:
			if i == iteration_name:
				grab_id_cortical = Godot_list.genome_data["genome"][i][7]
				return grab_id_cortical
	else:
		return ""
			
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

func _on_create_pressed(node):
	var name_input = node.GetReferenceByID("MorphologyName").get_node("field_MorphologyName").text
	var dropdown_selected = node.GetReferenceByID("MorphologyType").get_node("dropDown_MorphologyType").text
	if name_input != "":
		if dropdown_selected == "Patterns":
			var json = {}
			var new_name = symbols_checker_for_api(name_input)
			var new_type = dropdown_selected
			var string_input = []
			var full_array = []
			var empty_flag = 0
			for i in new_morphology_node:
				print("new morp node: ", new_morphology_node)
				empty_flag = 0
				full_array = []
				var empty_array1 = [i.get_node("floatfield_Xi").get_node("floatField_Xi").text, i.get_node("floatfield_Yi").get_node("floatField_Yi").text, i.get_node("floatfield_Zi").get_node("floatField_Zi").text]
				var empty_array2 = [i.get_node("floatfield_Xo").get_node("floatField_Xo").text, i.get_node("floatfield_Yo").get_node("floatField_Yo").text, i.get_node("floatfield_Zo").get_node("floatField_Zo").text]
				var symbols_to_check = ["?", "*"]
				for x in empty_array1:
					if x in symbols_to_check:
						empty_array1[empty_flag] = str(x)
					else:
						empty_array1[empty_flag] = int(x)
					empty_flag += 1
				empty_flag = 0
				for b in empty_array2:
					if b in symbols_to_check:
						empty_array2[empty_flag] = str(b)
					else:
						empty_array2[empty_flag] = int(b)
					empty_flag += 1
				full_array.append(empty_array1)
				full_array.append(empty_array2)
				string_input.append(full_array)
			json["patterns"] = string_input
			var combine_url = '/v1/feagi/genome/morphology?morphology_name=' + str(new_name) + '&morphology_type=' + str(new_type.to_lower())
			Autoload_variable.BV_Core.POST_Request_Brain_visualizer(SEC+combine_url, json)
			new_morphology_clear()
		if dropdown_selected == "Vectors":
			var json = {}
			var new_name = symbols_checker_for_api(name_input)
			var new_type = dropdown_selected
			var empty_array1 = []
			for i in new_morphology_node:
				var temp_array = [int(i.get_node("counter_X").get_node("counter_X").value), int(i.get_node("counter_Y").get_node("counter_Y").value), int(i.get_node("counter_Z").get_node("counter_Z").value)]
				empty_array1.append(temp_array)
			json["vectors"] = empty_array1
			var combine_url = '/v1/feagi/genome/morphology?morphology_name=' + new_name + '&morphology_type=' + new_type.to_lower()
			Autoload_variable.BV_Core.POST_Request_Brain_visualizer(SEC+combine_url, json)
			new_morphology_clear()
		if dropdown_selected == "Composite":
			var json = {}
			var new_name = symbols_checker_for_api(name_input)
			var new_type = dropdown_selected
			json["src_seed"] = []
			for i in node.GetReferenceByID("inner_composite").get_children():
				json["src_seed"].append(int(i.get_child(0).text)) 
			json["src_pattern"] = [[node.GetReferenceByID("Composite").get_node("box_XYZ_s1").get_node("box_XYZ_1").get_node("box_XYZ_X").get_node("counter_C1").get_node("counter_C1").value, node.GetReferenceByID("Composite").get_node("box_XYZ_s1").get_node("box_XYZ_1").get_node("box_XYZ_X").get_node("counter_S1").get_node("counter_S1").value], [node.GetReferenceByID("Composite").get_node("box_XYZ_s1").get_node("box_XYZ_Y").get_node("box_XYZ_2").get_node("counter_C2").get_node("counter_C2").value, node.GetReferenceByID("Composite").get_node("box_XYZ_s1").get_node("box_XYZ_Y").get_node("box_XYZ_2").get_node("counter_S2").get_node("counter_S2").value], [node.GetReferenceByID("Composite").get_node("box_XYZ_s1").get_node("box_XYZ_Z").get_node("box_XYZ_3").get_node("counter_C3").get_node("counter_C3").value, node.GetReferenceByID("Composite").get_node("box_XYZ_s1").get_node("box_XYZ_Z").get_node("box_XYZ_3").get_node("counter_S3").get_node("counter_S3").value]]
			json["mapper_morphology"] = node.GetReferenceByID("Composite").get_node("box_MAPPING_DROPDOWN").get_node("dropdown_MAPPINGDROPDOWN").get_node("dropDown_MAPPINGDROPDOWN").text
			var combine_url = '/v1/feagi/genome/morphology' + '?morphology_name=' + new_name + '&morphology_type=' + new_type.to_lower()
			Autoload_variable.BV_Core.POST_Request_Brain_visualizer(SEC+combine_url, json)
		node.queue_free()
func _on_X_inside_inner_box_pressed():
	$".."/".."/".."/Menu/Control.visible = false
	new_morphology_clear()
	$".."/".."/".."/Menu/Control/inner_box/morphology_name.text = ""

func _on_afferent_request_completed(_result, _response_code, _headers, body):
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	var UI_LeftBar = $"..".UI_LeftBar
	if UI_LeftBar:
		afferent_holder_clear()
		for i in api_data:
			var new_node = UI_LeftBar.GetReferenceByID("blank_afferent").get_node("field_blank_afferent").duplicate()
			UI_LeftBar.GetReferenceByID("blank_afferent").add_child(new_node)
			afferent_child_holder.append(new_node)
			new_node.visible = true
			new_node.text = id_to_name(i)
	$notification.generate_notification_message(api_data, _response_code, "_on_afferent_request_completed", "/v1/feagi/genome/cortical_mappings/afferents")

func afferent_holder_clear():
	# Clear duplicate cortical maps name up
	if afferent_child_holder:
		for i in afferent_child_holder:
			if i == null:
				pass
			else:
				i.queue_free()
		afferent_child_holder = []

func new_morphology_clear():
	if new_morphology_node:
		for i in new_morphology_node:
			if i != null:
				i.queue_free()
		new_morphology_node = []

func plus_node_clear():
	for i in plus_node:
		if i != null:
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

func _on_update_inside_map_pressed(node):
	var combine_url = '?src_cortical_area=#&dst_cortical_area=$'
	var get_id = node.GetReferenceByID("testlabel").get_node("dropdown_SOURCECORTICALAREA").get_node("dropDown_SOURCECORTICALAREA").text
	var get_dst_data = node.GetReferenceByID("testlabel").get_node("dropdown_DESTINATIONCORTICALAREA").get_node("dropDown_DESTINATIONCORTICALAREA").text
	var dst_data = {}
	dst_data["cortical_destinations"] = {}
	combine_url = combine_url.replace("#", get_id)
	combine_url = combine_url.replace("$", get_dst_data)
	for p in plus_node:
		var dst = {}
		dst["morphology_id"] = p.get_node("dropdown_mappingdefinitions").get_node("dropDown_mappingdefinitions").text
		dst["morphology_scalar"] = []
		dst["morphology_scalar"].append(int(p.get_node("box_overwritescalar").get_node("box_XYZ").get_node("floatfield_Pos_X").get_node("floatField_Pos_X").text))
		dst["morphology_scalar"].append(int(p.get_node("box_overwritescalar").get_node("box_XYZ").get_node("floatfield_Pos_Y").get_node("floatField_Pos_Y").text))
		dst["morphology_scalar"].append(int(p.get_node("box_overwritescalar").get_node("box_XYZ").get_node("floatfield_Pos_Z").get_node("floatField_Pos_Z").text))
		dst["postSynapticCurrent_multiplier"] = float(p.get_node("floatfield_PSPMULTIPLER").get_node("floatField_PSPMULTIPLER").text)
		dst["plasticity_flag"] = p.get_node("box_third_box1").get_node("checkbutton_PSPTOGGLE").get_node("checkButton_PSPTOGGLE").is_pressed()
		if dst_data["cortical_destinations"].has(get_id):
			dst_data["cortical_destinations"][get_id].append(dst)
		else:
			dst_data["cortical_destinations"][get_id] = []
			dst_data["cortical_destinations"][get_id].append(dst)
	if dst_data["cortical_destinations"].has(get_id):
		Autoload_variable.BV_Core.Update_Mapping_Properties(dst_data["cortical_destinations"][get_id],combine_url)
	else:
		Autoload_variable.BV_Core.Update_Mapping_Properties([],combine_url)
	node.queue_free()

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
	
func _on_insert_button_pressed(full_data):
	var combine_url = $"../..".SEC + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/append?circuit_name=' + full_data[0].get_item_text(full_data[0].selected) + "&circuit_origin_x=" + str(full_data[1].value) + "&circuit_origin_y=" + str(full_data[2].value) + "&circuit_origin_z=" + str(full_data[3].value)
	var new_data = ["placeholder"]
	Autoload_variable.BV_Core.POST_Request_Brain_visualizer(combine_url, new_data)
#	$"..".import_close_button.emit_signal("pressed") # what happen to this?
	$"..".UI_CircuitImport.queue_free()
	

func _on_circuit_request_request_completed(_result, _response_code, _headers, body):
	$".."/".."/".."/Menu/insert_menu/insert_button/ItemList.clear()
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
	for i in api_data:
		$".."/".."/".."/Menu/insert_menu/insert_button/ItemList.add_item(i, null, true)
	$notification.generate_notification_message(api_data, _response_code, "_on_circuit_request_request_completed", "/v1/feagi/genome/circuits")
	

func _on_import_pressed():
	if not $"..".UI_CircuitImport:
		Autoload_variable.BV_Core.Get_circuit_list()
		var create_circuitimport = HelperFuncs.GenerateDefinedUnitDict("CIRCUIT_IMPORT", $"..".currentLanguageISO)
		$"..".SpawnCircuitImport(create_circuitimport)
	else:
		$"..".UI_CircuitImport.queue_free()
		print("Godot list: ", Godot_list.godot_list)
		_clear_single_cortical("example", Godot_list.godot_list)
		

func _on_ItemList_item_selected(index, node):
	var name_text = node.get_item_text(index)
	name_text = symbols_checker_for_api(name_text)
	Autoload_variable.BV_Core.Get_circuit_size(name_text)

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

func _on_x_spinbox_value_changed(_value, array_data):
	generate_single_cortical(_value, array_data[1].value, array_data[2].value, array_data[3].value, array_data[4].value, array_data[5].value, "example")
	demo_new_cortical()
func _on_y_spinbox_value_changed(_value, array_data):
	generate_single_cortical(array_data[0].value, _value, array_data[2].value, array_data[3].value, array_data[4].value, array_data[5].value, "example")
	demo_new_cortical()
func _on_z_spinbox_value_changed(_value, array_data):
	generate_single_cortical(array_data[0].value, array_data[1].value, _value, array_data[3].value, array_data[4].value, array_data[5].value, "example")
	demo_new_cortical()

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
	if api_data != null:
		for i in api_data:
			string_list = string_list + str(id_to_name(i[0]), " > ", id_to_name(i[1])) + "\n"
		$"..".UI_ManageNeuronMorphology.GetReferenceByID("FIELDFORCURRENTINFO").get_node("textbox_FIELDFORCURRENTINFO").text += str(string_list)
	else:
		$"..".UI_ManageNeuronMorphology.GetReferenceByID("FIELDFORCURRENTINFO").get_node("textbox_FIELDFORCURRENTINFO").text = ""
	$notification.generate_notification_message(api_data, _response_code, "_on_get_morphology_usuage_request_completed", "/v1/feagi/genome/morphology")

func _morphology_button_pressed(node):
	var dropdown_selected = node.get_node("dropdown_MorphologyType").get_node("dropDown_MorphologyType").text
	if dropdown_selected == "Patterns":
		var new_node = node.GetReferenceByID("Patterns").get_node("box_PatternRow0").duplicate()
		node.GetReferenceByID("Patterns").add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_node("button_RemoveSelfRowButton").get_node("button_RemoveSelfRowButton").connect("pressed",Callable(self,"delete_morphology").bind(new_node))
	elif dropdown_selected == "Vectors":
		var new_node = node.GetReferenceByID("Vectors").get_node("box_XYZ").duplicate()
		new_morphology_node.append(new_node)
		new_node.visible = true
		node.GetReferenceByID("Vectors").add_child(new_node)
		new_node.visible = true
		new_node.get_node("button_RemoveRowButton").get_node("button_RemoveRowButton").connect("pressed",Callable(self,"delete_morphology").bind(new_node))


func _morphology_add_row(dropdown, row_node, parent_node, button, create_button):
	var counter = 0
	print("debug: ", dropdown, " and ", row_node, " and ", parent_node)
	counter = len(new_morphology_node)
	if dropdown == "patterns":
		var new_node = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.duplicate()
		$".."/".."/".."/Menu/Control/inner_box/box_of_pattern.add_child(new_node)
		new_morphology_node.append(new_node)
		new_node.visible = true
		new_node.get_child(7).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.position.x = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.position.x + $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.size.x
		new_node.position.y = $".."/".."/".."/Menu/Control/inner_box/box_of_pattern/Control.position.y + (30 * counter)
	elif dropdown == "Vectors":
		var new_node = row_node.duplicate()
		new_morphology_node.append(new_node)
		parent_node.add_child(new_node)
		new_node.visible = true
		print("delete button: ", row_node.get_node("Button_RemoveRowButton").get_child(0))
		new_node.get_node("Button_RemoveRowButton").get_child(0).connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.size = row_node.size
		new_node.position.x = row_node.position.x
		new_node.position.y = row_node.position.y + (30 * counter)
	# This below section needs to rework. This is not even good at all.
	# Start of Section
	button.position.y = new_morphology_node[len(new_morphology_node)-1].position.y + 20
	create_button.position.y = button.position.y + 10
	# End of Section

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
			Autoload_variable.BV_Core.Get_Morphology_information(name_morphology)

func _on_get_morphology_request_completed(_result, _response_code, _headers, body):
	new_morphology_clear()
	flag = false
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8()) # Every http data, it's done in poolbytearray
	var api_data = test_json_conv.get_data()
	var vectors = $"..".UI_ManageNeuronMorphology.GetReferenceByID("Vectors")
	var composite = $"..".UI_ManageNeuronMorphology.GetReferenceByID("Composite")
	var patterns = $"..".UI_ManageNeuronMorphology.GetReferenceByID("Patterns")
	var type_name = " "
	for i in api_data:
		if i == "type":
			for x in api_data["parameters"]:
				if x == "patterns":
					type_name = x
					for a in api_data["parameters"]["patterns"]:
						composite.visible = false; vectors.visible = false;patterns.visible = true
						var new_node = $"..".UI_ManageNeuronMorphology.GetReferenceByID("PatternRow0").duplicate()
						patterns.add_child(new_node)
						new_morphology_node.append(new_node)
						var remove = new_node.get_node("button_RemoveSelfRowButton").get_node("button_RemoveSelfRowButton")
						new_node.visible = true
						remove.connect("pressed",Callable(self,"delete_morphology").bind(new_node))
						if len(a) == 2:
							new_node.get_node("floatfield_Xi").get_node("floatField_Xi").text = str(a[0][0])
							new_node.get_node("floatfield_Yi").get_node("floatField_Yi").text = str(a[0][1])
							new_node.get_node("floatfield_Zi").get_node("floatField_Zi").text = str(a[0][2])
							new_node.get_node("floatfield_Xo").get_node("floatField_Xo").text = str(a[1][0])
							new_node.get_node("floatfield_Yo").get_node("floatField_Yo").text = str(a[1][1])
							new_node.get_node("floatfield_Zo").get_node("floatField_Zo").text = str(a[1][2])
						else:
							print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
							print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id()))
							break
				elif x == "vectors":
					composite.visible = false; vectors.visible = true;patterns.visible = false
					type_name = x
					for a in api_data["parameters"][x]:
						var new_node = $"..".UI_ManageNeuronMorphology.GetReferenceByID("XYZ").duplicate()
						new_morphology_node.append(new_node)
						new_node.visible = true
						var remove = new_node.get_node("button_RemoveRowButton").get_node("button_RemoveRowButton")
						var X_vectors = new_node.get_node("counter_X").get_node("counter_X")
						var Y_vectors = new_node.get_node("counter_Y").get_node("counter_Y")
						var Z_vectors = new_node.get_node("counter_Z").get_node("counter_Z")
						var parent_of_button = $"..".UI_ManageNeuronMorphology.GetReferenceByID("parent_XYZ")
						remove.connect("pressed",Callable(self,"delete_morphology").bind(new_node))
						parent_of_button.add_child(new_node)
						if typeof(a) == 28:
							if len(a) == 3:
								X_vectors.value = int(a[0])
								Y_vectors.value = int(a[1])
								Z_vectors.value = int(a[2])
							else:
								print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
								print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id()))
								break
						else:
							print("This morphology is outdated! Please use the manage neuron morphology to update the morphology.")
							print("The last array: ", x, " and the name of morphology: ", $".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_item_text($".."/".."/".."/Menu/rule_properties/mapping_rule_options.get_selected_id()))
							break
				elif x == "src_seed": # composite
					type_name = api_data[i]
					composite.visible = true; vectors.visible = false;patterns.visible = false
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("C1").get_node("counter_C1").value = api_data['parameters']['src_pattern'][0][0]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("S1").get_node("counter_S1").value = api_data['parameters']['src_pattern'][0][1]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("C2").get_node("counter_C2").value = api_data['parameters']['src_pattern'][1][0]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("S2").get_node("counter_S2").value = api_data['parameters']['src_pattern'][1][1]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("C3").get_node("counter_C3").value = api_data['parameters']['src_pattern'][2][0]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("S3").get_node("counter_S3").value = api_data['parameters']['src_pattern'][2][1]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("X_SEED").get_node("floatField_X_SEED").value = api_data['parameters']['src_seed'][0]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("Y_SEED").get_node("floatField_Y_SEED").value = api_data['parameters']['src_seed'][1]
					$"..".UI_ManageNeuronMorphology.GetReferenceByID("Z_SEED").get_node("floatField_Z_SEED").value = api_data['parameters']['src_seed'][2]
					$"..".UI_ManageNeuronMorphology.SetData({"box_one": {"box_three": {"Composite": {"MAPPING_DROPDOWN": {"MAPPINGDROPDOWN": {"value": api_data['parameters']["mapper_morphology"]}}}}}})
				elif x == "functions":
					composite.visible = false; vectors.visible = false;patterns.visible = false
	$"..".UI_ManageNeuronMorphology.GetReferenceByID("header_type").get_node("field_header_type").text = type_name
	$notification.generate_notification_message(api_data, _response_code, "_on_get_morphology_request_completed", "/v1/feagi/monitoring/neuron/synaptic_potential")


func _morphology_button_inside_red(node):
	var vectors = $"..".UI_ManageNeuronMorphology.GetReferenceByID("Vectors")
	var patterns = $"..".UI_ManageNeuronMorphology.GetReferenceByID("Patterns")
	var i = node.GetReferenceByID("header_type").get_node("field_header_type").text
	if i == "patterns":
		var new_node = $"..".UI_ManageNeuronMorphology.GetReferenceByID("PatternRow0").duplicate()
		patterns.add_child(new_node)
		new_morphology_node.append(new_node)
		var remove = new_node.get_node("button_RemoveSelfRowButton").get_node("button_RemoveSelfRowButton")
		new_node.visible = true
		remove.connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		new_node.get_node("floatfield_Xi").get_node("floatField_Xi").text = str(1)
		new_node.get_node("floatfield_Yi").get_node("floatField_Yi").text = str(1)
		new_node.get_node("floatfield_Zi").get_node("floatField_Zi").text = str(1)
		new_node.get_node("floatfield_Xo").get_node("floatField_Xo").text = str(1)
		new_node.get_node("floatfield_Yo").get_node("floatField_Yo").text = str(1)
		new_node.get_node("floatfield_Zo").get_node("floatField_Zo").text = str(1)
	elif i == "vectors":
		var new_node = $"..".UI_ManageNeuronMorphology.GetReferenceByID("XYZ").duplicate()
		new_morphology_node.append(new_node)
		new_node.visible = true
		var remove = new_node.get_node("button_RemoveRowButton").get_node("button_RemoveRowButton")
		var X_vectors = new_node.get_node("counter_X").get_node("counter_X")
		var Y_vectors = new_node.get_node("counter_Y").get_node("counter_Y")
		var Z_vectors = new_node.get_node("counter_Z").get_node("counter_Z")
		var parent_of_button = $"..".UI_ManageNeuronMorphology.GetReferenceByID("parent_XYZ")
		remove.connect("pressed",Callable(self,"delete_morphology").bind(new_node))
		parent_of_button.add_child(new_node)
		X_vectors.value = 1
		Y_vectors.value = 1
		Z_vectors.value = 1


func _on_close_pressed_def():
	plus_node_clear()
	ghost_morphology_clear()

func _on_menu_itemlist_item_selected(index):
	$Node3D/Camera3D._on_menu_itemlist_item_selected(index)


func _on_grab_location_of_cortical_request_completed(result, response_code, headers, body):
	$Node3D/Camera3D._on_grab_location_of_cortical_request_completed(result, response_code, headers, body)

func _on_X_SpinBox_value_changed(_value, node=[]):
	if node:
		generate_single_cortical(node[3].value, node[4].value, node[5].value, node[0].value, node[1].value, node[2].value, "example")
	else:
		generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_W_Spinbox_value_changed(_value, node=[]):
	if node:
		generate_single_cortical(node[3].value, node[4].value, node[5].value, node[0].value, node[1].value, node[2].value, "example")
	else:
		generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_H_Spinbox_value_changed(_value, node=[]):
	if node:
		generate_single_cortical(node[3].value, node[4].value, node[5].value, node[0].value, node[1].value, node[2].value, "example")
	else:
		generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_D_Spinbox_value_changed(_value, node=[]):
	if node:
		generate_single_cortical(node[3].value, node[4].value, node[5].value, node[0].value, node[1].value, node[2].value, "example")
	else:
		generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_Y_Spinbox_value_changed(_value, node=[]):
	if node:
		generate_single_cortical(node[3].value, node[4].value, node[5].value, node[0].value, node[1].value, node[2].value, "example")
	else:
		generate_single_cortical($".."/".."/".."/Menu/addition_menu/xyz/X_SpinBox.value, $".."/".."/".."/Menu/addition_menu/xyz/Y_Spinbox.value, $".."/".."/".."/Menu/addition_menu/xyz/Z_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/W_Spinbox.value,$".."/".."/".."/Menu/addition_menu/wdh/H_Spinbox.value, $".."/".."/".."/Menu/addition_menu/wdh/D_Spinbox.value, "example")
	demo_new_cortical()

func _on_Z_Spinbox_value_changed(_value, node=[]):
	if node:
		generate_single_cortical(node[3].value, node[4].value, node[5].value, node[0].value, node[1].value, node[2].value, "example")
	else:
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

func camera_list_selected(name_input):
		Autoload_variable.BV_Core.GOTO_CORTICALLOCATION(id_to_name(name_input))

# DE BUG ONLY:
func calculateSceneSize(node: Node) -> int:
	var size = node.get_memory_usage()
	for child in node.get_children():
		size += calculateSceneSize(child)

	return size
