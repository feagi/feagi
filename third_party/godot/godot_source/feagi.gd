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


onready var file = 'res://csv_data.gdc'
onready var textbox_display = get_node("Sprite3D")
onready var selected =  preload("res://selected.meshlib")
onready var deselected = preload("res://Cortical_area_box.meshlib")
onready var duplicate_model = get_node("Cortical_area")


var floor_size = 25
var grid_steps = 1000
var flag = 0
var test = 0
var data
var stored_value = ""
var current_pos = Vector3()
var x = 0
var y = 1
var z = 2
var total = []
var array_test = []
var number = 0
var voxel_history = 0
var history_total = 0
var depth = 0
var height = 0
var width = 0
var cortical_area = {}
var cortical_area_stored = {}
var Godot_list = {}
var x_increment = 0
var y_increment = 0
var z_increment = 0
var csv_flag = false
var connected = false
var stored_csv = ""
var global_name_list = []
var global_id
var start = 0 #for timer
var end = 0 #for timer
var increment_gridmap = 0


func _ready():
	set_physics_process(false)
	for i in 6:
		$GridMap3.set_cell_item(i,0,0,0) ##set the arrow indicator of 3D
	var create_textbox_axis = textbox_display.duplicate() #generate a new node to re-use the model
	var viewport = create_textbox_axis.get_node("Viewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("x_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(0.5,0.5,0.5)
	generate_textbox(create_textbox_axis, 5,0,0,"x")
	for j in 6:
		$GridMap3.set_cell_item(0,j,0,0)
	create_textbox_axis = create_textbox_axis.duplicate() #generate a new node to re-use the model
	viewport = create_textbox_axis.get_node("Viewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("y_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(0.5,0.5,0.5)
	generate_textbox(create_textbox_axis, 0,5,0,"y")
	for k in 6: 
		$GridMap3.set_cell_item(0,0,k,0)
	create_textbox_axis = textbox_display.duplicate() #generate a new node to re-use the model
	viewport = create_textbox_axis.get_node("Viewport")
	create_textbox_axis.set_texture(viewport.get_texture())
	create_textbox_axis.set_name("z_textbox")
	add_child(create_textbox_axis)#Copied the node to new node
	create_textbox_axis.scale = Vector3(0.5,0.5,0.5)
	generate_textbox(create_textbox_axis, -2,0.5,6,"z")
	$GridMap.clear()

	_csv_generator()
	
	while true:
		_process(self)
		stored_value = data
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
	add_child(new)
	global_name_list.append(new)
	new.scale = Vector3(width_input, height_input, depth_input)
	new.transform.origin = Vector3(width_input/2 + int(x_input), height_input/2+ int(y_input), depth_input/2 + int(z_input))
	generate_textbox(node, x_input,height_input,z_input, name_input)
	
	
func generate_model(node, x_input, y_input, z_input, width_input, depth_input, height_input, name_input):
	for x_gain in width_input:
		for y_gain in height_input:
			for z_gain in depth_input:
				if x_gain == 0 or x_gain == (int(width_input)-1) or y_gain == 0 or y_gain == (int(height_input) - 1) or z_gain == 0 or z_gain == (int(depth_input) - 1):
					var new = get_node("Cortical_area").duplicate()
					new.set_name(name_input)
					add_child(new)
					global_name_list.append(new)
					new.transform.origin = Vector3(x_gain+int(x_input), y_gain+int(y_input), z_gain+int(z_input))
					generate_textbox(node, x_input,height_input,z_input, name_input)

func generate_textbox(node, x_input,height_input,z_input, name_input):
	node.transform.origin = Vector3(int(x_input) + (width/1.5), int(int(y)+2 + (height_input)),z_input)
	node.get_node("Viewport/Label").set_text(str(name_input))
	node.get_node("Viewport").get_texture()

func adding_cortical_areas(name, x_input,y_input,z_input,width_input,height_input,depth_input):
	cortical_area[name]=[x_input,y_input,z_input,width_input,height_input,depth_input] ##This is just adding the list
	
func install_voxel_inside(x_input,y_input,z_input):
	$GridMap.set_cell_item(x_input,y_input,z_input, 0)

func _csv_generator():
	_clear_node_name_list(global_name_list)
	var f = File.new() #This is to read each line from the file
	if f.file_exists('res://csv_data.gdc'):
		f.open(file, File.READ)
		stored_csv = f.get_as_text()
		while not f.eof_reached(): # iterate through all lines until the end of file is reached
			var line = f.get_line()
			if line != "":
				line += " "
				var CSV_data = line.split(",", true, '0') ##splits into value array per line
				x = CSV_data[0]; y = CSV_data[1]; z = CSV_data[2]; width= int(CSV_data[3]) 
				height = int(CSV_data[4]); depth = int(CSV_data[5]); name = CSV_data[6]
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
				global_name_list.append(copy)
				create_textbox.set_name(name + "_textbox")
				add_child(create_textbox)#Copied the node to new node
				global_name_list.append(create_textbox)
				create_textbox.scale = Vector3(1,1,1)
				if int(width) * int(depth) * int(height) < 999: # Prevent massive cortical area 
					generate_model(create_textbox, x,y,z,width, depth, height, name)
				else:
					generate_one_model(create_textbox, x,y,z,width, depth, height, name)
				# copy.queue_free() #This acts like .clear() but for CSGBox
				if cortical_area.empty(): #Checks if dict is empty
					adding_cortical_areas(name,x,y,z,height,width,depth) #adding to dict
					cortical_area_stored = cortical_area
				elif cortical_area.hash() == cortical_area_stored.hash():
					adding_cortical_areas(name,x,y,z,height,width,depth)
		f.close()
	else:
		csv_flag = false
		
func _clear_node_name_list(node_name):
	var list = node_name
	if list.empty() != true:
		for search_name in list:
			print(search_name)
			search_name.queue_free()
	global_name_list = []
	$Floor_grid.clear()

func check_csv():
	if csv_flag == false:
		var check = File.new()
		if check.file_exists('res://csv_data.gdc'):
			csv_flag = true
			check.open(file, File.READ)
			var current_csv = check.get_as_text()
			check.close()
			#print(stored_csv)
			if stored_csv != current_csv:
				_csv_generator()
				stored_csv = current_csv
	else:
		var check = File.new()
		check.open(file, File.READ)
		var current_csv = check.get_as_text()
		check.close()
		if stored_csv != current_csv:
			stored_csv = current_csv
			_csv_generator()
			
func generate_voxels():
	if stored_value != "" and stored_value != null:
		array_test = stored_value.replace("[", "")
		array_test = array_test.replace("]", "")
		test=array_test.split(",", true, '0')
		total = (test.size())
		$red_voxel.multimesh.instance_count = total
		$red_voxel.multimesh.visible_instance_count = total
		var key = 0
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
