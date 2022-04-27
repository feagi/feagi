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
onready var gridmap_new = get_node("GridMap2")
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


func _ready():
#	Godot_list.godot_list["data"]
	#Engine.target_fps = 20
	$GridMap2.clear()
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
		generate_voxels(stored_value)
		yield(get_tree().create_timer(0.01), "timeout")
		$GridMap.clear()


func _process(delta):
	check_csv() ##Check if csv is changed
	data = websocket.one_frame
	
	
func generate_model(node, x, y, z, width, depth, height, name):
	var new_grid = gridmap_new.duplicate()
	new_grid.set_name(name)
	add_child(new_grid)
	for x_increment in width:
		for y_increment in height:
			for z_increment in depth:
				if x_increment == 0 or x_increment == (int(width)-1) or y_increment == 0 or y_increment == (int(height) - 1) or z_increment == 0 or z_increment == (int(depth) - 1):
					#new_grid.set_cell_item(x_increment+int(x), y_increment+int(y), z_increment+int(z), 0)
					var new = get_node("Cortical_area").duplicate()
					new.set_name(name)
					add_child(new)
					global_name_list.append(new)
					new.transform.origin = Vector3(x_increment+int(x), y_increment+int(y), z_increment+int(z))
					generate_textbox(node, x,height,z, name)

func generate_textbox(node, x,height,z, name):
	node.transform.origin = Vector3(int(x) + (width/1.5), int(int(y)+2 + (height)),z)
	node.get_node("Viewport/Label").set_text(str(name))
	node.get_node("Viewport").get_texture()

func adding_cortical_areas(name, x,y,z,width,height,depth):
	cortical_area[name]=[x,y,z,width,height,depth] ##This is just adding the list
	
func install_voxel_inside(x,y,z):
	$GridMap.set_cell_item(x,y,z, 0)

func _csv_generator():
	_clear_node_name_list(global_name_list)
	var f = File.new() #This is to read each line from the file
	if f.file_exists('res://csv_data.gdc'):
		f.open(file, File.READ)
		stored_csv = f.get_as_text()
		var index = 1
		while not f.eof_reached(): # iterate through all lines until the end of file is reached
			var line = f.get_line()
			if line != "":
				line += " "
				#print(line)
				var CSV_data = line.split(",", true, '0') ##splits into value array per line
				x = CSV_data[0]
				y = CSV_data[1]
				z = CSV_data[2]
				width= int(CSV_data[3])
				height = int(CSV_data[4])
				depth = int(CSV_data[5])
				name = CSV_data[6]
				$Floor_grid.set_cell_item(int(x),0,int(z),0)
				if sign(int(width)) > 0:
					x_increment = (int(width) / floor_size) + 1
					for i in x_increment:
						print("generated: " , i, " for ", name)
						$Floor_grid.set_cell_item(int(x)+(i*floor_size),0,0,0)
				if sign(int(width)) < 0:
					x_increment = (int(width) / floor_size) - 1
					for i in range(0, x_increment):
						print("generated: " , i, " for ", name)
						$Floor_grid.set_cell_item(int(x)+(-1*i*floor_size),0,0,0)
#				$Floor_grid.set_cell_item(0,0,int(z),0)
				if sign(int(depth)) > 0:
					z_increment = (int(depth) / floor_size) + 1
					for i in z_increment:
						print("generated: " , i, " for ", name)
						$Floor_grid.set_cell_item(int(x),0,int(z)+(i*floor_size),0)
				if sign(int(width)) < 0:
					z_increment = (int(depth) / floor_size) - 1
					for i in range(0, z_increment):
						print("generated: " , i, " for ", name)
						$Floor_grid.set_cell_item(int(x),0,int(z) + (-1*i*floor_size),0)
				
#				if sign(int(x)) >= 0:
#					print("+name: ", name, " BWUK: ", x_increment)
#					x_increment = (int(width) / floor_size) + 1
#					for i in x_increment:
#						$Floor_grid.set_cell_item(i*floor_size,0,0,0)
#				if sign(int(x)) <= 0:
#					print("-name: ", name, " BWUK: ", x_increment)
#					x_increment = (int(width) / floor_size) - 1
#					for i in range(0, x_increment):
#						$Floor_grid.set_cell_item(i*floor_size,0,0,0)
#						print("generated: " , i, " for ", name)
#				if sign(int(z)) >= 0:
#					z_increment = (int(depth) / floor_size) + 1
#					for i in z_increment:
#						$Floor_grid.set_cell_item(int(x),0,i*floor_size,0)
#				if sign(int(z)) <= 0:
#					z_increment = (int(depth) / floor_size) - 1
#					var i = 0
#					while i >= z_increment:
#						$Floor_grid.set_cell_item(int(x),0,i*floor_size,0)
#						i -= 1
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
				generate_model(create_textbox, x,y,z,width, depth, height, name)
				# copy.queue_free() #This acts like .clear() but for CSGBox
				if cortical_area.empty(): #Checks if dict is empty
					adding_cortical_areas(name,x,y,z,height,width,depth) #adding to dict
					cortical_area_stored = cortical_area
				elif cortical_area.hash() == cortical_area_stored.hash():
					adding_cortical_areas(name,x,y,z,height,width,depth)
				index += 1
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
		#print("FALSE!")
		var check = File.new()
		if check.file_exists('res://csv_data.gdc'):
			csv_flag = true
			check.open(file, File.READ)
			var current_csv = check.get_as_text()
			check.close()
			print(stored_csv)
			if stored_csv != current_csv:
				_csv_generator()
				stored_csv = current_csv
	else:
		#print("TRUE!")
		var check = File.new()
		check.open(file, File.READ)
		var current_csv = check.get_as_text()
		check.close()
		if stored_csv != current_csv:
			stored_csv = current_csv
			_csv_generator()
			
func generate_voxels(data):
	if stored_value != "" and stored_value != null:
		#print("ENTERED!")
		var array_test = stored_value.replace("[", "")
		array_test = array_test.replace("]", "")
		test=array_test.split(",", true, '0')
		total = (test.size())
		#print(test)
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
				#check_cortical_area(x,y,z)
				install_voxel_inside(x,y,z) #install voxel inside cortical area
			key+= 1
		flag = 0 #keep x,y,z in correct place
