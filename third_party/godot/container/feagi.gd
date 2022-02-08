extends Spatial


onready var file = 'res://csv_data.csv'
onready var textbox_display = get_node("Sprite3D")
onready var gridmap_new = get_node("GridMap2")
onready var selected =  preload("res://selected.meshlib")
onready var deselected = preload("res://Cortical_area_box.meshlib")
onready var duplicate_model = get_node("Cortical_area")


var grid_size = 1000
var grid_steps = 1000
var flag = 0
var test = 0
var data
var socket = PacketPeerUDP.new()
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
var green_light = false

var udp := PacketPeerUDP.new()
var connected = false





func _ready():
	Engine.target_fps = 20
#	for _i in self.get_children():
#		print(_i)
	if(socket.listen(20001, "127.0.0.1") != OK):
		print("error")
	else:
		print("connecting...")
		print("get_node: connected_UDP.")
		print("setup_local_to_scene loaded.")
		print("Connection established.")
		green_light  = true

	$GridMap2.clear()
	var f = File.new() #This is to read each line from the file
	if f.file_exists('res://csv_data.csv'):
		f.open(file, File.READ)
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
				var copy = duplicate_model.duplicate() ##What is this?
				var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
				var viewport = create_textbox.get_node("Viewport")
				create_textbox.set_texture(viewport.get_texture())
				add_child(copy) ##Was this previous one?
				create_textbox.set_name(name + "_textbox")
				add_child(create_textbox)#Copied the node to new node
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
		for i in 6:
			$GridMap3.set_cell_item(i,0,0,0) ##set the arrow indicator of 3D
		var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
		var viewport = create_textbox.get_node("Viewport")
		create_textbox.set_texture(viewport.get_texture())
		create_textbox.set_name("x_textbox")
		add_child(create_textbox)#Copied the node to new node
		create_textbox.scale = Vector3(0.5,0.5,0.5)
		generate_textbox(create_textbox, 5,0,0,"x")
		for j in 6:
			$GridMap3.set_cell_item(0,j,0,0)
		create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
		viewport = create_textbox.get_node("Viewport")
		create_textbox.set_texture(viewport.get_texture())
		create_textbox.set_name("y_textbox")
		add_child(create_textbox)#Copied the node to new node
		create_textbox.scale = Vector3(0.5,0.5,0.5)
		generate_textbox(create_textbox, 0,5,0,"y")
		for k in 6: 
			$GridMap3.set_cell_item(0,0,k,0)
		create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
		viewport = create_textbox.get_node("Viewport")
		create_textbox.set_texture(viewport.get_texture())
		create_textbox.set_name("z_textbox")
		add_child(create_textbox)#Copied the node to new node
		create_textbox.scale = Vector3(0.5,0.5,0.5)
		generate_textbox(create_textbox, -2,0.5,6,"z")
		$GridMap.clear()
		
	
	while green_light:
#		$GridMap.set_cell_item(100, 0, 9,0)
#		$GridMap.set_cell_item(100, 0, 8,0)
#		$GridMap.set_cell_item(100, 0, 7,0)
#		$GridMap.set_cell_item(100, 0, 6,0)
		
		_callout()
		## This will build from one frame
		#print(stored_value)
		if stored_value != "":
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
			yield(get_tree().create_timer(.01), "timeout")
		udp.put_packet("None".to_utf8())
		$GridMap.clear() ##clear the new data


func _process(delta):
	while socket.get_available_packet_count() > 0:
		data = socket.get_packet().get_string_from_utf8()
		stored_value = data
	udp.connect_to_host("127.0.0.1", 20002)
	
		
#	if Input.is_action_just_pressed("ui_del"):
#		var totall= get_node_count()
#		print(totall)
		
		
func _callout():
	_process(self)
	
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
