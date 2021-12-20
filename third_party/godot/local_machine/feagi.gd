extends Spatial


onready var file = 'res://csv_data.csv'
onready var textbox_display = get_node("Sprite3D")

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



func _ready():
	if(socket.listen(20001, "127.0.0.1") != OK):
		print("error")
	else:
		print("connecting...")
		print("get_node: connected_UDP.")
		print("setup_local_to_scene loaded.")
		print("Connection established.")
		green_light  = true
	
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
	#			var copy = duplicate_model.duplicate()
				var create_textbox = textbox_display.duplicate() #generate a new node to re-use the model
				var viewport = create_textbox.get_node("Viewport")
				create_textbox.set_texture(viewport.get_texture())
	#			add_child(copy)
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
		$GridMap.clear()
	
	while green_light:
		_callout()
		## This will build from one frame
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
			yield(get_tree().create_timer(.5), "timeout")
		$GridMap.clear() ##clear the new data


func _process(delta):
	while socket.get_available_packet_count() > 0:
		data = socket.get_packet().get_string_from_utf8()
		stored_value = data
		
		
func _callout():
	_process(self)
	
func generate_model(node, x, y, z, width, depth, height, name):
	for x_increment in width:
		for y_increment in height:
			for z_increment in depth:
				if x_increment == 0 or x_increment == (int(width)-1) or y_increment == 0 or y_increment == (int(height) - 1) or z_increment == 0 or z_increment == (int(depth) - 1):
					$GridMap2.set_cell_item(x_increment+int(x), y_increment+int(y), z_increment+int(z), 0)
					generate_textbox(node, x,height,z, name)

func generate_textbox(node, x,height,z, name):
	node.transform.origin = Vector3(int(x) + (width/1.5), int(int(y)+2 + (height)),z)
	node.get_node("Viewport/Label").set_text(str(name))
	node.get_node("Viewport").get_texture()

func adding_cortical_areas(name, x,y,z,width,height,depth):
	cortical_area[name]=[x,y,z,width,height,depth]
	
func install_voxel_inside(x,y,z):
	$GridMap.set_cell_item(x,y,z, 0)
