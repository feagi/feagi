extends ColorRect

var flag = false
var current = 0
var setpoint = false
var projectResolution: Vector2
var pulse_transparent = 0.3
var default_color: Color
var dictionary_manager = {}
var previous_dict_total = 0
var default_y = 0
var api_message


func _ready():
	var filess = FileAccess.open("res://brain_visualizer_source/api_messages.json", FileAccess.READ)
	var test_json_conv = JSON.new()
	test_json_conv.parse(filess.get_as_text())
	api_message = test_json_conv.get_data()
	filess.close()
	visible = false
	default_color = color
	default_y = position.y
#	projectResolution=Vector2(ProjectSettings.get_setting("display/window/size/viewport_width"), ProjectSettings.get_setting("display/window/size/viewport_width"))

func _process(_delta):
	for i in dictionary_manager:
		if i.visible:
			if flag == false:
				if get_child(1).text != "0":
					flag = true
	#				timer()
				else:
					get_child(0).text = "FEAGI IS NOT STARTED NOR LOADED!!"
					i.color = Color(0.545098, 0, 0, pulse_transparent)
			if i.get_child(1).text == "Operation Succeeded" or i.get_child(1).text == "200":
				i.get_child(1).text = "Operation Succeeded"
				i.get_child(1).add_theme_color_override("font_color", "#1f9239")
				i.color = default_color
			elif i.get_child(1).text == "422":
				i.get_child(1).add_theme_color_override("font_color", "#ff0000")
				i.get_child(0).text = "This cortical area is not exist in FEAGI yet! Try again in 10 seconds"
				i.color = Color(0.545098, 0, 0, pulse_transparent)
			else:
				i.get_child(1).add_theme_color_override("font_color", "#ff0000")
				i.color = Color(0.545098, 0, 0, pulse_transparent)
		if previous_dict_total != len(dictionary_manager):
			previous_dict_total = len(dictionary_manager)
			var increment_number = 0
			for a in dictionary_manager:
				a.position.y = default_y + (increment_number * 130)
				increment_number += 1


func timer(node):
	await get_tree().create_timer(5).timeout
	node.visible = false
	node.queue_free()
	dictionary_manager.erase(node)

func generate_notification_message(_api_data, API_response, func_name, feagi_url, type="GET"):
	API_response = str(API_response)
	if API_response != "0":
		if api_message.has(feagi_url):
			if api_message[feagi_url].has(type):
				if api_message[feagi_url][type].has(API_response):
					var counter = len(dictionary_manager)
					var node_duplicate_name = duplicate()
					get_parent().add_child(node_duplicate_name)
					dictionary_manager[node_duplicate_name] = func_name
					node_duplicate_name.position.y += counter * 130
					node_duplicate_name.visible = true
					if str(API_response) == "200":
						node_duplicate_name.get_child(1).text = "Operation Succeeded"
						node_duplicate_name.get_child(1).add_theme_color_override("font_color", "#1f9239")
					elif str(API_response) == "400":
						node_duplicate_name.get_child(1).text = "Operation Failed"
						node_duplicate_name.get_child(1).add_theme_color_override("font_color", "#ff0000")
					else:
						node_duplicate_name.get_child(1).text = str(API_response)
					node_duplicate_name.get_child(0).text = str(api_message[feagi_url][type][API_response]["EN"])
					node_duplicate_name.get_child(2).text = str(func_name)
					timer(node_duplicate_name)
	else:
		var counter = len(dictionary_manager)
		var node_duplicate_name = duplicate()
		get_parent().add_child(node_duplicate_name)
		dictionary_manager[node_duplicate_name] = func_name
		node_duplicate_name.position.y += counter * 130
		node_duplicate_name.visible = true
		if str(API_response) == "0":
			node_duplicate_name.get_child(1).text = "Operation Failed"
			node_duplicate_name.get_child(1).add_theme_color_override("font_color", "#ff0000")
		else:
			node_duplicate_name.get_child(1).text = str(API_response)
		node_duplicate_name.get_child(0).text = str("FEAGI is not connected currently")
		node_duplicate_name.get_child(2).text = str(func_name)
		timer(node_duplicate_name)
