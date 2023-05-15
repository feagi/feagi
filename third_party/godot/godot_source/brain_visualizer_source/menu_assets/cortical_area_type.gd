extends OptionButton

func _ready():
	var filess = FileAccess.open("res://brain_visualizer_source/type_option.json", FileAccess.READ)
	var test_json_conv = JSON.new()
	test_json_conv.parse(filess.get_as_text())
	var data = test_json_conv.get_data()
	filess.close()
	
	# Create optionbutton
	for i in data["option"]:
		add_item(i)
