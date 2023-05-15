extends ColorRect

func _ready():
	# Set visibility to false
	visible = false
	
	var test_json_conv = JSON.new()
	test_json_conv.parse(HelperFuncs.readTextFile("res://brain_visualizer_source/option.json"))
	var data = test_json_conv.get_data()
	
	# Create optionbutton
	for i in data["option"]:
		$OptionButton.add_item(i)
