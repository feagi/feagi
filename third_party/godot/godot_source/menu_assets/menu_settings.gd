extends ColorRect

func _ready():
		# # # DELETE WHEN DONE
	var path = "res://option.json"
	var filess = File.new()
	
	# Set visibility to false
	visible = false
	
	filess.open(path, filess.READ)
	var data = parse_json(filess.get_as_text())
	filess.close()
	
	# Create optionbutton
	for i in data["option"]:
		$OptionButton.add_item(i)
