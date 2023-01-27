extends OptionButton

func _ready():
		# # # DELETE WHEN DONE
	var path = "res://type_option.json"
	var filess = File.new()
	
	filess.open(path, filess.READ)
	var data = parse_json(filess.get_as_text())
	filess.close()
	
	# Create optionbutton
	for i in data["option"]:
		add_item(i)
