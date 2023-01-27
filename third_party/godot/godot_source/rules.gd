extends ColorRect

var flag = false

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.
	
func _process(_delta):
	if $morphology_definition/associations_data.text != "":
		$delete.disabled = true
	else:
		$delete.disabled = false


func _on_get_morphology_request_completed(_result, _response_code, _headers, body):
	flag = false
	var json = JSON.parse(body.get_string_from_utf8()) # Every http data, it's done in poolbytearray
	var api_data = json.result
	var new_name = ""
	for i in api_data:
		new_name = str(api_data[i])
		for x in $rule_type_options.get_item_count():
			if $rule_type_options.get_item_text(x) == i:
				$rule_type_options.selected = x
				if "*" in new_name:
					new_name = new_name.replace("*", "\""+"*"+"\"")
					$morphology_definition/morphology_def.text = new_name
					flag = true
				if "?" in new_name:
					flag = true
					new_name = new_name.replace("?", "\""+"?"+"\"")
					$morphology_definition/morphology_def.text = new_name
				if flag == false:
					$morphology_definition/morphology_def.text = new_name
