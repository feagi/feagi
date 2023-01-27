extends ColorRect

# Called when the node enters the scene tree for the first time.
func _ready():
	visible = false



func _on_mapping_rule_options_item_selected(index):
	var rule_name = $mapping_rule_options.get_item_text(index)
	if rule_name != " ":
		if "+" in rule_name:
			rule_name = rule_name.replace("+", "%2B")
		var combine_url = 'http://' + network_setting.api_ip_address + ':8000/v1/feagi/genome/morphology?morphology_name=' + rule_name
		$get_morphology.request(combine_url)
		combine_url = 'http://' + network_setting.api_ip_address + ':8000/v1/feagi/genome/morphology_usage?morphology_name=' + rule_name
		$get_morphology_usuage.request(combine_url)

func _process(_delta):
	if $mapping_rule_options.get_item_count() > 0:
		if $mapping_rule_options.get_item_text($mapping_rule_options.get_selected_id()) == " ":
			$rules.visible = false
		else:
			$rules.visible = true

func _on_get_morphology_usuage_request_completed(_result, _response_code, _headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	var new_name = ""
	for i in api_data:
		if new_name == "":
			new_name = str(i).replace(",", " > ") + "\n"
		else:
			new_name = new_name + str(i).replace(",", " > ") +"\n"
	new_name = new_name.replace("[", "")
	new_name = new_name.replace("]", "")
	$rules/morphology_definition/associations_data.text = new_name
