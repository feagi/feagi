extends ColorRect

func _ready():
	visible = false

func _on_mapping_rule_options_item_selected(index):
	var rule_name = $mapping_rule_options.get_item_text(index)
	if rule_name != " ":
		if "+" in rule_name:
			rule_name = rule_name.replace("+", "%2B")
		if "[" in rule_name:
			rule_name = rule_name.replace("[", "%5B")
		if "]" in rule_name:
			rule_name = rule_name.replace("]", "%5D")
		if ", " in rule_name:
			rule_name = rule_name.replace(", ", "%2C%20")
		var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + rule_name
		$get_morphology.request(combine_url)
		combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_usage?morphology_name=' + rule_name
		$get_morphology_usuage.request(combine_url)

func _process(_delta):
	if $mapping_rule_options.get_item_count() > 0:
		if $mapping_rule_options.get_item_text($mapping_rule_options.get_selected_id()) == " ":
			$rules.visible = false
		else:
			$rules.visible = true
