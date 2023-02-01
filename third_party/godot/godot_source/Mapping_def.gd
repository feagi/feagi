extends OptionButton


func _ready():
	load_options()

func load_options():
	$mapping_def.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_list')

func _on_mapping_def_request_completed(result, response_code, headers, body):
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	add_item(" ")
	for i in api_data:
		add_item(i)
