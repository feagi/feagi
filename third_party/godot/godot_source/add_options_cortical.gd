extends OptionButton

var flag = false
var timer = false

func _ready():
	load_options()
	flag = true
	if timer:
		loading_in_two_seconds()
	
func load_options():
	$load_options_cortical_name.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area_name_list')
	
func _on_load_options_cortical_name_request_completed(_result, _response_code, _headers, body):
	clear()
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	add_item(" ")
	for i in api_data:
		add_item(i)
		
func loading_in_two_seconds():
	yield(get_tree().create_timer(2), "timeout")
	load_options()
	timer=false
