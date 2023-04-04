extends OptionButton

var timer = false
var waiting_in_the_line = []

func _ready():
	load_options()
	if timer:
		loading_in_two_seconds()
	
func load_options():
	if timer:
		waiting_in_the_line.append(1)
		loading_in_two_seconds()
	else:
		timer = true
		$load_options_cortical_name.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area_name_list')

func _on_load_options_cortical_name_request_completed(_result, _response_code, _headers, body):
	clear()
	get_parent().get_parent().get_node("rule_properties/mapping_rule_options").clear()
	get_parent().get_node("source_dropdown").clear()
	var json = JSON.parse(body.get_string_from_utf8())
	var api_data = json.result
	if api_data != null:
		add_item(" ")
		get_parent().get_parent().get_node("rule_properties/mapping_rule_options").add_item(" ")
		get_parent().get_node("source_dropdown").add_item(" ")
		for i in api_data:
			add_item(i)
			get_parent().get_parent().get_node("rule_properties/mapping_rule_options").add_item(i)
			get_parent().get_node("source_dropdown").add_item(i)
			
	get_parent().get_parent().get_parent().get_parent().get_parent().get_node("notification").visible = true
	get_parent().get_parent().get_parent().get_parent().get_parent().get_node("notification/Label").text = str(_response_code)
	get_parent().get_parent().get_parent().get_parent().get_parent().get_node("notification/TextEdit").text = str(api_data)
	get_parent().get_parent().get_parent().get_parent().get_parent().get_node("notification/Label2").text = "_on_cortical_type_options_request_request_completed"
		
func loading_in_two_seconds():
	yield(get_tree().create_timer(2), "timeout")
	load_options()
	timer=false

func _process(_delta):
	if timer == false:
		if len(waiting_in_the_line) != 0:
#			waiting_item = waiting_in_the_line[0]
			waiting_in_the_line.remove(0)
			$load_options_cortical_name.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/cortical_area_name_list')
	else:
		timer = false
