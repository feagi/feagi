extends Control


func _ready():
	load_options()
	visible = false

func load_options():
	$mapping_def.request('http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology_list')


