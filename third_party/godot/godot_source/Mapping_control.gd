extends Control


func _ready():
	load_options()
	visible = false

func load_options():
	$mapping_def.request('http://' + network_setting.api_ip_address + ':8000/v1/feagi/genome/morphology_list')


