extends OptionButton

var timer = false
var waiting_in_the_line = []

func _ready():	
	Autoload_variable.addOption_Core = get_parent().get_parent().get_parent().get_node("Core")
	load_options()
	if timer:
		loading_in_two_seconds()
	
func load_options():
	if timer:
		waiting_in_the_line.append(1)
		loading_in_two_seconds()
	else:
		timer = true
		Autoload_variable.addOption_Core.Update_CorticalAreaNameList()

func _on_load_options_cortical_name_request_completed(_result, _response_code, _headers, body):
	clear()
	get_parent().get_node("source_dropdown").clear()
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
#	var api_data = json.result
	if api_data != null and not api_data.has("Request failed..."):
		add_item(" ")
		get_parent().get_node("source_dropdown").add_item(" ")
		for i in api_data:
			add_item(i)
			get_parent().get_node("source_dropdown").add_item(i)
			
	get_parent().get_parent().get_parent().get_node("Core/GlobalUISystem").get_node("Brain_Visualizer").get_node("notification").generate_notification_message(api_data, _response_code, "_on_cortical_type_options_request_request_completed", "/v1/feagi/genome/cortical_area_name_list")

func loading_in_two_seconds():
	await get_tree().create_timer(2).timeout
	load_options()
	timer=false

func _process(_delta):
	if timer == false:
		if len(waiting_in_the_line) != 0:
#			waiting_item = waiting_in_the_line[0]
			waiting_in_the_line.pop_at(0)
			Autoload_variable.addOption_Core.Update_CorticalAreaNameList()
	else:
		timer = false
