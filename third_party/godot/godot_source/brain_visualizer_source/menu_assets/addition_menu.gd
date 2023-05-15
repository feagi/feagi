extends ColorRect

var pointer = 0
var opu_list = []
var ipu_list = []
# Called when the node enters the scene tree for the first time.
func _ready():
	Autoload_variable.addition_Core = get_parent().get_parent().get_node("Core")
	add_list()
	visible = false
#	$cortical_area_type_label.visible = true


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(_delta):
	if $OptionButton.selected == 3:
		$wdh.visible = true
		$count.visible = false
		$cortical_name_textbox.visible = true
		$cortical_name_label.visible = false
		$xyz.visible = true
	elif $OptionButton.selected == 2:
		if pointer != $OptionButton.selected:
			pointer = $OptionButton.selected
			$cortical_name_label/type.clear()
			for i in ipu_list:
				$cortical_name_label/type.add_item(i)
		$wdh.visible = false
		$count.visible = true
		$cortical_name_textbox.visible = false
		$cortical_name_label.visible = true
		$xyz.visible = true
	elif $OptionButton.selected == 1:
		$wdh.visible = false
		$count.visible = true
		$cortical_name_textbox.visible = false
		$cortical_name_label.visible = true
		if pointer != $OptionButton.selected:
			pointer = $OptionButton.selected
			$cortical_name_label/type.clear()
			for i in opu_list:
				$cortical_name_label/type.add_item(i)
		$xyz.visible = true
	elif $OptionButton.selected == 0:
		$wdh.visible = false
		$count.visible = false
		$cortical_name_textbox.visible = false
		$xyz.visible = false
		$cortical_name_label.visible = false
	if visible == false:
		$OptionButton.select(0)
	if $cortical_name_textbox/type.text != "" and $OptionButton.selected == 3:
		$add.disabled = false
	elif $OptionButton.selected != 3 and $cortical_name_label/type.selected != 0:
		$add.disabled = false
	else:
		$add.disabled = true


func add_list():
	Autoload_variable.addition_Core.GET_OPU('OPU')

func _on_cortical_type_options_request_request_completed(_result, _response_code, _headers, body):
	opu_list = []
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
#	var api_data = json.result
	if api_data != null:
		opu_list.append(" ")
		for i in api_data:
			opu_list.append(i)
	Autoload_variable.addition_Core.GET_IPU('IPU')
	get_parent().get_parent().get_node("Core/GlobalUISystem").get_node("Brain_Visualizer").get_node("notification").generate_notification_message(api_data, _response_code, "_on_cortical_type_options_request_request_completed", "/v1/feagi/genome/cortical_type_options")

func _on_IPU_list_request_completed(_result, _response_code, _headers, body):
	ipu_list = []
	var test_json_conv = JSON.new()
	test_json_conv.parse(body.get_string_from_utf8())
	var api_data = test_json_conv.get_data()
#	var api_data = json.result
	ipu_list.append(" ")
	if api_data != null:
		for i in api_data:
			ipu_list.append(i)
	get_parent().get_parent().get_node("Core/GlobalUISystem").get_node("Brain_Visualizer").get_node("notification").generate_notification_message(api_data, _response_code, "_on_cortical_type_options_request_request_completed", "/v1/feagi/genome/cortical_type_options?cortical_type=IPU")
