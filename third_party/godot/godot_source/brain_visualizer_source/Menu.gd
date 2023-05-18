extends Control

var pressed = false
var flag = false
var visible_array = [] # Store 3 sections for scalable 
var icon_pressed = preload("res://brain_visualizer_source/menu_assets/image/collapse_selected.png")
var icon_not_pressed = preload("res://brain_visualizer_source/menu_assets/image/collapse_not_selected.png")
var one_time_flag = true
var json_menu

func _ready():
	Autoload_variable.Menu_Core = $".."/Core
	visible = true
	var test_json_conv = JSON.new()
	test_json_conv.parse(HelperFuncs.ReadTextFile("res://JSON_library/2D_top_menu.json"))
	json_menu = test_json_conv.get_data()

func check_esc():
	if Input.is_action_just_pressed("esc"):
		if visible:
			visible = false
		else:
			visible = true

func _process(_delta):
	if Godot_list.Node_2D_control:
		$"3D_enable".visible = true
	else:
		$"3D_enable".visible = false 
	check_esc()
	visibility_Mapping_Properties()
	if $Mapping_Properties/cortical_dropdown.get_item_count() != $information_menu/cortical_cam_label/menu_itemlist.get_item_count() or $Mapping_Properties/cortical_dropdown.get_item_count() != $Mapping_Properties/source_dropdown.get_item_count():
		$information_menu/cortical_cam_label/menu_itemlist.clear()
		$Mapping_Properties/source_dropdown.clear()
		for i in $Mapping_Properties/cortical_dropdown.get_item_count():
			$Mapping_Properties/source_dropdown.add_item($Mapping_Properties/cortical_dropdown.get_item_text(i))
			$information_menu/cortical_cam_label/menu_itemlist.add_item($Mapping_Properties/cortical_dropdown.get_item_text(i))
	if flag:
		flag = false
		resize_buttons()
#	print("status: ", $close_for_all.visible)

func _on_add_pressed():
	if $addition_menu.visible:
		$addition_menu.visible = false
	elif $addition_menu.visible == false:
		$addition_menu.visible = true
	$add.release_focus()

func _on_close_pressed():
	$"Mapping_Properties".visible = false

func _on_rule_button_close_pressed():
	$rule_properties.visible = false

func _on_info_map_def_pressed():
	$rule_properties.visible = true
#	$rule_properties/mapping_rule_options.select($"Mapping_Properties/inside_mapping_menu/Control/Mapping_def".get_selected_id())
	$rule_properties/mapping_rule_options.disabled = true
	var rule_name = $rule_properties/mapping_rule_options.get_item_text($rule_properties/mapping_rule_options.get_selected_id())
	if rule_name != " ":
		if "+" in rule_name:
			rule_name = rule_name.replace("+", "%2B")
		Autoload_variable.Menu_Core.Get_Morphology_information(rule_name)

func visibility_Mapping_Properties():
	if not($cortical_menu.visible):
		resize_buttons()
	if $cortical_menu.visible:
		resize_buttons()
		$insert_menu.visible = false
	if not($cortical_mapping/Control.visible):
		$Mapping_Properties.visible = false

func resize_buttons():
#	var screen_size = get_window().get_size()
#	$insert_menu.size = Vector2(screen_size[0]/6, screen_size[1]-150)
	$insert_menu/circuit_information_label.position.y = $insert_menu/insert_button.size.y + $insert_menu/insert_button.position.y + 10
	$insert_menu/inner_box.position.y = $insert_menu/circuit_information_label.position.y + $insert_menu/circuit_information_label.size.y + 10
	
	# Close button resize section
	if $insert_menu.visible:
		$close_for_all.visible = true
		$close_for_all.position.x = $insert_menu.size.x
		$close_for_all.position.y = $insert_menu.size.y-($insert_menu.size.y-65)
		$close_for_all.size.x = 33
		$close_for_all.size.y = 23
	if $cortical_menu.visible or $properties.visible:
		$close_for_all.visible = true
		$close_for_all.position.x = $cortical_menu/Control/name_string.size.x + $cortical_menu/Control/name_string.position.x + 60
		$close_for_all.position.y = $information_menu.position.y + $information_menu.size.y
		$close_for_all.size.x = 25
		$close_for_all.size.y = 23
	if ($cortical_menu.visible == false and $insert_menu.visible == false and $collapse_4.visible == false):
		$close_for_all.visible = false
	if $collapse_4.visible:
		$close_for_all.visible = true

	# TOP menu
	$information_menu.position = Vector2(json_menu["top_menu"]["top_background_menu"]["position"][0], json_menu["top_menu"]["top_background_menu"]["position"][1])
	$information_menu.size = Vector2(json_menu["top_menu"]["top_background_menu"]["size"][0], json_menu["top_menu"]["top_background_menu"]["size"][1])
	$information_menu/genome_label.position = Vector2(json_menu["top_menu"]["genome_label"]["position"][0], json_menu["top_menu"]["genome_label"]["position"][1])
	$information_menu/genome_string.size = Vector2(json_menu["top_menu"]["hz"]["size"][0], json_menu["top_menu"]["hz"]["size"][1])
	$information_menu/genome_string.position = Vector2(json_menu["top_menu"]["hz"]["position"][0], json_menu["top_menu"]["hz"]["position"][1])
	$import.position = Vector2(json_menu["top_menu"]["import_button"]["position"][0], json_menu["top_menu"]["import_button"]["position"][1])
	$import.size = Vector2(json_menu["top_menu"]["import_button"]["size"][0], json_menu["top_menu"]["import_button"]["size"][1])
	$information_menu/burst_duration_label.position = Vector2(json_menu["top_menu"]["system_refresh_label"]["position"][0], json_menu["top_menu"]["system_refresh_label"]["position"][1])
	$brown_bar.size = Vector2(json_menu["top_menu"]["brown_bar"]["size"][0], json_menu["top_menu"]["brown_bar"]["size"][1])
	$brown_bar.position = Vector2(json_menu["top_menu"]["brown_bar"]["position"][0], json_menu["top_menu"]["brown_bar"]["position"][1])
	$information_menu/cortical_cam_label.position = Vector2(json_menu["top_menu"]["cortical_cam_label"]["position"][0], json_menu["top_menu"]["cortical_cam_label"]["position"][1])
	$add.size = Vector2(json_menu["top_menu"]["add_cortical_button"]["size"][0],json_menu["top_menu"]["add_cortical_button"]["size"][1]) 
	$add.position = Vector2(json_menu["top_menu"]["add_cortical_button"]["position"][0] + ($information_menu/cortical_cam_label/menu.size.x - 31),json_menu["top_menu"]["add_cortical_button"]["position"][1])
	$brown_bar2.size = Vector2(json_menu["top_menu"]["second_brown_bar"]["size"][0],json_menu["top_menu"]["second_brown_bar"]["size"][1])
	$brown_bar2.position = Vector2(json_menu["top_menu"]["second_brown_bar"]["position"][0] + ($add.position.x - 22.281),json_menu["top_menu"]["second_brown_bar"]["position"][1]) 
	$information_menu/Label.position = Vector2(json_menu["top_menu"]["neuron_morphology_label"]["position"][0] + ($brown_bar2.position.x - 694), json_menu["top_menu"]["neuron_morphology_label"]["position"][1])
	$information_menu/Neuron_morphologies_button.position.x = $information_menu/Label.position.x + $information_menu/Label.size.x - 20
	$information_menu/Neuron_morphologies_item.position.x = $information_menu/Label.position.x + $information_menu/Label.size.x - 20
	$information_menu/Neuron_morphologies_item.position.y = $information_menu/Neuron_morphologies_button.position.y + $information_menu/Neuron_morphologies_button.size.y
	$information_menu/add_inside_neuron_morph.position = Vector2(json_menu["top_menu"]["add_morphology_button"]["position"][0] + ($information_menu/Label.position.x - 34.862), json_menu["top_menu"]["add_morphology_button"]["position"][1])
	$information_menu/add_inside_neuron_morph.size = Vector2(json_menu["top_menu"]["add_morphology_button"]["size"][0], json_menu["top_menu"]["add_morphology_button"]["size"][1])
	$".."/Button.position.x = $information_menu.size.x 
#	$information_menu.size.x = $information_menu/add_inside_neuron_morph.position.x + $information_menu/add_inside_neuron_morph.size.x + 40

	# Cortical menu section
	$cortical_menu.position.y = $information_menu.size.y + $information_menu.position.y
	$cortical_menu/title.position.y = $information_menu.position.y 
	$cortical_menu/Label.position.y = 30
	$cortical_menu/Control/name.position.y = $cortical_menu/Label.position.y + $cortical_menu/Label.size.y
	$cortical_menu/Control/name_string.position.x = $cortical_menu/Control/name.size.x
	$cortical_menu/Control/name_string.position.y = $cortical_menu/Label.position.y + $cortical_menu/Label.size.y
	$cortical_menu/Control/name2.position.y = $cortical_menu/Control/name.position.y + $cortical_menu/Control/name.size.y + 20
	$cortical_menu/Control/cortical_id.position.x = $cortical_menu/Control/name2.size.x
	$cortical_menu/Control/cortical_id.position.y = $cortical_menu/Control/name_string.size.y + $cortical_menu/Control/name_string.position.y + 5
	$cortical_menu/Control/Properties.position.y = $cortical_menu/Control/name2.size.y + $cortical_menu/Control/name2.position.y + 15
	$cortical_menu/Control/OptionButton.position.y =  $cortical_menu/Control/Properties.position.y + 5
	$cortical_menu/Control/x_string.position.y = $cortical_menu/Control/OptionButton.size.y + $cortical_menu/Control/OptionButton.position.y + 5
	$cortical_menu/Control/y_string.position.y = $cortical_menu/Control/OptionButton.size.y + $cortical_menu/Control/OptionButton.position.y + 5
	$cortical_menu/Control/z_string.position.y = $cortical_menu/Control/OptionButton.size.y + $cortical_menu/Control/OptionButton.position.y + 5
	$cortical_menu/Control/X.position.y = $cortical_menu/Control/x_string.position.y + $cortical_menu/Control/x_string.size.y + 5
	$cortical_menu/Control/Y.position.y = $cortical_menu/Control/y_string.size.y + $cortical_menu/Control/y_string.position.y + 5
	$cortical_menu/Control/Z.position.y = $cortical_menu/Control/z_string.size.y + $cortical_menu/Control/z_string.position.y + 5

	# W D H section
	$cortical_menu/Control/D_string.position.y = $cortical_menu/Control/Z.position.y + $cortical_menu/Control/Z.size.y + 5
	$cortical_menu/Control/H_string.position.y = $cortical_menu/Control/Y.position.y + $cortical_menu/Control/Y.size.y + 5
	$cortical_menu/Control/W_string.position.y = $cortical_menu/Control/X.position.y + $cortical_menu/Control/X.size.y + 5
	$cortical_menu/Control/W.position.y = $cortical_menu/Control/W_string.size.y + $cortical_menu/Control/W_string.position.y + 5
	$cortical_menu/Control/D.position.y = $cortical_menu/Control/D_string.size.y + $cortical_menu/Control/D_string.position.y + 5
	$cortical_menu/Control/H.position.y = $cortical_menu/Control/H_string.size.y + $cortical_menu/Control/H_string.position.y + 5

	# Delete/Update section
	$cortical_menu/Control/Update.position.y = $cortical_menu/Control/H.position.y + $cortical_menu/Control/H.size.y + 10
	$cortical_menu/Control/remove.position.y = $cortical_menu/Control/H.position.y + $cortical_menu/Control/H.size.y + 10
	if $cortical_menu/Control.visible:
		$cortical_menu.size.y = $cortical_menu/Control/Update.size.y + $cortical_menu/Control/Update.position.y + 10

	# Properties position
	if $button_choice/Control.visible:
		$properties.position.y = $button_choice.size.y + $button_choice.position.y
	else:
		if $properties.position.y >= $button_choice/Label.position.y + $button_choice/Label.size.y + $button_choice.position.y:
			$properties.position.y -= 20
	$properties.size.x = $button_choice.size.x
	$properties/Control/neuron_count_.position.y = $properties/Label.position.y + $properties/Label.size.y + 5
	$properties/Control/neuron_count.position.y = $properties/Label.position.y + $properties/Label.size.y + 5
	$properties/Control/syn.position.y = $properties/Control/neuron_count_.position.y + $properties/Control/neuron_count_.size.y + 15
	$properties/Control/synaptic.position.y = $properties/Control/neuron_count_.position.y + $properties/Control/neuron_count_.size.y + 15
	$properties/Control/post_syn.position.y = $properties/Control/synaptic.position.y + $properties/Control/synaptic.size.y + 15
	$properties/Control/pst_syn.position.y = $properties/Control/synaptic.position.y + $properties/Control/synaptic.size.y + 15
	$properties/Control/post_syn_max.position.y = $properties/Control/post_syn.position.y + $properties/Control/post_syn.size.y + 15
	$properties/Control/pst_syn_max.position.y = $properties/Control/post_syn.position.y + $properties/Control/post_syn.size.y + 15
	$properties/Control/plasticity.position.y = $properties/Control/post_syn_max.position.y + $properties/Control/post_syn_max.size.y + 15
	$properties/Control/plst.position.y = $properties/Control/post_syn_max.position.y + $properties/Control/post_syn_max.size.y + 15
	$properties/Control/fire_count.position.y = $properties/Control/plasticity.position.y + $properties/Control/plasticity.size.y + 15
	$properties/Control/fire.position.y = $properties/Control/plasticity.position.y + $properties/Control/plasticity.size.y + 15
	$properties/Control/Threshold_Sensitivity_Label.position.y = $properties/Control/fire_count.position.y + $properties/Control/fire_count.size.y
	$properties/Control/Threshold_Sensitivity_text.position.y = $properties/Control/fire_count.position.y + $properties/Control/fire_count.size.y 
	$properties/Control/refractory.position.y = $properties/Control/Threshold_Sensitivity_text.position.y + $properties/Control/Threshold_Sensitivity_text.size.y
	$properties/Control/refa.position.y = $properties/Control/Threshold_Sensitivity_text.position.y + $properties/Control/Threshold_Sensitivity_text.size.y 
	$properties/Control/leak.position.y = $properties/Control/refractory.position.y + $properties/Control/refractory.size.y
	$properties/Control/leakco.position.y = $properties/Control/refractory.position.y + $properties/Control/refractory.size.y
	$properties/Control/leak_va.position.y = $properties/Control/leak.position.y + $properties/Control/leak.size.y
	$properties/Control/leak_Vtext.position.y = $properties/Control/leak.position.y + $properties/Control/leak.size.y
	$properties/Control/fireshold_increment_label.position.y = $properties/Control/leak_va.position.y + $properties/Control/leak_va.size.y
	$properties/Control/fireshold_increment.position.y = $properties/Control/leak_va.position.y + $properties/Control/leak_va.size.y
	$properties/Control/consecutive.position.y = $properties/Control/fireshold_increment_label.position.y + $properties/Control/fireshold_increment_label.size.y
	$properties/Control/cfr.position.y = $properties/Control/fireshold_increment.position.y + $properties/Control/fireshold_increment.size.y
	$properties/Control/snze.position.y = $properties/Control/consecutive.position.y + $properties/Control/consecutive.size.y
	$properties/Control/snooze.position.y = $properties/Control/consecutive.position.y + $properties/Control/consecutive.size.y
	$properties/Control/dege.position.y = $properties/Control/snooze.position.y + $properties/Control/snooze.size.y
	$properties/Control/degeneracy.position.y = $properties/Control/snooze.position.y + $properties/Control/snooze.size.y
	$properties/Control/psud.position.y = $properties/Control/degeneracy.position.y + $properties/Control/degeneracy.size.y 
	$properties/Control/psp.position.y = $properties/Control/degeneracy.position.y + $properties/Control/degeneracy.size.y
	$properties/Control/MP_Label.position.y = $properties/Control/psp.position.y + $properties/Control/psp.size.y
	$properties/Control/MP.position.y = $properties/Control/MP_Label.position.y
	$properties/Control/Update.position.y = $properties/Control/MP_Label.position.y + $properties/Control/MP_Label.size.y
	if $properties/Control.visible:
		$properties.size.y =  $properties/Control/Update.size.y + $properties/Control/Update.position.y
	# Delete and Update section
	$collapse_1.position.y = $properties/Control/psp.position.y + $properties/Control/psp.size.y + 260

	# Button choice section
	if $cortical_menu/Control.visible:
		$button_choice.position.y = $cortical_menu.size.y + $cortical_menu.position.y - 5
		$button_choice.size.x = $cortical_menu.size.x
		$button_choice/Label.position.y = 0
		$button_choice/Control/Label.position.y = $button_choice/Label.position.y + $button_choice/Label.size.y
		$button_choice/Control/Label2.position.y = $button_choice/Label.position.y + $button_choice/Label.size.y
		$button_choice/Control/mem.position.y = $button_choice/Control/Label.position.y + $button_choice/Control/Label.size.y
		$button_choice/Control/syn.position.y = $button_choice/Control/Label2.position.y + $button_choice/Control/Label2.size.y
		$button_choice.size.y = $button_choice/Control/syn.position.y + $button_choice/Control/syn.size.y
	else:
		if $button_choice.position.y > $collapse_4.position.y + $cortical_menu/title.position.y + $cortical_menu/title.size.y + 20:
			$button_choice.position.y -= 20
		$button_choice.size.x = $cortical_menu.size.x
		$button_choice/Label.position.y = -5
		$button_choice/Control/mem.position.y = $button_choice/Label.position.y + $button_choice/Label.size.y + 30
		$button_choice/Control/syn.position.y = $button_choice/Label.position.y + $button_choice/Label.size.y + 30
	# Cortical mappoing section
	if $properties/Control.visible == false:
		if $cortical_mapping.position.y > $properties.position.y + 45:
			$cortical_mapping.position.y -= 20
			$properties.size.y = $cortical_menu/title.size.y + $cortical_menu/title.position.y
		$cortical_mapping.size.x = $properties.size.x
		$cortical_mapping/Control/afferent_label.position.y = $cortical_mapping/Label.position.y + $cortical_mapping/Label.size.y
		$cortical_mapping/Control/afferent.position.y = $cortical_mapping/Control/afferent_label.position.y + $cortical_mapping/Control/afferent_label.size.y + 5
		$cortical_mapping/Control/efferent_label.position.y = $cortical_mapping/Control/afferent.position.y + $cortical_mapping/Control/afferent.size.y + 10
		$cortical_mapping/Control/cortical_mapping_add.position.y = $cortical_mapping/Control/afferent.position.y + $cortical_mapping/Control/afferent.size.y
		$cortical_mapping/Control/ScrollContainer.position.y = $cortical_mapping/Control/cortical_mapping_add.position.y + $cortical_mapping/Control/cortical_mapping_add.size.y + 10
	else:
		$cortical_mapping.position.y = $properties.size.y + $properties.position.y
		$cortical_mapping.size.x = $properties.size.x
		$cortical_mapping/Control/afferent_label.position.y = $cortical_mapping/Label.position.y + $cortical_mapping/Label.size.y
		$cortical_mapping/Control/afferent.position.y = $cortical_mapping/Control/afferent_label.position.y + $cortical_mapping/Control/afferent_label.size.y + 5
		$cortical_mapping/Control/efferent_label.position.y = $cortical_mapping/Control/afferent.position.y + $cortical_mapping/Control/afferent.size.y
		$cortical_mapping/Control/cortical_mapping_add.position.y = $cortical_mapping/Control/afferent.position.y + $cortical_mapping/Control/afferent.size.y
		$cortical_mapping/Control/ScrollContainer.position.y = $cortical_mapping/Control/cortical_mapping_add.position.y + $cortical_mapping/Control/cortical_mapping_add.size.y + 10

	# collapse section
	$collapse_1.position.y = $properties.position.y + 5
	$collapse_1.position.x = $properties.size.x + $properties.position.x - 30
	$collapse_2.position.x = $button_choice/Label.size.x + $button_choice/Label.position.x - 30
	$collapse_2.position.y = $button_choice.position.y + $button_choice/Label.position.y
	$collapse_3.position.y = $cortical_mapping.position.y
	$collapse_3.position.x = $cortical_mapping.position.x + $cortical_mapping.size.x - 30
	$collapse_4.position.x = $close_for_all.position.x
	$collapse_4.position.y = $close_for_all.size.y + $close_for_all.position.y - 5
	$collapse_4.size = Vector2(25,25)
	
	$"3D_enable".position.x = $properties.size.x
	$"3D_enable".size.x = get_window().get_size().x
	$"3D_enable".size.y = get_window().get_size().y
	$box_loading.position.x = get_window().get_size().x- $box_loading.size.x
	$box_loading.position.y = get_window().get_size().y/2 - $box_loading.size.y
	
func _on_Menu_resized():
	flag = true

func _on_import_pressed():
	$import.release_focus()
	$cortical_menu.visible = false
	$properties.visible= false
	$cortical_mapping.visible= false
	$button_choice.visible= false
	$collapse_1.visible= false
	$collapse_2.visible= false
	$collapse_3.visible= false
	$insert_menu.visible = true
	$cortical_menu/title.visible = false
#	resize_buttons()

func _on_close_for_all_pressed():
	$cortical_menu.visible = false
	$properties.visible= false
	$cortical_mapping.visible= false
	$button_choice.visible= false
	$collapse_1.visible= false
	$collapse_2.visible= false
	$collapse_3.visible= false
	$collapse_4.visible= false
	$cortical_menu/title.visible = false
	$insert_menu.visible = false

#func _on_collapse_1_pressed():
#	if $properties/Control.visible:
#		$properties/Control.visible = false
#		$collapse_1.icon = icon_pressed
#	else:
#		$properties/Control.visible = true
#		$collapse_1.icon = icon_not_pressed
#	$collapse_1.release_focus()


#func _on_collapse_2_pressed():
#	if $button_choice/Control.visible:
#		$button_choice/Control.visible = false
#		$button_choice.size.y = $button_choice/Control.size.y + $button_choice/Control.position.y
#		$collapse_2.icon = icon_pressed
#	else:
#		$button_choice/Control.visible = true
#		$collapse_2.icon = icon_not_pressed
#		$button_choice.size.y = $button_choice/Control/syn.size.y + $button_choice/Control/syn.position.y + 15
#	$collapse_2.release_focus()
#
#func _on_collapse_3_pressed():
#	if $cortical_mapping/Control.visible:
#		$cortical_mapping/Control.visible = false
#		$collapse_3.icon = icon_pressed
#		$cortical_mapping.size.y = $cortical_mapping/Label.position.y + $cortical_mapping/Label.size.y
#	else:
#		$cortical_mapping/Control.visible = true
#		$collapse_3.icon = icon_not_pressed
#		$cortical_mapping.size.y = $cortical_mapping/Control/ScrollContainer.size.y + $cortical_mapping/Control/ScrollContainer.position.y
#	$collapse_3.release_focus()
#
func _on_collapse_4_pressed():
	if $cortical_menu/Control.visible:
		$cortical_menu/Control.visible = false
		$collapse_4.icon = icon_pressed
		$cortical_menu.size.y = $cortical_menu/Label.size.y + $cortical_menu/Label.position.y
	else:
		$cortical_menu/Control.visible = true
		$collapse_4.icon = icon_not_pressed
	$collapse_4.release_focus()
