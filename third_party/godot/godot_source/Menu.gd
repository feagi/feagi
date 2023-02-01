extends Control

var pressed = false
var flag = false
var visible_array = [] # Store 3 sections for scalable 
var icon_pressed = preload("res://menu_assets/image/collapse_selected.png")
var icon_not_pressed = preload("res://menu_assets/image/collapse_not_selected.png")
var one_time_flag = true

func _ready():
	visible = true

func check_esc():
	if Input.is_action_just_pressed("esc"):
		if visible:
			visible = false
		else:
			visible = true

func _process(_delta):
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
		var combine_url = 'http://' + network_setting.api_ip_address + ':' + network_setting.api_port_address + '/v1/feagi/genome/morphology?morphology_name=' + rule_name
		$rule_properties/get_morphology.request(combine_url)
	
func visibility_Mapping_Properties():
	if not($cortical_menu.visible):
		resize_buttons()
	if $cortical_menu.visible:
		resize_buttons()
		$insert_menu.visible = false
	if not($cortical_mapping/Control.visible):
		$Mapping_Properties.visible = false

func resize_buttons():
	var screen_size = OS.get_window_size()
	$insert_menu.rect_size = Vector2(screen_size[0]/6, screen_size[1]-150)
	$insert_menu/circuit_information_label.rect_position.y = $insert_menu/insert_button.rect_size.y + $insert_menu/insert_button.rect_position.y + 10
	$insert_menu/inner_box.rect_position.y = $insert_menu/circuit_information_label.rect_position.y + $insert_menu/circuit_information_label.rect_size.y + 10
	
	# Close button resize section
	if $insert_menu.visible:
		$close_for_all.visible = true
		$close_for_all.rect_position.x = $insert_menu.rect_size.x - 30
		$close_for_all.rect_position.y = $insert_menu.rect_size.y-($insert_menu.rect_size.y-65)
		$close_for_all.rect_size.x = 33
		$close_for_all.rect_size.y = 23
	if $cortical_menu.visible or $properties.visible:
		$close_for_all.visible = true
		$close_for_all.rect_position.x = $cortical_menu/Control/name_string.rect_size.x + $cortical_menu/Control/name_string.rect_position.x -1
		$close_for_all.rect_position.y = $information_menu.rect_position.y + $information_menu.rect_size.y
		$close_for_all.rect_size.x = 33
		$close_for_all.rect_size.y = 23
	if ($cortical_menu.visible == false and $insert_menu.visible == false and $collapse_4.visible == false):
		$close_for_all.visible = false
	if $collapse_4.visible:
		$close_for_all.visible = true

	# TOP menu
	$information_menu.rect_position.x = 2
	$information_menu.rect_size.y = 35
	$information_menu/genome_label.rect_position = Vector2(0,0)
	$information_menu/genome_string.rect_size.x = 150
	$information_menu/genome_string.rect_position.x = $information_menu/genome_label.rect_position.x + $information_menu/genome_label.rect_size.x - 20
	$information_menu/genome_string.rect_size.y = $information_menu.rect_size.y/2
	$import.rect_position.x = $information_menu/genome_string.rect_size.x + $information_menu/genome_string.rect_position.x
	$import.rect_size.y = $information_menu.rect_size.y
	$import.rect_size.x = $import.rect_size.y
	$information_menu/burst_duration_label.rect_position.x = $import.rect_size.x + $import.rect_position.x 
	$brown_bar.rect_size.y = $information_menu.rect_size.y/1.7
	$brown_bar.rect_position.x = $information_menu/burst_duration_label/burst_value.rect_position.x + $information_menu/burst_duration_label/burst_value.rect_size.x + $information_menu/burst_duration_label.rect_position.x + $information_menu/burst_duration_label.rect_size.x - 60
	$information_menu/cortical_cam_label.rect_position.x = $brown_bar.rect_position.x + $brown_bar.rect_size.x + 10
	$add.rect_size.y = $import.rect_size.y 
	$add.rect_size.x = $import.rect_size.x
	$add.rect_position.x = $information_menu/cortical_cam_label/menu.rect_position.x +$information_menu/cortical_cam_label/menu.rect_size.x + $information_menu/cortical_cam_label.rect_position.x
	$brown_bar2.rect_size.y = $information_menu.rect_size.y/1.7
	$brown_bar2.rect_position.x = $add.rect_position.x + $add.rect_size.x 
	$information_menu/Label.rect_position.x = $brown_bar2.rect_position.x + $brown_bar2.rect_size.x - 20
	$information_menu/Neuron_morphologies_button.rect_position.x = $information_menu/Label.rect_position.x + $information_menu/Label.rect_size.x - 20
	$information_menu/Neuron_morphologies_item.rect_position.x = $information_menu/Label.rect_position.x + $information_menu/Label.rect_size.x - 20
	$information_menu/Neuron_morphologies_item.rect_position.y = $information_menu/Neuron_morphologies_button.rect_position.y + $information_menu/Neuron_morphologies_button.rect_size.y
	$information_menu/add_inside_neuron_morph.rect_position.x = $information_menu/Neuron_morphologies_button.rect_position.x + $information_menu/Neuron_morphologies_button.rect_size.x
	$information_menu/add_inside_neuron_morph.rect_size.x = $import.rect_size.x
	$information_menu/add_inside_neuron_morph.rect_size.y = $import.rect_size.y
#	$information_menu.rect_size.x = $information_menu/add_inside_neuron_morph.rect_position.x + $information_menu/add_inside_neuron_morph.rect_size.x + 40

	# Cortical menu section
	$cortical_menu.rect_position.y = $information_menu.rect_size.y + $information_menu.rect_position.y
	$cortical_menu/title.rect_position.y = $information_menu.rect_position.y 
	$cortical_menu/Label.rect_position.y = 30
	$cortical_menu/Control/name.rect_position.y = $cortical_menu/Label.rect_position.y + $cortical_menu/Label.rect_size.y
	$cortical_menu/Control/name_string.rect_position.x = $cortical_menu/Control/name.rect_size.x
	$cortical_menu/Control/name_string.rect_position.y = $cortical_menu/Label.rect_position.y + $cortical_menu/Label.rect_size.y
	$cortical_menu/Control/name2.rect_position.y = $cortical_menu/Control/name.rect_position.y + $cortical_menu/Control/name.rect_size.y + 20
	$cortical_menu/Control/cortical_id.rect_position.x = $cortical_menu/Control/name2.rect_size.x
	$cortical_menu/Control/cortical_id.rect_position.y = $cortical_menu/Control/name_string.rect_size.y + $cortical_menu/Control/name_string.rect_position.y + 5
	$cortical_menu/Control/Properties.rect_position.y = $cortical_menu/Control/name2.rect_size.y + $cortical_menu/Control/name2.rect_position.y + 15
	$cortical_menu/Control/OptionButton.rect_position.y =  $cortical_menu/Control/Properties.rect_position.y + 5
	$cortical_menu/Control/x_string.rect_position.y = $cortical_menu/Control/OptionButton.rect_size.y + $cortical_menu/Control/OptionButton.rect_position.y + 5
	$cortical_menu/Control/y_string.rect_position.y = $cortical_menu/Control/OptionButton.rect_size.y + $cortical_menu/Control/OptionButton.rect_position.y + 5
	$cortical_menu/Control/z_string.rect_position.y = $cortical_menu/Control/OptionButton.rect_size.y + $cortical_menu/Control/OptionButton.rect_position.y + 5
	$cortical_menu/Control/X.rect_position.y = $cortical_menu/Control/x_string.rect_position.y + $cortical_menu/Control/x_string.rect_size.y + 5
	$cortical_menu/Control/Y.rect_position.y = $cortical_menu/Control/y_string.rect_size.y + $cortical_menu/Control/y_string.rect_position.y + 5
	$cortical_menu/Control/Z.rect_position.y = $cortical_menu/Control/z_string.rect_size.y + $cortical_menu/Control/z_string.rect_position.y + 5

	# W D H section
	$cortical_menu/Control/D_string.rect_position.y = $cortical_menu/Control/Z.rect_position.y + $cortical_menu/Control/Z.rect_size.y + 5
	$cortical_menu/Control/H_string.rect_position.y = $cortical_menu/Control/Y.rect_position.y + $cortical_menu/Control/Y.rect_size.y + 5
	$cortical_menu/Control/W_string.rect_position.y = $cortical_menu/Control/X.rect_position.y + $cortical_menu/Control/X.rect_size.y + 5
	$cortical_menu/Control/W.rect_position.y = $cortical_menu/Control/W_string.rect_size.y + $cortical_menu/Control/W_string.rect_position.y 
	$cortical_menu/Control/D.rect_position.y = $cortical_menu/Control/D_string.rect_size.y + $cortical_menu/Control/D_string.rect_position.y + 5
	$cortical_menu/Control/H.rect_position.y = $cortical_menu/Control/H_string.rect_size.y + $cortical_menu/Control/H_string.rect_position.y + 5

	# Delete/Update section
	$cortical_menu/Control/Update.rect_position.y = $cortical_menu/Control/H.rect_position.y + $cortical_menu/Control/H.rect_size.y + 10
	$cortical_menu/Control/remove.rect_position.y = $cortical_menu/Control/H.rect_position.y + $cortical_menu/Control/H.rect_size.y + 10
	if $cortical_menu/Control.visible:
		$cortical_menu.rect_size.y = $cortical_menu/Control/Update.rect_size.y + $cortical_menu/Control/Update.rect_position.y + 10

	# Properties position
	if $button_choice/Control.visible:
		$properties.rect_position.y = $button_choice.rect_size.y + $button_choice.rect_position.y
	else:
		if $properties.rect_position.y >= $button_choice/Label.rect_position.y + $button_choice/Label.rect_size.y + $button_choice.rect_position.y:
			$properties.rect_position.y -= 20
	$properties.rect_size.x = $button_choice.rect_size.x
	$properties/Control/neuron_count_.rect_position.y = $properties/Label.rect_position.y + $properties/Label.rect_size.y + 5
	$properties/Control/neuron_count.rect_position.y = $properties/Label.rect_position.y + $properties/Label.rect_size.y + 5
	$properties/Control/syn.rect_position.y = $properties/Control/neuron_count_.rect_position.y + $properties/Control/neuron_count_.rect_size.y + 15
	$properties/Control/synaptic.rect_position.y = $properties/Control/neuron_count_.rect_position.y + $properties/Control/neuron_count_.rect_size.y + 15
	$properties/Control/post_syn.rect_position.y = $properties/Control/synaptic.rect_position.y + $properties/Control/synaptic.rect_size.y + 15
	$properties/Control/pst_syn.rect_position.y = $properties/Control/synaptic.rect_position.y + $properties/Control/synaptic.rect_size.y + 15
	$properties/Control/post_syn_max.rect_position.y = $properties/Control/post_syn.rect_position.y + $properties/Control/post_syn.rect_size.y + 15
	$properties/Control/pst_syn_max.rect_position.y = $properties/Control/post_syn.rect_position.y + $properties/Control/post_syn.rect_size.y + 15
	$properties/Control/plasticity.rect_position.y = $properties/Control/post_syn_max.rect_position.y + $properties/Control/post_syn_max.rect_size.y + 15
	$properties/Control/plst.rect_position.y = $properties/Control/post_syn_max.rect_position.y + $properties/Control/post_syn_max.rect_size.y + 15
	$properties/Control/fire_count.rect_position.y = $properties/Control/plasticity.rect_position.y + $properties/Control/plasticity.rect_size.y + 15
	$properties/Control/fire.rect_position.y = $properties/Control/plasticity.rect_position.y + $properties/Control/plasticity.rect_size.y + 15
	$properties/Control/refractory.rect_position.y = $properties/Control/fire_count.rect_position.y + $properties/Control/fire_count.rect_size.y
	$properties/Control/refa.rect_position.y = $properties/Control/fire_count.rect_position.y + $properties/Control/fire_count.rect_size.y 
	$properties/Control/leak.rect_position.y = $properties/Control/refractory.rect_position.y + $properties/Control/refractory.rect_size.y
	$properties/Control/leakco.rect_position.y = $properties/Control/refractory.rect_position.y + $properties/Control/refractory.rect_size.y
	$properties/Control/leak_va.rect_position.y = $properties/Control/leak.rect_position.y + $properties/Control/leak.rect_size.y + 10
	$properties/Control/leak_Vtext.rect_position.y = $properties/Control/leak.rect_position.y + $properties/Control/leak.rect_size.y + 10
	$properties/Control/cfr.rect_position.y = $properties/Control/leak_va.rect_position.y + $properties/Control/leak_va.rect_size.y
	$properties/Control/consecutive.rect_position.y = $properties/Control/leak_va.rect_position.y + $properties/Control/leak_va.rect_size.y
	$properties/Control/snze.rect_position.y = $properties/Control/consecutive.rect_position.y + $properties/Control/consecutive.rect_size.y
	$properties/Control/snooze.rect_position.y = $properties/Control/consecutive.rect_position.y + $properties/Control/consecutive.rect_size.y
	$properties/Control/dege.rect_position.y = $properties/Control/snooze.rect_position.y + $properties/Control/snooze.rect_size.y
	$properties/Control/degeneracy.rect_position.y = $properties/Control/snooze.rect_position.y + $properties/Control/snooze.rect_size.y
	$properties/Control/psud.rect_position.y = $properties/Control/degeneracy.rect_position.y + $properties/Control/degeneracy.rect_size.y
	$properties/Control/psp.rect_position.y = $properties/Control/degeneracy.rect_position.y + $properties/Control/degeneracy.rect_size.y
	$properties/Control/Update.rect_position.y = $properties/Control/psp.rect_position.y + $properties/Control/psp.rect_size.y + 10
	if $properties/Control.visible:
		$properties.rect_size.y =  $properties/Control/Update.rect_size.y + $properties/Control/Update.rect_position.y
	# Delete and Update section
	$collapse_1.rect_position.y = $properties/Control/psp.rect_position.y + $properties/Control/psp.rect_size.y + 260

	# Button choice section
	if $cortical_menu/Control.visible:
		$button_choice.rect_position.y = $cortical_menu.rect_size.y + $cortical_menu.rect_position.y - 5
		$button_choice.rect_size.x = $cortical_menu.rect_size.x
		$button_choice/Label.rect_position.y = 0
		$button_choice/Control/Label.rect_position.y = $button_choice/Label.rect_position.y + $button_choice/Label.rect_size.y
		$button_choice/Control/Label2.rect_position.y = $button_choice/Label.rect_position.y + $button_choice/Label.rect_size.y
		$button_choice/Control/mem.rect_position.y = $button_choice/Control/Label.rect_position.y + $button_choice/Control/Label.rect_size.y
		$button_choice/Control/syn.rect_position.y = $button_choice/Control/Label2.rect_position.y + $button_choice/Control/Label2.rect_size.y
		$button_choice.rect_size.y = $button_choice/Control/syn.rect_position.y + $button_choice/Control/syn.rect_size.y
	else:
		if $button_choice.rect_position.y > $collapse_4.rect_position.y + $cortical_menu/title.rect_position.y + $cortical_menu/title.rect_size.y + 20:
			$button_choice.rect_position.y -= 20
		$button_choice.rect_size.x = $cortical_menu.rect_size.x
		$button_choice/Label.rect_position.y = -5
		$button_choice/Control/mem.rect_position.y = $button_choice/Label.rect_position.y + $button_choice/Label.rect_size.y + 30
		$button_choice/Control/syn.rect_position.y = $button_choice/Label.rect_position.y + $button_choice/Label.rect_size.y + 30
	# Cortical mappoing section
	if $properties/Control.visible == false:
		if $cortical_mapping.rect_position.y > $properties.rect_position.y + 45:
			$cortical_mapping.rect_position.y -= 20
			$properties.rect_size.y = $cortical_menu/title.rect_size.y + $cortical_menu/title.rect_position.y
		$cortical_mapping.rect_size.x = $properties.rect_size.x
		$cortical_mapping/Control/afferent_label.rect_position.y = $cortical_mapping/Label.rect_position.y + $cortical_mapping/Label.rect_size.y
		$cortical_mapping/Control/afferent.rect_position.y = $cortical_mapping/Control/afferent_label.rect_position.y + $cortical_mapping/Control/afferent_label.rect_size.y + 5
		$cortical_mapping/Control/efferent_label.rect_position.y = $cortical_mapping/Control/afferent.rect_position.y + $cortical_mapping/Control/afferent.rect_size.y + 10
		$cortical_mapping/Control/cortical_mapping_add.rect_position.y = $cortical_mapping/Control/afferent.rect_position.y + $cortical_mapping/Control/afferent.rect_size.y
		$cortical_mapping/Control/ScrollContainer.rect_position.y = $cortical_mapping/Control/cortical_mapping_add.rect_position.y + $cortical_mapping/Control/cortical_mapping_add.rect_size.y + 10
	else:
		$cortical_mapping.rect_position.y = $properties.rect_size.y + $properties.rect_position.y
		$cortical_mapping.rect_size.x = $properties.rect_size.x
		$cortical_mapping/Control/afferent_label.rect_position.y = $cortical_mapping/Label.rect_position.y + $cortical_mapping/Label.rect_size.y
		$cortical_mapping/Control/afferent.rect_position.y = $cortical_mapping/Control/afferent_label.rect_position.y + $cortical_mapping/Control/afferent_label.rect_size.y + 5
		$cortical_mapping/Control/efferent_label.rect_position.y = $cortical_mapping/Control/afferent.rect_position.y + $cortical_mapping/Control/afferent.rect_size.y
		$cortical_mapping/Control/cortical_mapping_add.rect_position.y = $cortical_mapping/Control/afferent.rect_position.y + $cortical_mapping/Control/afferent.rect_size.y
		$cortical_mapping/Control/ScrollContainer.rect_position.y = $cortical_mapping/Control/cortical_mapping_add.rect_position.y + $cortical_mapping/Control/cortical_mapping_add.rect_size.y + 10

	# collapse section
	$collapse_1.rect_position.y = $properties.rect_position.y + 5
	$collapse_1.rect_position.x = $properties.rect_size.x + $properties.rect_position.x - 30
	$collapse_2.rect_position.x = $button_choice/Label.rect_size.x + $button_choice/Label.rect_position.x - 30
	$collapse_2.rect_position.y = $button_choice.rect_position.y + $button_choice/Label.rect_position.y
	$collapse_3.rect_position.y = $cortical_mapping.rect_position.y
	$collapse_3.rect_position.x = $cortical_mapping.rect_position.x + $cortical_mapping.rect_size.x - 30
	$collapse_4.rect_position.x = $close_for_all.rect_position.x
	$collapse_4.rect_position.y = $close_for_all.rect_size.y + $close_for_all.rect_position.y - 5
	$collapse_4.rect_size = Vector2(35,35)
	
#	if one_time_flag:
#		$button_choice.visible = true
#		$cortical_mapping.rect_position.y = $properties.rect_size.y + $properties.rect_position.y
#		$cortical_mapping.rect_size.x = $button_choice.rect_size.x
#		$collapse_2.emit_signal("pressed")
#		$collapse_4.emit_signal("pressed")
#		$cortical_menu.visible = false
#		$cortical_mapping.visible = false
#		$button_choice.visible = false
#		$properties.visible = false
#		$Mapping_Properties.visible = false
#		$collapse_1.visible = false
#		$collapse_2.visible = false
#		$collapse_3.visible = false
#		$collapse_4.visible = false
#		one_time_flag = false
	
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
#		$button_choice.rect_size.y = $button_choice/Control.rect_size.y + $button_choice/Control.rect_position.y
#		$collapse_2.icon = icon_pressed
#	else:
#		$button_choice/Control.visible = true
#		$collapse_2.icon = icon_not_pressed
#		$button_choice.rect_size.y = $button_choice/Control/syn.rect_size.y + $button_choice/Control/syn.rect_position.y + 15
#	$collapse_2.release_focus()
#
#func _on_collapse_3_pressed():
#	if $cortical_mapping/Control.visible:
#		$cortical_mapping/Control.visible = false
#		$collapse_3.icon = icon_pressed
#		$cortical_mapping.rect_size.y = $cortical_mapping/Label.rect_position.y + $cortical_mapping/Label.rect_size.y
#	else:
#		$cortical_mapping/Control.visible = true
#		$collapse_3.icon = icon_not_pressed
#		$cortical_mapping.rect_size.y = $cortical_mapping/Control/ScrollContainer.rect_size.y + $cortical_mapping/Control/ScrollContainer.rect_position.y
#	$collapse_3.release_focus()
#
func _on_collapse_4_pressed():
	if $cortical_menu/Control.visible:
		$cortical_menu/Control.visible = false
		$collapse_4.icon = icon_pressed
		$cortical_menu.rect_size.y = $cortical_menu/Label.rect_size.y + $cortical_menu/Label.rect_position.y
	else:
		$cortical_menu/Control.visible = true
		$collapse_4.icon = icon_not_pressed
	$collapse_4.release_focus()
