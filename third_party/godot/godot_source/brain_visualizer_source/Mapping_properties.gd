extends ColorRect
var flag = false

# Called when the node enters the scene tree for the first time.
func _ready():
	visible = false

func _process(_delta):
	if not(visible):
		$inside_mapping_menu/Control/psp_multipler_text.text = ""
		$inside_mapping_menu/Control/over1.value = 0
		$inside_mapping_menu/Control/over2.value = 0
		$inside_mapping_menu/Control/over3.value = 0
		if $inside_mapping_menu/Control/Mapping_def.get_item_count() > 0:
			$inside_mapping_menu/Control/Mapping_def.select(0)
	if $source_dropdown.get_selected_id() != 0 and $cortical_dropdown.get_selected_id() != 0:
		$inside_mapping_menu.visible = true
	else:
		$inside_mapping_menu.visible = false
	if $inside_mapping_menu.visible:
		$update_inside_map.disabled = false
	else:
		$update_inside_map.disabled = true
