extends Control


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	visible = false

func _process(_delta):
	if $inner_box/morphology_type.selected != 0 and $inner_box/morphology_name.text != "":
		$create.disabled = false
	else:
		$create.disabled = true
	if $inner_box/morphology_type.get_item_count() >= 1:
		if $inner_box/morphology_type.selected != 0:
			if $inner_box/morphology_type.get_item_text($inner_box/morphology_type.selected) == "patterns":
				$inner_box/box_of_pattern.visible = true
				$inner_box/box_of_pattern/labels.visible = true
			else:
				$inner_box/box_of_pattern.visible = false
				$inner_box/box_of_pattern/labels.visible = false
			if $inner_box/morphology_type.get_item_text($inner_box/morphology_type.selected) == "vectors":
				$inner_box/box_of_vectors.visible = true
				$inner_box/box_of_vectors/labels.visible = true
			else:
				$inner_box/box_of_vectors.visible = false
				$inner_box/box_of_vectors/labels.visible = false
			if $inner_box/morphology_type.get_item_text($inner_box/morphology_type.selected) == "composite":
				$inner_box/grey_bg.rect_size = Vector2(481, 342)
				$ColorRect.rect_size = Vector2(519, 437)
				$create.rect_position = Vector2(152, 387)
				$inner_box/box_of_composite.visible = true
				$inner_box/Button.visible = false
			else:
				$inner_box/grey_bg.rect_size = Vector2(481, 283)
				$ColorRect.rect_size = Vector2(519, 394)
				$create.rect_position = Vector2(152, 330)
				$inner_box/box_of_composite.visible = false
				$inner_box/Button.visible = true
		else:
				$inner_box/box_of_pattern.visible = false
				$inner_box/box_of_vectors.visible = false
				$inner_box/box_of_vectors/labels.visible = false
				$inner_box/box_of_pattern/labels.visible = false
			
