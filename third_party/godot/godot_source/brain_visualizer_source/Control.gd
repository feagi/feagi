extends Control


var flag = true


# Called when the node enters the scene tree for the first time.
func _ready():
	visible = false

func _process(_delta):
	if $inner_box/morphology_type.selected != 0 and $inner_box/morphology_name.text != "":
		$create.disabled = false
	else:
		$create.disabled = true
	if $inner_box/morphology_name.text != "":
		$inner_box/Label3.visible = true
		$inner_box/morphology_type.visible = true
	else:
		$inner_box/Label3.visible = false
		$inner_box/morphology_type.visible = false
		if $inner_box/morphology_type.get_item_count() > 0:
			$inner_box/morphology_type.select(0)


	if $inner_box/morphology_type.selected != 0:
		$inner_box/Label4.visible = true
		$inner_box/Button.visible = true
	else:
		$inner_box/Label4.visible = false
		$inner_box/Button.visible = false

	
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
				$inner_box/grey_bg.size = Vector2(481, 342)
				flag = true
				$ColorRect.size = Vector2(519, 437)
				$create.position = Vector2(152, 387)
				$inner_box/box_of_composite.visible = true
				$inner_box/Button.visible = false
				if $inner_box/box_of_composite/mapper_composite.get_selected_id() == 0:
					$create.disabled = true
				else:
					$create.disabled = false
			else:
				if flag:
					$inner_box/grey_bg.size = Vector2(481, 283)
					$ColorRect.size = Vector2(519, 394)
					$create.position = Vector2(152, 330)
					flag = false
				$inner_box/box_of_composite.visible = false
				$inner_box/Button.visible = true
		else:
				$inner_box/box_of_pattern.visible = false
				$inner_box/box_of_vectors.visible = false
				$inner_box/box_of_vectors/labels.visible = false
				$inner_box/box_of_pattern/labels.visible = false
				$inner_box/box_of_composite.visible = false
				$inner_box/box_of_composite.visible = false
			
