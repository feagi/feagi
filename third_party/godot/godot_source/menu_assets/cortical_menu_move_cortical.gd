extends ColorRect

func _ready():
	visible = false
	
func _process(_delta):
	if $cortical_mapping.visible:
		$button_choice/afferent.visible = true
	else:
		$button_choice/afferent.visible = false
