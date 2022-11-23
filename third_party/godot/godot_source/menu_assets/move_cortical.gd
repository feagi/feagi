extends Button

func _ready():
	visible = false

func _on_move_cortical_toggled():
	$cortical_menu.visible = toggle_mode
	
func _process(_delta):
	if $cortical_menu.visible == false:
		$cortical_menu/name.text = ""
