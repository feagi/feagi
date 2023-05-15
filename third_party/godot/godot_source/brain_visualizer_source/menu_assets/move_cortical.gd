extends Button

func _ready():
	visible = false

func _process(_delta):
	if $cortical_menu.visible == false:
		$cortical_menu/name.text = ""


#func _on_move_cortical_pressed():
#	if $cortical_menu.visible:
#		$cortical_menu.visible = false
#	else:
#		$cortical_menu.visible = true
