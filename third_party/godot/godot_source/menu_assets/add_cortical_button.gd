extends Button


func _ready():
	visible = false


func _on_Button_pressed():
	pass # Replace with function body.


func _on_add_cortical_button_pressed():
	if $cortical_menu.visible:
		$cortical_menu.visible = false
	else:
		$cortical_menu.visible = true
#
#	if $name.visible:
#		$name.visible = false
#		$name_string.visible = false
#	else:
#		$name.visible = true
#		$name_string.visible = true

func _on_LineEdit_text_changed(new_text):
	pass
