extends LineEdit

var value : float = 0.0

func _ready():
	pass

func _on_leak_text_changed(new_text):
	if new_text.is_valid_float():
		value = float(new_text)


func _on_leak_Vtext_mouse_exited():
	release_focus()


func _on_leak_mouse_exited():
	release_focus()
