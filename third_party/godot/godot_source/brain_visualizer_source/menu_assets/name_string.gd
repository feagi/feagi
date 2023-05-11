extends LineEdit

func _ready():
	visible = true
	
func _process(_delta):
	if visible == false:
		$name_string.value = ""


func _on_name_string_mouse_exited():
	release_focus()
