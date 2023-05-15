extends SpinBox


@onready var line = get_line_edit()

func _ready():
	line.connect("text_submitted",Callable(self,"_on_text_entered"))
	line.connect("text_changed",Callable(self,"_on_text_changed"))

func _on_text_entered(_new_text):
	line.release_focus()
	
func _on_text_changed(_new_text):
	Godot_list.Node_2D_control = true
