extends TextEdit


@onready var line = self

func _ready():
	line.connect("text_changed",Callable(self,"_on_text_changed"))

	
func _on_text_changed():
	Godot_list.Node_2D_control = true
