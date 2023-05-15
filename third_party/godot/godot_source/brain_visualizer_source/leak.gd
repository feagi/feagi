extends LineEdit

var value : float = 0.0
@onready var line = self

func _ready():
	line.connect("text_changed",Callable(self,"_on_text_changed"))
#	line.connect("text_submitted",Callable(self,"_on_text_entered"))

func _on_text_changed(new_text):
	if new_text.is_valid_float():
		line.value = float(new_text)
	
func _input(event):
	if event.is_action_pressed("ui_up") and self.has_focus():
		Godot_list.Node_2D_control = true
	if event.is_action_pressed("ui_down") and self.has_focus():
		Godot_list.Node_2D_control = true
	if event.is_action_pressed("ui_left") and self.has_focus():
		Godot_list.Node_2D_control = true
	if event.is_action_pressed("ui_right") and self.has_focus():
		Godot_list.Node_2D_control = true
