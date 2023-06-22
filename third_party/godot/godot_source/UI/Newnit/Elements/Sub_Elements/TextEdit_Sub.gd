extends TextEdit
class_name TextEdit_Sub

signal value_edited(newString: String)

var minWidth: float:
	get: return get_theme_font("font").get_string_size(text).x

var value:
	get: return text
	set(v): text = v

var _cacheText: String

func _ready():
	text_changed.connect(_TextChangeProxy)

func _TextChangeProxy() -> void:
	if text == _cacheText: return
	_cacheText = text
	value_edited.emit(text)

## TODO this camera focusing system is flawed, and should be replaced
#func _ready():
#	mouse_entered.connect(_toggleCamUsageOn)
#	mouse_exited.connect(_toggleCamUsageOff)
#
#func _toggleCamUsageOn():
#	Godot_list.Node_2D_control = true
#
#func _toggleCamUsageOff():
#	Godot_list.Node_2D_control = false


# built in vars
# text: String
# size: Vector2
# editable: bool
# expand_to_text_length: bool
# max_length: int
# text_changed: Signal
# text_submitted: Signal
# placeholder_text: String
