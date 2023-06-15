extends CheckButton
class_name CheckButton_Sub

signal value_edited(bool)

var editable: bool:
	get: return !disabled
	set(v): disabled = !v

var value: bool:
	get: return button_pressed
	set(v): set_pressed_no_signal(v)

func _ready():
	toggled.connect(_ToggleProxy)

func _ToggleProxy(newVal: bool):
	value_edited.emit(newVal)

# built in vars
# disabled: bool
# button_pressed: bool
