extends Button
class_name Button_Sub

signal value_edited(bool)

var editable: bool:
	get: return !disabled
	set(v): disabled = !v

func _ready():
	pressed.connect(_PressProxy)

func _PressProxy():
	value_edited.emit(true) # we need some sort of value in here

# built in vars
# text: String
# size: Vector2
# signal pressed
