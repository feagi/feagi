extends SpinBox
class_name Spinbox_Sub

signal value_edited(newValue: int) # Bit more failsafe than stock signal

var _prevValue: float

func _ready():
	_prevValue = value

func GaurdProxyValueChange(checkingValue) -> void:
	if checkingValue == _prevValue: return # avoid spam of same number
	_prevValue = checkingValue
	value_edited.emit(checkingValue)


# TODO this camera focusing system is flawed, and should be replaced
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
# editable: bool
# max_value: float
# min_value: float
# step: float
# value: float
# rounded: bool
# prefix: String
# suffix: String
# value_changed: Signal(float)
