extends SpinBox
class_name SpinBox_SubComponent

# Nothing special needed here, just the required Hsize

signal SizeChanged(selfReference)

var Hsize: Vector2:
	get: return size
	set(v):
		size = v
		SizeChanged.emit(self)

func _ready():
	mouse_entered.connect(_toggleCamUsageOn)
	mouse_exited.connect(_toggleCamUsageOff)

func _toggleCamUsageOn():
	Godot_list.Node_2D_control = true

func _toggleCamUsageOff():
	Godot_list.Node_2D_control = false
