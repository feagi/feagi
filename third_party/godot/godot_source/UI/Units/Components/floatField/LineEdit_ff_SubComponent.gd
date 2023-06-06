extends LineEdit
class_name LineEdit_ff_SubComponent

signal SizeChanged(selfReference)
signal FloatChanged(float, selfReference)
const INTERNAL_WIDTH_PADDING := 4.0

var shouldScaleWithInputText := false

var Hsize: Vector2:
	get: return size
	set(v): 
		size = v
		SizeChanged.emit(self)

var fieldWidth: float:
	get: return size.x
	set(v):
		size.x = v

var textWidth: float:
	get: return get_theme_font("font").get_string_size(text).x

var HFloat: float:
	get: return _HFloat

var _HFloat: float = 0.0

func _ready():
	text_changed.connect(UpdateHFloat)
	focus_entered.connect(_toggleCamUsageOn)
	focus_exited.connect(_toggleCamUsageOff)

func _ScaleWithInputText() -> void:
	if(!shouldScaleWithInputText): return
	Hsize = Vector2(textWidth + INTERNAL_WIDTH_PADDING, Hsize.y)

func _TextChangeRelay(_input: String) -> void:
	_ScaleWithInputText()

func _toggleCamUsageOn():
	Godot_list.Node_2D_control = true

func _toggleCamUsageOff():
	Godot_list.Node_2D_control = false

# How you update the HFloat value, returns true if input is valid, else false
func UpdateHFloat(requested) -> bool:
	if typeof(requested) == TYPE_FLOAT:
		_HFloat = requested
		return true
	if typeof(requested) != TYPE_STRING: return false
	if requested.is_valid_float():
		# input seems valid, pass through
		_HFloat = float(requested)
		_TextChangeRelay(requested)
		return true
	return false
