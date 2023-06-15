extends LineEdit
class_name LineEdit_ff_Sub

#TODO - minimum, maximum, prefix, suffix

signal value_edited(val: float)

var minWidth: float:
	get: return get_theme_font("font").get_string_size(text).x

var value: float:
	get: return float(_cachedText)
	set(v): text = str(v); _cachedText = str(v)

var min_value := -INF
var max_value := INF

var _cachedText: String = "0.0"

func _ready():
	text_changed.connect(_TextChangedRelay)

func _TextChangedRelay(input: String) -> void:
	if !input.is_valid_float(): return
	_cachedText = input
	
	value_edited.emit(float(input))

# built in vars
# text: String
# size: Vector2
# editable: bool
# expand_to_text_length: bool
# max_length: int
# text_changed: Signal
# text_submitted: Signal
# placeholder_text: String
