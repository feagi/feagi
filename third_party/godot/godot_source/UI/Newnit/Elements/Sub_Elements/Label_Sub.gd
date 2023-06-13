extends Label
class_name Label_Sub

# Contains functions relevant for labels

signal value_edited() # This never emits, but is required for compatibility with greater system

var minTextSize: Vector2:
	get: return get_theme_font("font").get_string_size(text)

# built in vars
# text: String
# size: Vector2
