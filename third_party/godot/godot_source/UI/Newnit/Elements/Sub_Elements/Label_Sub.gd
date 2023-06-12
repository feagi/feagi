extends Label
class_name Label_Sub

# Contains functions relevant for labels

var minTextSize: Vector2:
	get: return get_theme_font("font").get_string_size(text)

# built in vars
# text: String
# size: Vector2
