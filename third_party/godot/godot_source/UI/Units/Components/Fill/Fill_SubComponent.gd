extends Panel
class_name Fill_SubComponent


signal SizeChanged(selfReference)

var _style: StyleBoxFlat = StyleBoxFlat.new()

var Hsize: Vector2:
	get: return size
	set(v):
		size = v
		SizeChanged.emit(self)

var customColor: Color:
	get: return modulate
	set(v): 
		_style.bg_color = v
		add_theme_stylebox_override("panel", _style)
		

func resetColor() -> void:
	remove_theme_stylebox_override("panel")
