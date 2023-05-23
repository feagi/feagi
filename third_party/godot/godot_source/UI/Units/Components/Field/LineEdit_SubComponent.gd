extends LineEdit
class_name LineEdit_SubComponent

signal SizeChanged(selfReference)
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

var Htext: String:
	get: return text # lol
	set(v):
		text = v
		if(shouldScaleWithInputText): return
		ScaleWithInputText()

func ScaleWithInputText():
	Hsize = Vector2(textWidth + INTERNAL_WIDTH_PADDING, Hsize.y)
	
