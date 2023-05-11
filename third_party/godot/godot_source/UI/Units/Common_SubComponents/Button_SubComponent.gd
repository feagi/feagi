extends Button
class_name Button_SubComponent

signal SizeChanged(selfReference)

var Hsize: Vector2:
	get: return size
	set(v):
		size = v
		SizeChanged.emit(self)

# Proxy for setting text, also emits size changes
var Htext: String:
	get: return text
	set(v):
		text = v
		SizeChanged.emit(self)
