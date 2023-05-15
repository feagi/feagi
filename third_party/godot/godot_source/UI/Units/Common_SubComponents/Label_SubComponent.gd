extends Label
class_name Label_SubComponent

# contains Common Label Functions

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
