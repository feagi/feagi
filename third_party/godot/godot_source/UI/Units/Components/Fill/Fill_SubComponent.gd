extends Panel
class_name Fill_SubComponent


signal SizeChanged(selfReference)

var Hsize: Vector2:
	get: return size
	set(v):
		size = v
		SizeChanged.emit(self)

var customColor: Color:
	get: return modulate
	set(v): modulate = v
