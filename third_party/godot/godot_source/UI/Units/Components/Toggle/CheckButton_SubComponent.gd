extends CheckButton
class_name CheckButton_SubComponent

# Nothing special needed here, just the required Hsize

signal SizeChanged(selfReference)


var Hsize: Vector2:
	get: return size
	set(v): 
		size = v
		SizeChanged.emit(v, self)
