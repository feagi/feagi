extends CheckBox
class_name CheckBox_SubComponent

# Nothing special needed here, just the required Hsize
# (changing the size of a checkbox is a bit weird thoough)

signal SizeChanged(selfReference)

var Hsize: Vector2:
	get: return size
	set(v): 
		size = v
		SizeChanged.emit(self)
