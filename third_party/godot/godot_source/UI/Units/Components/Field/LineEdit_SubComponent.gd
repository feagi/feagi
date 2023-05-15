extends LineEdit
class_name LineEdit_SubComponent

signal SizeChanged(selfReference)

var Hsize: Vector2:
	get: return size
	set(v): 
		size = v
		SizeChanged.emit(self)

var fieldWidth: float:
	get: return size.x
	set(v):
		size.x = v
		SizeChanged.emit(self)
