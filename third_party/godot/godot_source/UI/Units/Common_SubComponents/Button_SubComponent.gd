extends Button
class_name Button_SubComponent

# NOTE: This is DIFFERENT from the Button COmponent, this is for adding button to existing components
# (IE, the dropdown), and not intended to be used as a component directly. Please see
# Button_Component.gd for that instead!

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

# Use to toggle pressability
var editable: bool:
	get: return !disabled
	set(v): disabled = !v
