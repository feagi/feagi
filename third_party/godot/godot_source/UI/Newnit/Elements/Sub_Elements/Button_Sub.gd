extends Button
class_name Button_Sub

var editable: bool:
	get: return !disabled
	set(v): disabled = !v

# built in vars
# text: String
# size: Vector2
# signal pressed
