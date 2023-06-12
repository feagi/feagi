extends CheckButton
class_name CheckButton_Sub

var editable: bool:
	get: return !disabled
	set(v): disabled = !v

# built in vars
# disabled: bool
# button_pressed: bool
