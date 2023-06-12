extends CheckBox
class_name CheckBox_Sub

var editable: bool:
	get: return !disabled
	set(v): disabled = !v
