extends Element_Base
class_name Element_CheckBox

const D_editable = true

var value: bool:
	get: return _CheckBox.button_pressed

var editable: bool:
	get: return _CheckBox.editable
	set(v): _CheckBox.editable = v

var _CheckBox: CheckBox_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _CheckBox = get_children()[1]
	else: _CheckBox = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["checkbox"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
