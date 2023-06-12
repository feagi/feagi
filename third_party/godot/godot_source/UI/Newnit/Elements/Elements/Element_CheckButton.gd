extends Element_Base
class_name Element_CheckButton

const D_editable = true

var value: bool:
	get: return _CheckButton.button_pressed

var editable: bool:
	get: return _CheckButton.editable
	set(v): _CheckButton.editable = v

var _CheckButton: CheckButton_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _CheckButton = get_children()[1]
	else: _CheckButton = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)


func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["checkButton"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
