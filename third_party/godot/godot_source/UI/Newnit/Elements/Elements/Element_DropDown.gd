extends Element_Base
class_name Element_DropDown

const D_editable = true

var value: String:
	get: return _optionButton.value
	set(v): _optionButton.value = v

var _optionButton: OptionButton_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _optionButton = get_children()[1]
	else: _optionButton = get_children()[0]

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["dropDown"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
