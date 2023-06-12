extends Element_Base
class_name Element_Label
# Yes, you can add a label to a label

var value: String:
	get: return _label.text
	set(v): _label.text = v

var _label: Label_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _label = get_children()[1]
	else: _label = get_children()[0]


func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["label"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
