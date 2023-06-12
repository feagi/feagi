extends Element_Base
class_name Element_Field

const D_editable = true

var value: String:
	get: return _LineEdit.value
	set(v): _LineEdit.value = v

var editable: bool:
	get: return _LineEdit.editable
	set(v): _LineEdit.editable = v

var _LineEdit: LineEdit_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _LineEdit = get_children()[1]
	else: _LineEdit = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["field"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
