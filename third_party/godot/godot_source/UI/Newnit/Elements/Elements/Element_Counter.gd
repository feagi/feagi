extends Element_Base
class_name Element_Counter

const D_editable = true
const D_value = 0
const specificSettableProps = {
	"value": TYPE_INT,
	"editable": TYPE_BOOL
}

var value: int:
	get: return int(_SpinBox.value)
	set(v): _SpinBox.value = v

var editable: bool:
	get: return _SpinBox.editable
	set(v): _SpinBox.editable = v

var _SpinBox: Spinbox_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _SpinBox = get_children()[1]
	else: _SpinBox = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	_SpinBox.value = HelperFuncs.GetIfCan(settings, "value", D_value)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["counter"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
