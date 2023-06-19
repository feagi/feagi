extends Element_Base
class_name Element_Counter

const D_editable = true
const D_value = 0
const D_max_value: int = int(INF)
const D_min_value: int = int(-INF)
const D_step = 1.0
const D_prefix = ""
const D_suffix = ""

const _specificSettableProps = {
	"value": TYPE_INT,
	"editable": TYPE_BOOL,
	"max_value": TYPE_INT,
	"min_value": TYPE_INT,
	"step": TYPE_FLOAT,
	"prefix": TYPE_STRING,
	"suffix": TYPE_STRING
}

var value: int:
	get: return int(_SpinBox.value)
	set(v): _SpinBox.value = v

var editable: bool:
	get: return _SpinBox.editable
	set(v): _SpinBox.editable = v

var max_value: int:
	get: return _SpinBox.max_value
	set(v): _SpinBox.max_value = v

var min_value: int:
	get: return _SpinBox.min_value
	set(v): _SpinBox.min_value = v

var step: float:
	get: return _SpinBox.step
	set(v): _SpinBox.step = v

var prefix: String:
	get: return _SpinBox.prefix
	set(v): _SpinBox.prefix = v

var suffix: String:
	get: return _SpinBox.suffix
	set(v): _SpinBox.suffix = v

var _SpinBox: Spinbox_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _SpinBox = get_children()[1]
	else: _SpinBox = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	_SpinBox.value = HelperFuncs.GetIfCan(settings, "value", D_value)
	max_value = HelperFuncs.GetIfCan(settings, "max_value", D_max_value)
	min_value = HelperFuncs.GetIfCan(settings, "min_value", D_min_value)
	step = HelperFuncs.GetIfCan(settings, "step", D_step)
	prefix = HelperFuncs.GetIfCan(settings, "prefix", D_prefix)
	suffix = HelperFuncs.GetIfCan(settings, "suffix", D_suffix)
	_runtimeSettableProperties.merge(_specificSettableProps)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["counter"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(numIn) -> void:
	DataUp.emit({"value": numIn}, ID, self)
