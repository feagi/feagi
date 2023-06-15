extends Element_Base
class_name Element_FloatField

#TODO - minimum, maximum, prefix, suffix

const D_editable = true

const _specificSettableProps = {
	"value": TYPE_INT,
	"editable": TYPE_BOOL,
}

var value: float:
	get: return _LineEditFF.value
	set(v): _LineEditFF.value = v

var editable: bool:
	get: return _LineEditFF.editable
	set(v): _LineEditFF.editable = v

var _LineEditFF: LineEdit_ff_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _LineEditFF = get_children()[1]
	else: _LineEditFF = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	_runtimeSettableProperties.merge(_specificSettableProps)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["floatField"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(newFloat) -> void:
	DataUp.emit({"value": newFloat}, ID, self)
