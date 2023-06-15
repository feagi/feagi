extends Element_Base
class_name Element_Field

const D_editable = true
const D_value = ""
const D_expand_to_text_length = false
const D_max_length = 50
const D_placeholder_text = ""

const _specificSettableProps = {
	"value": TYPE_STRING,
	"editable": TYPE_BOOL,
	"expand_to_text_length": TYPE_BOOL,
	"max_length": TYPE_INT,
	"placeholder_text": TYPE_STRING,
}

var value: String:
	get: return _LineEdit.value
	set(v): _LineEdit.value = v

var editable: bool:
	get: return _LineEdit.editable
	set(v): _LineEdit.editable = v

var expand_to_text_length: bool:
	get: return _LineEdit.expand_to_text_length
	set(v): _LineEdit.expand_to_text_length = v

var max_length: int:
	get: return _LineEdit.max_length
	set(v): _LineEdit.max_length = v

var placeholder_text: String:
	get: return _LineEdit.placeholder_text
	set(v): _LineEdit.placeholder_text

var _LineEdit: LineEdit_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _LineEdit = get_children()[1]
	else: _LineEdit = get_children()[0]
	
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	expand_to_text_length = HelperFuncs.GetIfCan(settings, "expand_to_text_length", D_expand_to_text_length)
	max_length = HelperFuncs.GetIfCan(settings, "max_length", D_max_length)
	placeholder_text = HelperFuncs.GetIfCan(settings, "placeholder_text", D_placeholder_text)
	_LineEdit.text = HelperFuncs.GetIfCan(settings, "value", D_value)
	_runtimeSettableProperties.merge(_specificSettableProps)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["field"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(newString) -> void:
	DataUp.emit({"value": newString}, ID, self)
