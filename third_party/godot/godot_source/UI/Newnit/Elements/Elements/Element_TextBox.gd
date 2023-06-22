extends Element_Base
class_name Element_TextBox

const D_editable = true
const D_value = ""
const D_expand_to_text_length = false
const D_max_length = 50
const D_placeholder_text = ""

const _specificSettableProps = {
	"value": TYPE_STRING,
	"editable": TYPE_BOOL,
	"placeholder_text": TYPE_STRING,
	"wrap_mode": TYPE_INT,
}

var value: String:
	get: return _TextEdit.value
	set(v): _TextEdit.value = v

var editable: bool:
	get: return _TextEdit.editable
	set(v): _TextEdit.editable = v

var placeholder_text: String:
	get: return _TextEdit.placeholder_text
	set(v): _TextEdit.placeholder_text

var wrap_mode: int:
	get: return _TextEdit.wrap_mode
	set(v): _TextEdit.wrap_mode

var _TextEdit: TextEdit_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _TextEdit = get_children()[1]
	else: _TextEdit = get_children()[0]
	
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	placeholder_text = HelperFuncs.GetIfCan(settings, "placeholder_text", D_placeholder_text)
	_TextEdit.text = HelperFuncs.GetIfCan(settings, "value", D_value)
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
