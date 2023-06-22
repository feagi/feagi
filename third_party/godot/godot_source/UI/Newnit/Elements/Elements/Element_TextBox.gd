extends Element_Base
class_name Element_TextBox

const D_editable = true
const D_value = ""
const D_expand_to_text_length = false
const D_max_length = 50
const D_placeholder_text = ""
const D_fill_textBox = true
const D_wrap_mode = 0

const _specificSettableProps = {
	"value": TYPE_STRING,
	"editable": TYPE_BOOL,
	"placeholder_text": TYPE_STRING,
	"wrap_mode": TYPE_INT,
	"fill_textBox": TYPE_BOOL
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

var fill_textBox: bool:
	set(v): 
		if v:
			if vertical: _TextEdit.size_flags_vertical = 3
			else: _TextEdit.size_flags_horizontal = 3
		else:
			if vertical: _TextEdit.size_flags_vertical = 1
			else: _TextEdit.size_flags_horizontal = 1

var _TextEdit: TextEdit_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _TextEdit = get_children()[1]
	else: _TextEdit = get_children()[0]
	
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	placeholder_text = HelperFuncs.GetIfCan(settings, "placeholder_text", D_placeholder_text)
	_TextEdit.text = HelperFuncs.GetIfCan(settings, "value", D_value)
	fill_textBox = HelperFuncs.GetIfCan(settings, "fill_textBox", D_fill_textBox)
	wrap_mode = HelperFuncs.GetIfCan(settings, "wrap_mode", D_wrap_mode)
	_runtimeSettableProperties.merge(_specificSettableProps)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["textbox"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(newString) -> void:
	DataUp.emit({"value": newString}, ID, self)
