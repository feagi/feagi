extends Element_Base
class_name Element_DropDown

const D_editable = true
const D_index = -1
const D_options = []
const D_value = ""

const _specificSettableProps = {
	"value": TYPE_STRING,
	"editable": TYPE_BOOL,
	"options": TYPE_ARRAY
}


var value: String:
	get: return _optionButton.value
	set(v): _optionButton.value = v

var options: Array:
	get: return _optionButton.options
	set(v): _optionButton.options = v

var index: int:
	get: return _optionButton.index
	set(v): _optionButton.index

var editable: bool:
	get: return _optionButton.editable
	set(v): _optionButton.editable = v

var _optionButton: OptionButton_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _optionButton = get_children()[1]
	else: _optionButton = get_children()[0]
	
	options = HelperFuncs.GetIfCan(settings, "options", D_options)
	value = HelperFuncs.GetIfCan(settings, "value", D_value)
	editable = HelperFuncs.GetIfCan(settings, "editable", TYPE_BOOL)
	_runtimeSettableProperties.merge(_specificSettableProps)
	

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["dropDown"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
		"selectedIndex": index
	}

func _DataUpProxy(data) -> void: #Data already is a proper Formatted Dictionary
	DataUp.emit(data, ID, self)
