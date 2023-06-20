extends Element_Base
class_name Element_Label
# Yes, you can add a label to a label, but why?

const D_text := ""
const _specificSettableProps := {
	"value": TYPE_STRING,
	"text": TYPE_STRING
}


var value: String:
	get: return _label.text
	set(v): _label.text = v

var text: String: # Same as 'value', here just for ease of use
	get: return _label.text
	set(v): _label.text = v

var _label: Label_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _label = get_child(1)
	else: _label = get_child(0)
	text = HelperFuncs.GetIfCan(settings, "text", D_text)
	_runtimeSettableProperties.merge(_specificSettableProps)


func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["label"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

# This never gets called, but here for compatibility reasons
func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
