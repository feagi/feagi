extends Element_Base
class_name Element_Button
# Yes, you can technically enable the sideButton for this Button element if you wanted to

const D_editable = true
var specificSettableProps := {
	"editable": TYPE_BOOL
}

var value: bool:
	get: return _Button.button_pressed

var editable: bool:
	get: return _Button.editable
	set(v): _Button.editable = v

var _Button: Button_Sub

func _ActivationSecondary(settings: Dictionary) -> void:
	if(_has_label): _Button = get_children()[1]
	else: _Button = get_children()[0]
	editable = HelperFuncs.GetIfCan(settings, "editable", D_editable)
	specificSettableProps.merge(settableProperties)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["button"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit(_data, ID, self)
