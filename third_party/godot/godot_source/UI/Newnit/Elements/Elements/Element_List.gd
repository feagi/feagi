extends Element_Base
class_name Element_List

const D_title = "NO TITLE GIVEN"
const D_usingCachedData = true

const DefaultHeaderElement := {
	"type": "button",
	"sideButtonText": "+",
	"text": "Update",
	"ID": "ListHeader"
}

const _specificSettableProps := {
	"usingCachedData": TYPE_BOOL,
	"title": TYPE_STRING
}

var title: String:
	get: return _ElementHeader.text
	set(v): _ElementHeader.text = v

var value: Array:
	get: return _List.Export()


var usingCachedData: bool:
	get: return _List.usingCachedData
	set(v): _List.usingCachedData = v

var prefabActivation: Dictionary

var _SplittingBox: BoxContainer
var _ElementHeader: Element_Button
var _List: List_Sub



func _ActivationSecondary(settings: Dictionary) -> void:
	if len(get_children()) > 1:
		@warning_ignore("assert_always_false")
		assert(false, "Please do not enable side buttons or label for list!")
	
	# Acquire References
	_SplittingBox = get_child(0)
	_ElementHeader = _SplittingBox.get_child(0)
	_List = _SplittingBox.get_child(1)
	
	# Activate button
	var buttonAct := DefaultHeaderElement
	buttonAct.merge({"text": HelperFuncs.GetIfCan(settings, "text", D_title)})
	_ElementHeader.Activate(buttonAct)
	
	# Activate List, even though its an element (it's special)
	prefabActivation = HelperFuncs.MustGet(settings, "prefabActivation")
	usingCachedData = HelperFuncs.GetIfCan(settings, "editable", D_usingCachedData)
	prefabActivation.merge({"count": 0})
	_List.Activate(prefabActivation, usingCachedData)
	
	_runtimeSettableProperties.merge(_specificSettableProps)

func _PopulateSubElements() -> Array:
	# used during Activation Primary to add Counter
	return ["list"]

func _getChildData() -> Dictionary:
	return {
		"value": value,
	}

func _DataUpProxy(_data) -> void:
	DataUp.emit({"value": true}, ID, self)
