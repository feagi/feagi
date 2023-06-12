extends BoxContainer
class_name Element_Base

# This base class is used to construct all Elements, which are parallel in 
# implmentation to Newnits but insteadof holding containers, are UI elements

######################## START Newnit Parallel - this section must match that of Newnit_Base ########################

const NEWNIT_CORE = preload("res://UI/Newnit/PreloadScripts/Newnit_core.gd")

signal DataUp(data, originatingID, originatingRef)

var ID: StringName:
	get: return _ID

var parent: Node:
	get: return get_node("../")

var parentID: StringName:
	get: return NEWNIT_CORE.Get_ParentID(self)

var childrenIDs: Array:
	get: return NEWNIT_CORE.Func_GetChildIDs(children)

var data: Dictionary:
	get: return NEWNIT_CORE.Get_data(self)

var type: StringName:
	get: return _type

var _ID: StringName
var _isActivated := false
var _isTopLevel := true
var _runtimeSettableProperties := NEWNIT_CORE.settableProperties
var _type: StringName

func Activate(settings: Dictionary) -> void:
	NEWNIT_CORE.Func_Activate(settings, self)

# Set Properties from dictionary
func SetData(input: Dictionary) -> void:
	NEWNIT_CORE.Func_SetData(input, self)

# Does an indepth search of all children by ID, and returns a node reference to
# a matching ID. If none are found, returns false
func GetReferenceByID(searchID: StringName): # returns either a bool or a Node
	NEWNIT_CORE.Func_GetReferenceByID(searchID, self)

################################################ END Newnit Parallel ################################################

### START Element Unique ###

const children := [] # elements do not have editable children

# Defaults
const D_alignment = 1
const D_vertical = false
const D_labelText = ""
const D_sideButtonText = ""
const D_sideButtonEditable = true

var sideLabelText: String:
	get: return _sideLabelText
	set(v):
		if(!_has_label): return
		_sideLabelText = v
		_sideLabel.text = v
var sideButtonText: String:
	get: return _sideButtonText
	set(v):
		if(!_has_button): return
		_sideButtonText = v
		_sideButton.text = v


var _has_label: bool
var _has_button: bool
var _sideLabelText: String
var _sideButtonText: String
var _sideLabel #:Header_Sub
var _sideButton #:Button_Sub
var _sideButtonEditable: bool

const settableProperties := {
	"alignment": TYPE_INT,
	"vertical": TYPE_INT
}

# Base Element Activation
func _ActivationPrimary(settings: Dictionary) -> void:
	
	
	var subComponents := _PopulateSubElements() 
	
	_runtimeSettableProperties.merge(settableProperties)
	
	_sideLabelText = HelperFuncs.GetIfCan(settings, "labelText", D_labelText)
	_sideButtonText = HelperFuncs.GetIfCan(settings, "sideButtonText", D_sideButtonText)
	
	_has_label = _sideLabelText != ""
	_has_button = _sideButtonText != ""
	
	alignment = HelperFuncs.GetIfCan(settings, "alignment", D_alignment)
	vertical = HelperFuncs.GetIfCan(settings, "vertical", D_vertical)
	
	if _has_label:
		subComponents.push_front("label")
	
	if _has_button: #TODO
		subComponents.push_back("sideButton")
		pass
	
	
	_SpawnSubElements(subComponents)
	_ActivationSecondary(settings)


func _SpawnSubElements(componentTypes: Array) -> void:
	
	for compType in componentTypes:
		var subComp
		match compType:
			"counter":
				subComp = Spinbox_Sub.new()
			"label":
				subComp = Label_Sub.new()
				_sideLabel = subComp
				subComp.name = NEWNIT_CORE.Func__GetUIChildName(compType, self)
				add_child(subComp)
				subComp.text = sideLabelText
				continue
			"sideButton":
				subComp = Button_Sub.new()
				_sideButton = subComp
				subComp.name = NEWNIT_CORE.Func__GetUIChildName(compType, self)
				add_child(subComp)
				subComp.text = sideButtonText
				subComp.pressed.connect(_SideButtonPressed)
				continue
			#TODO more types
			_:
				@warning_ignore("assert_always_false")
				assert(false, "Invalid Element Type " + str(compType))
		subComp.value_edited.connect(_DataUpProxy)
		subComp.name = NEWNIT_CORE.Func__GetUIChildName(compType, self)
		add_child(subComp)

func _SideButtonPressed() -> void:
	DataUp.emit({"sideButton": true}, ID, self)

# Other properties:
# https://docs.godotengine.org/en/stable/classes/class_boxcontainer.html
# alignment: int -> center children left/top, center, right/bottom
# vertical: bool

func _ActivationSecondary(_settings: Dictionary) -> void:
	@warning_ignore("assert_always_false")
	assert(false, "_ActivationSecondary function not overriden correctly!")


func _getChildData() -> Dictionary:
	@warning_ignore("assert_always_false")
	assert(false, "_getChildData function not overriden correctly!")	
	return {}

func _DataUpProxy(_data) -> void:
	@warning_ignore("assert_always_false")
	assert(false, "_DataUpProxy function not overriden correctly!")	

func _PopulateSubElements() -> Array:
	@warning_ignore("assert_always_false")
	assert(false, "_PopulateSubElements function not overriden correctly!")	
	return []
