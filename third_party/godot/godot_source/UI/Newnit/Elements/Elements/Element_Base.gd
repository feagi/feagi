extends BoxContainer
class_name Element_Base

# This base class is used to construct all Elements, which are parallel in 
# implmentation to Newnits but insteadof holding containers, are UI elements

######################## START Newnit Parallel - this section must match that of Newnit_Base ########################

const NEWNIT_CORE = preload("res://UI/Newnit/PreloadScripts/Newnit_core.gd")

signal DataUp(data: Dictionary, originatingID: StringName, originatingRef: Node)

var ID: StringName:
	get: return _ID

var parent: Node:
	get: return _parent

var parentID: StringName:
	get: return NEWNIT_CORE.Get_ParentID(self)

var childrenIDs: Array:
	get: return NEWNIT_CORE.Func_GetChildIDs(children)

var data: Dictionary:
	get: return NEWNIT_CORE.Get_data(self)

var type: StringName:
	get: return _type

var isUsingPanel: bool:
	get: return _isUsingPanel

var panelRef: Node:
	get: return panelRef

var hasNewnitParent: bool:
	get: return _hasNewnitParent

var _ID: StringName
var _isActivated := false
var _isTopLevel := true
var _runtimeSettableProperties := NEWNIT_CORE.settableProperties
var _type: StringName
var _isUsingPanel: bool
var _panelRef: Node = null
var _parent: Node = null
var _hasNewnitParent: bool = false

func Activate(settings: Dictionary) -> void:
	NEWNIT_CORE.Func_Activate(settings, self)

# Set Properties from dictionary
func SetData(input: Dictionary) -> void:
	NEWNIT_CORE.Func_SetData(input, self)

func GetReferenceByID(searchID: StringName): # returns either a bool or a Node
	if searchID == ID: return self
	for child in children:
		var result = child.GetReferenceByID(searchID)
		if typeof(result) != TYPE_BOOL:
			return child
	return false

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
var sideButtonEditable: bool:
	get: return _sideButtonEditable
	set(v):
		if(!_has_button): return
		_sideButtonEditable = v
		_sideButton.editable = v


var _has_label: bool
var _has_button: bool
var _sideLabelText: String
var _sideButtonText: String
var _sideLabel: Label_Sub
var _sideButton :Button_Sub
var _sideButtonEditable: bool

const settableProperties := {
	"alignment": TYPE_INT,
	"vertical": TYPE_INT,
	"sideLabelText": TYPE_STRING,
	"sideButtonText": TYPE_STRING,
	"sideButtonEditable": TYPE_BOOL}

# Base Element Activation
func _ActivationPrimary(settings: Dictionary) -> void:
	
	var subComponents := _PopulateSubElements() 

	_runtimeSettableProperties.merge(settableProperties)
	
	_sideLabelText = HelperFuncs.GetIfCan(settings, "sideLabelText", D_labelText)
	_sideButtonText = HelperFuncs.GetIfCan(settings, "sideButtonText", D_sideButtonText)
	
	_has_label = _sideLabelText != ""
	_has_button = _sideButtonText != ""
	
	alignment = HelperFuncs.GetIfCan(settings, "alignment", D_alignment)
	vertical = HelperFuncs.GetIfCan(settings, "vertical", D_vertical)
	
	if _has_label:
		subComponents.push_front("sideLabel")
	
	if _has_button:
		subComponents.push_back("sideButton")
	_SpawnSubElements(subComponents)
	_ActivationSecondary(settings)


func _SpawnSubElements(componentTypes: Array) -> void:
	
	for compType in componentTypes:
		var subComp
		match compType:
			"sideLabel":
				subComp = Label_Sub.new()
				_sideLabel = subComp
				subComp.name = NEWNIT_CORE.Func__GetUIChildName(compType, self)
				_childRoot.add_child(subComp)
				subComp.text = sideLabelText
				continue
			"sideButton":
				subComp = Button_Sub.new()
				_sideButton = subComp
				subComp.name = NEWNIT_CORE.Func__GetUIChildName(compType, self)
				_childRoot.add_child(subComp)
				subComp.text = sideButtonText
				subComp.pressed.connect(_SideButtonPressed)
				continue
			"vector3":
				for i in range(3):
					subComp = BoxContainer.new(); _childRoot.add_child(subComp)
					subComp.add_child(Label_Sub.new())
					subComp.add_child(LineEdit_ff_Sub.new())
				continue
			"list":
				subComp = BoxContainer.new(); _childRoot.add_child(subComp)
				subComp.add_child(Element_Button.new())
				subComp.add_child(List_Sub.new())
				continue
			"button": subComp = Button_Sub.new()
			"counter": subComp = Spinbox_Sub.new()
			"checkBox": subComp = CheckBox_Sub.new()
			"checkButton": subComp = CheckButton_Sub.new()
			"dropDown": subComp = OptionButton_Sub.new()
			"field": subComp = LineEdit_Sub.new()
			"floatField": subComp = LineEdit_ff_Sub.new()
			"label": subComp = Label_Sub.new()
			# More Types!
			_:
				@warning_ignore("assert_always_false")
				assert(false, "Invalid Element Type " + str(compType))
		subComp.value_edited.connect(_DataUpProxy)
		subComp.name = NEWNIT_CORE.Func__GetUIChildName(compType, self)
		_childRoot.add_child(subComp)

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
