extends Component
class_name FloatField_Component

# Defaults, see readme
const DEF_LABEL = "MISSING"
const DEF_PLACEHOLDER = "1.0"
const DEF_EDITABLE = true
const DEF_FIELDWIDTH = 100.0
const DEF_MAXCHARACTERS = 8
const DEF_ONLYTRIGGERWITHENTER = true
const DEF_VALUE = "0.0"
const DEF_SCALEWITHINPUTTEXT = false
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"editable": TYPE_BOOL,
	"value": TYPE_STRING,
	"label": TYPE_STRING,
	"fieldWidth": TYPE_FLOAT, 
	"maxCharacters": TYPE_INT,
	"placeHolder": TYPE_STRING,
	"onlyTriggerWithEnter": TYPE_BOOL,
	"scaleWithInputText": TYPE_BOOL
}

const TYPE: String = "floatField"

var editable: bool:
	get: return _LineEdit.editable
	set(v): _LineEdit.editable = v
var label: String:
	get: return _Label.Htext
	set(v): 
		_Label.Htext = v
var fieldWidth: float:
	get: return _LineEdit.fieldWidth
	set(v): _LineEdit.fieldWidth = v
var placeHolder: String:
	get: return _LineEdit.placeholder_text
	set(v): _LineEdit.placeholder_text = v
var value: float:
	get: return _LineEdit.HFloat
	set(v): _LineEdit.UpdateHFloat(v) # note, if value isnt a float, it will be ignored!
var onlyTriggerWithEnter: bool:
	get: return _triggerOnlyWithEnter
	# No Set!
var textWidth: float:
	get: return _LineEdit.textWidth
var scaleWithInputText: bool:
	get: return _LineEdit.shouldScaleWithInputText
	set(v): _LineEdit.shouldScaleWithInputText = v


# Privates
var _Label: Label
var _LineEdit: LineEdit_ff_SubComponent

var _triggerOnlyWithEnter: bool


# Sets-up the FLoatField
func _Activation(settings: Dictionary):
	self.name = "FloatField_" + ID
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	
	# Get References
	_Label = $Label
	_LineEdit = $LineEdit
	
	# Signaling
	_triggerOnlyWithEnter = HelperFuncs.GetIfCan(settings, "onlyTriggerWithEnter", DEF_ONLYTRIGGERWITHENTER)
	if(_triggerOnlyWithEnter):
		_LineEdit.text_submitted.connect(ProxyValueChanges)
	else:
		_LineEdit.text_changed.connect(ProxyValueChanges)
	
	# Fill in Label and Field Inits
	label = HelperFuncs.GetIfCan(settings, "label", DEF_LABEL)
	placeHolder = HelperFuncs.GetIfCan(settings, "placeHolder", DEF_PLACEHOLDER)
	fieldWidth = HelperFuncs.GetIfCan(settings, "fieldWidth", DEF_FIELDWIDTH)
	value = HelperFuncs.GetIfCan(settings, "value", DEF_VALUE)
	scaleWithInputText = HelperFuncs.GetIfCan(settings, "scaleWithInputText", DEF_SCALEWITHINPUTTEXT)


# Used to proxy user value changes
func ProxyValueChanges(newValue: String):
	if !_isActivated: return # avoid feedback when starting up
	DataUp.emit({"text": newValue, "type": TYPE, "ID": ID}, self)


func _GetData():
	return value
