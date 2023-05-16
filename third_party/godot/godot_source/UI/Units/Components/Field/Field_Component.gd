extends Component
class_name Field_Component

#TODO Refactor this to make it more clean like Counter_Component

# Defaults, see readme
const DEF_LABEL = "MISSING LABEL!"
const DEF_PLACEHOLDER = ""
const DEF_EDITABLE = true
const DEF_FIELDWIDTH = 100.0
const DEF_MAXCHARACTERS = 50
const DEF_ONLYTRIGGERWITHENTER = true
const DEF_VALUE = ""
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"editable": TYPE_BOOL,
	"value": TYPE_STRING,
	"label": TYPE_STRING,
	"fieldWidth": TYPE_FLOAT,
	"maxCharacters": TYPE_INT,
	"placeHolder": TYPE_STRING,
	"onlyTriggerWithEnter": TYPE_BOOL
}

const TYPE: String = "field"

var editable: bool:
	get: return _LineEdit.editable
	set(v): _LineEdit.editable = v
var label: String:
	get: return _Label.Htext
	set(v): 
		_Label.Htext = v
var maxCharacters: int:
	get: return _LineEdit.max_length
	set(v): _LineEdit.max_length = v
var fieldWidth: float:
	get: return _LineEdit.fieldWidth
	set(v): _LineEdit.fieldWidth = v
var placeHolder: String:
	get: return _LineEdit.placeholder_text
	set(v): _LineEdit.placeholder_text = v
var value: String:
	get: return _LineEdit.text
	set(v): _LineEdit.text = v
var onlyTriggerWithEnter: bool:
	get: return _triggerOnlyWithEnter
	# No Set!


# Privates
var _Label: Label
var _LineEdit: LineEdit_SubComponent

var _triggerOnlyWithEnter: bool



# Sets-up the Field
func _Activation(settings: Dictionary):
	self.name = "Field_" + ID
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
	editable = HelperFuncs.GetIfCan(settings, "editable", DEF_EDITABLE)
	maxCharacters = HelperFuncs.GetIfCan(settings, "maxCharacters", DEF_MAXCHARACTERS)
	fieldWidth = HelperFuncs.GetIfCan(settings, "fieldWidth", DEF_FIELDWIDTH)
	value = HelperFuncs.GetIfCan(settings, "value", DEF_VALUE)


# Used to proxy user value changes
func ProxyValueChanges(newValue: String):
	if !_isActivated: return # avoid feedback when starting up
	DataUp.emit({"text": newValue, "type": TYPE, "ID": ID}, self)


func _GetData():
	return value

# UNFINISHED
func _SetData(data: Dictionary):
	pass
