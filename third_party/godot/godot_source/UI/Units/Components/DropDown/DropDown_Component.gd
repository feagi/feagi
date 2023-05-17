extends Component
class_name DropDown_Component

# Defaults, see readme
const DEF_LABEL = "MISSING LABEL!"
const DEF_EDITABLE = true
const DEF_HASBUTTON = false
const DEF_COUNTERSIZE = 50
const DEF_OPTIONS = []
const DEF_BUTTONTEXT = "+"
const DEF_INITINDEX = -1
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"buttonText": TYPE_STRING,
	"options": TYPE_ARRAY,
	"value": TYPE_STRING,
	"label": TYPE_STRING
}

const TYPE: String = "dropdown"

var hasButton: bool:
	get: return _hasButton
var buttonText: String:
	get:
		if(_hasButton): return _Button.text
		else: return ""
	set(v):
		if(_hasButton): _Button.text = v
var options: Array:
	get: return _DropDown.options
	set(v): 
		_DropDown.options = v
		# May Signal up if previously selected option is now unavilable
var value: String:
	get: return _DropDown.value
	set(v): 
		_DropDown.value = v
		# Causes a signal up
var label: String:
	get: return _Label.text
	set(v): 
		_Label.Htext = v


var _Label: Label
var _DropDown: OptionButton_SubComponent
var _Button: Button

var _hasButton: bool


# Sets-up the Counter
func _Activation(settings: Dictionary):
	self.name = "DropDown_" + ID
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	
	# Enable button if it is to be
	_hasButton = HelperFuncs.GetIfCan(settings, "hasButton", DEF_HASBUTTON)
	_Button = $Button
	if _hasButton:
		_Button.text = HelperFuncs.GetIfCan(settings, "buttonText", DEF_BUTTONTEXT)
		_Button.pressed.connect(ButtonPressed)
	else:
		# No button, delete it
		_Button.queue_free()
	
	
	# Get rest of References
	_Label = $Label
	_DropDown = $OptionButton
	
	# Fill in Label and Dropdown Inits
	label = HelperFuncs.GetIfCan(settings, "label", DEF_LABEL)
	options = HelperFuncs.GetIfCan(settings, "options", DEF_OPTIONS)
	
	# Connect
	_DropDown.item_selected.connect(OptionSelected)

# Used to button down
func ButtonPressed():
	if !_isActivated: return # avoid feedback when starting up
	DataUp.emit({"button": true, "type": TYPE, "ID": ID}, self)

func OptionSelected(selectedIndex: int):
	if !_isActivated: return # avoid feedback when starting up
	var selection: String = _DropDown.GetStringFromIndex(selectedIndex)
	DataUp.emit({"selected" : selection, "type": TYPE, "ID": ID}, self)

func _GetData():
	return value

# UNFINISHED
func _SetData(data: Dictionary):
	pass
