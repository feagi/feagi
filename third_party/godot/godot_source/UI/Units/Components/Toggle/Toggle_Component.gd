extends Component
class_name Toggle_Component


# Defaults, see readme
const DEF_LABEL = "MISSING LABEL!"
const DEF_EDITABLE = true
const DEF_ISCHECKBOX = false
const DEF_VALUE = false
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"editable": TYPE_BOOL,
	"value": TYPE_BOOL,
	"label": TYPE_STRING,
}

const TYPE: String = "toggle"


var value: bool:
	get: return _Toggle.button_pressed
	set(v): _Toggle.button_pressed = v
var isCheckBox: bool:
	get: return _isCheckBox
	# No Set!
var label: String:
	get: return _Label.Htext
	set(v): 
		_Label.Htext = v
var editable: bool:
	get: return !_Toggle.disabled
	set(v): _Toggle.disabled = !v


var _Label: Label
var _Toggle # can be checkbox or checkbutton

var _isCheckBox: bool



# Sets-up the Counter
func _Activation(settings: Dictionary):
	self.name = "Toggle_" + ID
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	
	# Differentiate between checkbox and checkbutton
	var nodeToDelete
	if HelperFuncs.GetIfCan(settings, "isCheckBox", DEF_ISCHECKBOX):
		# checkbox
		nodeToDelete = $CheckButton
		_Toggle = $CheckBox
	else:
		# checkbutton
		nodeToDelete = $CheckBox
		_Toggle = $CheckButton
	nodeToDelete.queue_free() # delete unneeded node
	
	# Get References
	_Label = $Label
	
	# Fill in Label and Toggle Inits
	editable = HelperFuncs.GetIfCan(settings, "editable", DEF_EDITABLE)
	label = HelperFuncs.GetIfCan(settings, "label", DEF_LABEL)
	value = HelperFuncs.GetIfCan(settings, "value", DEF_VALUE)

# Used to proxy user value changes
func ProxyValueChanges(newValue: bool):
	if !_isActivated: return # avoid feedback when starting up
	DataUp.emit({"state": newValue, "type": TYPE, "ID": ID}, self)



func _GetData():
	return value

# UNFINISHED
func _SetData(data: Dictionary):
	pass
