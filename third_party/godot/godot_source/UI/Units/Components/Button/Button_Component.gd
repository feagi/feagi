extends Component
class_name Button_Component

# Defaults, see readme
const DEF_LABEL = "MISSING LABEL!"
const DEF_EDITABLE = true
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"label": TYPE_STRING,
	"editable": TYPE_BOOL}

const TYPE: String = "Button"

var editable: bool:
	get: return _Button.editable
	set(v): _Button.editable = v
var label: String:
	get: return _Button.Htext
	set(v): 
		_Button.Htext = v

var _Button: Button_SubComponent


# Activates the Counter
func _Activation(settings: Dictionary):
	self.name = "Button_" + ID
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	
	# Get References
	_Button = $button
	
	# Signal connections
	_Button.pressed.connect(ProxyValueChanges)
	
	# Fill in Button Inits
	editable = HelperFuncs.GetIfCan(settings, 'editable', DEF_EDITABLE)
	
	# Init Label
	label = HelperFuncs.GetIfCan(settings, "label", DEF_LABEL)

# Used to proxy user value changes
func ProxyValueChanges():
	if !_isActivated: return # avoid feedback when starting up
	DataUp.emit({"type": TYPE, "ID": ID}, self)
