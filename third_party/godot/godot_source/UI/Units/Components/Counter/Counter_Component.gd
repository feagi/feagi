extends Component
class_name Counter_Component

# Defaults, see readme
const DEF_LABEL = "MISSING LABEL!"
const DEF_PLACEHOLDER = 0
const DEF_EDITABLE = true
const DEF_COUNTERWIDTH = 100.0
const DEF_COUNTERSIZE = 50
const DEF_ISINT = false
const DEF_PREFIX = ""
const DEF_SUFFIX = ""
const DEF_MAX = 1000.0
const DEF_MIN = -1000.0
const DEF_INTERVAL = 1
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"isInt": TYPE_BOOL,
	"editable": TYPE_BOOL,
	"prefix": TYPE_STRING,
	"suffix": TYPE_STRING,
	"maxValue": TYPE_FLOAT,
	"minValue": TYPE_FLOAT,
	"interval": TYPE_FLOAT,
	"value": TYPE_FLOAT,
	"label": TYPE_STRING,
	"counterWidth": TYPE_FLOAT,
}

const TYPE: String = "counter"

var isInt: bool:
	get: return _Counter.rounded
	set(v): _Counter.rounded = v
var editable: bool:
	get: return _Counter.editable
	set(v): _Counter.editable = v
var prefix: String:
	get: return _Counter.prefix
	set(v): _Counter.prefix = v
var suffix: String:
	get: return _Counter.suffix
	set(v): _Counter.suffix = v
var maxValue: float:
	get: return _Counter.max_value
	set(v): _Counter.max_value = v
var minValue: float:
	get: return _Counter.min_value
	set(v): _Counter.min_value = v
var interval: float:
	get: return _Counter.step
	set(v): _Counter.step = v
var value: float:
	get: return _Counter.value
	set(v): 
		_Counter.value = v
		# In the specific case of spinbox, changing value by code also
		# emits the internal signal
var label: String:
	get: return _Label.text
	set(v): 
		_Label.Htext = v
var counterWidth: float:
	get: return _Counter.size.x
	set(v):
		_Counter.Hsize.x = v

var _Label: Label_SubComponent
var _Counter: SpinBox


# Activates the Counter
func _Activation(settings: Dictionary):
	self.name = "Counter_" + ID
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	
	# Get References
	_Label = $Label
	_Counter = $SpinBox
	
	# Signal connections
	_Counter.value_changed.connect(ProxyValueChanges)
	
	# Fill in Spinbox Inits
	isInt = HelperFuncs.GetIfCan(settings, "isInt", DEF_ISINT)
	editable = HelperFuncs.GetIfCan(settings, 'editable', DEF_EDITABLE)
	prefix = HelperFuncs.GetIfCan(settings, 'prefix', DEF_PREFIX)
	suffix = HelperFuncs.GetIfCan(settings, 'suffix', DEF_SUFFIX)
	maxValue = HelperFuncs.GetIfCan(settings, 'maxValue', DEF_MAX)
	minValue = HelperFuncs.GetIfCan(settings, 'minValue', DEF_MIN)
	interval = HelperFuncs.GetIfCan(settings, 'interval', DEF_INTERVAL)
	_Counter.value = HelperFuncs.GetIfCan(settings, 'value', DEF_INTERVAL)
	
	# Init Label
	label = HelperFuncs.GetIfCan(settings, "label", DEF_LABEL)

# Used to proxy user value changes
func ProxyValueChanges(newValue: float):
	if !_isActivated: return # avoid feedback when starting up
	DataUp.emit({"number": newValue, "type": TYPE, "ID": ID}, self)

func _GetData():
	return value


