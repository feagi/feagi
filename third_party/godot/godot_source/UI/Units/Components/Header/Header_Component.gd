extends Component
class_name Header_Component

# Defaults, see readme
const DEF_HEADER = "MISSING HEADER!"
const DEF_SIDE = 1
const ADDITIONAL_SETTABLE_PROPERTIES = {
	"label": TYPE_STRING,
	"side": TYPE_INT,
}

var label: String:
	get: return _Label.text
	set(v): 
		_Label.Htext = v
var side: int:
	get: return _side
	set(v):
		assert(((v == 0) or (v == 1) or (v == 2)), "Invalid Side Value!")
		_side = v

# Privates
const TYPE: String = "header"
var _Label: Label

var _side: int


# Sets up Activation
func _Activation(settings: Dictionary):
	self.name = "Header_" + ID
	_dataAvailable = false
	_dataSignalAvailable = false
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	
	# Get References
	_Label = $Label
	
	# Fill in Label
	label = HelperFuncs.GetIfCan(settings, "label", DEF_HEADER)
	side = HelperFuncs.GetIfCan(settings, "side", DEF_SIDE)
	
	

func _GetData():
	push_warning("Headers don't have values!")
	return "Invalid Header Value! You should not be seeing this!"

# UNFINISHED
func _SetData(data: Dictionary):
	pass
