extends Component
class_name Fill_Component

# Defaults, see readme
const DEF_USINGCUSTOMCOLOR = false
const DEF_COLOR = Color(0.0,0.0,0.0,0.0)


const ADDITIONAL_SETTABLE_PROPERTIES = {
	"usingCustomColor": TYPE_BOOL,
	"color": TYPE_VECTOR3I}

const TYPE: String = "Fill"

var usingCustomColor: bool:
	get: return _Panel.visible
	set(v): 
		_Panel.visible = v
var color: Color:
	get: return _Panel.customColor
	set(v): _Panel.customColor

var _Panel: Fill_SubComponent


# Activates the Counter
func _Activation(settings: Dictionary):
	self.name = "Button_" + ID
	_componentType = TYPE
	_runtimeSettableProperties.merge(ADDITIONAL_SETTABLE_PROPERTIES)
	_dataAvailable = false
	_dataSignalAvailable = false
	# Get References
	_Panel = $Panel
	
	# Fill in Inits
	usingCustomColor = HelperFuncs.GetIfCan(settings, 'usingCustomColor', DEF_USINGCUSTOMCOLOR)
	color = HelperFuncs.GetIfCan(settings, 'customColor', DEF_COLOR)
	
