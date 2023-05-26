extends Component
class_name Fill_Component

# Defaults, see readme
const DEF_USINGCUSTOMCOLOR = false
const DEF_COLOR = Vector3i(255, 255, 255)
const COLOR_OFFSET = Vector3i(255, 255, 255)


const ADDITIONAL_SETTABLE_PROPERTIES = {
	"usingCustomColor": TYPE_BOOL,
	"color": TYPE_VECTOR3I,
	"colorR": TYPE_INT,
	"colorG": TYPE_INT,
	"colorb": TYPE_INT}

const TYPE: String = "Fill"

var usingCustomColor: bool:
	get: return _usingCustomColor
	set(v): 
		_usingCustomColor = v
		if v:
			color = color # funny property shenanigans
		else:
			_Panel.resetColor()
var color: Vector3i:
	get: return HelperFuncs.ColorToV3I(_Panel.customColor)
	set(v): 
		if !usingCustomColor: return
		_Panel.customColor = HelperFuncs.V3IToColor(v)
		_usingCustomColor = true
var colorR: int:
	get: return color.x
	set(v): color = Vector3i(v, color.y, color.z)
var colorG: int:
	get: return color.y
	set(v): color = Vector3i(color.x, v, color.z)
var colorB: int:
	get: return color.z
	set(v): color = Vector3i(color.x, color.y, v)


var _Panel: Fill_SubComponent
var _usingCustomColor: bool

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
	color = HelperFuncs.LoadMostDefaultv3_Color(settings, 'color', DEF_COLOR)


func _GetData():
	return color
