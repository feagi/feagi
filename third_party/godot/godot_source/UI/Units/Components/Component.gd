extends Panel
class_name Component

# Master class for all Components. Handles calculations that all must do (height and width) and
# storing their catagory

#TODO signal up if the sizing is changed within the component itself
#TODO add way to widen a component temporarily for better appearences

# Defaults
const DEF_PADDING: Vector2 = Vector2(10.0, 6.0)
const DEF_ISHORIZONTAL: bool = true
const DEF_WIDTHALIGNMENT = 1
const DEF_HEIGHTALIGNMENT = 1
const DEF_HSIZE = Vector2(0.0, 0.0) # Force scaling up
const DEF_VISIBILITY = 0

# Signals
signal DataUp(customData: Dictionary, changedObjectReference)
signal SizeChanged(selfReference)

#public Vars
var componentType: String:
	get: return _componentType
var ID: String:
	get: return _ID
var isHorizontal: bool:
	get: return _isHorizontal
var dataAvailable: bool:
	get: return _dataAvailable
var dataSignalAvailable: bool:
	get: return _dataSignalAvailable
var data: # easier to use, read only verison of value
	get: return _GetData()
var minimumSize: Vector2:
	get: return _minDimensions
var padding: Vector2:
	get: return _padding
	set(v): 
		_padding = v
		UpdateSizeData(true)
var paddingX: float:
	get: return padding.x
	set(v):
		padding = Vector2(v, padding.y)
var paddingY: float:
	get: return padding.y
	set(v):
		padding = Vector2(padding.x, v)
var Hsize: Vector2:
	get: 
		if visibility != 2: return size
		return Vector2(0.0, 0.0)
	set(v): 
		RequestSizeChange(v)
var HsizeX: float:
	get: return Hsize.x
	set(v):
		Hsize = Vector2(v, Hsize.y)
var HsizeY: float:
	get: return Hsize.y
	set(v):
		Hsize = Vector2(Hsize.x, v)
var heightAlignment: int:
	get: return _heightAlignment
	set(v):
		_heightAlignment = v
		_RepositionChildren(Hsize)
var widthAlignment: int:
	get: return _widthAlignment
	set(v):
		_widthAlignment = v
		_RepositionChildren(Hsize)
var visibility: int:
	get: return _visibility
	set(v): _UpdateVisibility(v)

#Private Vars
var _isActivated: bool = false
var _padding: Vector2
var _minDimensions: Vector2 = Vector2(0.0, 0.0) # preinit for func comparison calls
var _isHorizontal: bool
var _componentType: String # is filled in by the specific component activation
var _ID: String # Constant identification for the component
var _dataAvailable: bool = true # set false in activations of components without data
var _dataSignalAvailable: bool = true # ditto
var _widthAlignment: int
var _heightAlignment: int
var _runtimeSettableProperties = {
	"Hsize": TYPE_VECTOR2,
	"padding": TYPE_VECTOR2,
	"widthAlignment": TYPE_INT,
	"heightAlignment": TYPE_INT
}
var _initialSize: Vector2
var _visibility: int # 0 is normal visibile, 1 is invisible, 2 is hidden

# used to prevent multiple subcomponents from spamming requests all at once
var _requestingSizeChange: bool = false


# Call to ensure initialization with correct settings
func Activate(settings: Dictionary) -> void:
	if(_isActivated): return
	_AddSizeChangeSignals()
	_ID = HelperFuncs.MustGet(settings, "ID")
	_padding = HelperFuncs.GetIfCan(settings, "padding", DEF_PADDING)
	_isHorizontal = HelperFuncs.GetIfCan(settings, "isHorizontal", DEF_ISHORIZONTAL)
	_widthAlignment = HelperFuncs.GetIfCan(settings, "widthAlignment", DEF_WIDTHALIGNMENT)
	_heightAlignment = HelperFuncs.GetIfCan(settings, "heightAlignment", DEF_HEIGHTALIGNMENT)
	
	_Activation(settings) # activate specific component settings via virtual func

	_isActivated = true # prevent multiple activations
	
	
		# init size
	_initialSize = HelperFuncs.LoadMostDefaultV2(settings, "Hsize", DEF_HSIZE)
	call_deferred("_InitInitialSize")
	
	return
	


# Instead of applying properties through the object, instead apply them through a dict
func ApplyPropertiesFromDict(dict: Dictionary) -> void:
	for inputKey in dict.keys():
		if !(inputKey in _runtimeSettableProperties.keys()):
			# Key is not a valid settable property
			print("Setting ", inputKey, " not a valid settable property for component ",
				ID, " of type ", componentType)
			continue
		
		
#		print(dict)
		var a = dict[inputKey]
#		print(typeof(a))
		self[inputKey] = dict[inputKey] # This is completely and utterly cursed
		# Why do you refer the object to inputkey? When dict[inputkey] is changed, self[input]
		# will be also changed. - ko

####################################
########## Internal Only ###########
####################################

func _AddSizeChangeSignals() -> void:
	var children = get_children()
	for child in children:
		var signalList = child.get_signal_list()
		
		for foundSignal in signalList:
			if foundSignal["name"] == "SizeChanged":
				child.connect("SizeChanged", _RecieveSizeChangeNotificationFromBelow)



####################################
########## Sizing Systems ##########
####################################

# Can be called externally from above to request a size change.
# Returns false if requested size smaller than min size
# If requested size is smaller than allowed size, it will be expanded
func  RequestSizeChange(newSize: Vector2) -> bool:
	var notResized := true
	if (HelperFuncs.IsVector2SmallerInAnyDim(newSize, _minDimensions)):
		newSize = HelperFuncs.GrowVector2ToSmallestAllowed(newSize, _minDimensions)
		notResized = false
	_RepositionChildren(newSize)
	size = newSize
	return notResized

# Similar to Request Size Change, but forces size expansion to at least the
# minimum allowed size instead of something user defined. Can also be optionally
# forced to run size / child positioning updates regardless
func UpdateSizeData(forceUpdate: bool = false) -> void:
	_requestingSizeChange = false # allow reset next frame
	if(_UpdateMinimumDimensions() || forceUpdate):
		Hsize = HelperFuncs.GrowVector2ToSmallestAllowed(Hsize, _minDimensions)
	_RepositionChildren(Hsize)
	SizeChanged.emit(self)

# Call deffered on activation to init all sizes in the correct order
# This is not particuarly efficient. Too Bad!
func _InitInitialSize() -> void:
	_UpdateMinimumDimensions()
	RequestSizeChange(_initialSize)

# This function relays signal input through a deffered call.
# This allows default Godot UI resizing behvior to apply and for us to read it
# rapidly afterwards.
# This is not particuarly efficient. Too bad!
func _RecieveSizeChangeNotificationFromBelow(_objectReference) -> void:
	if _requestingSizeChange: return # avoid repitition
	_requestingSizeChange = true
	call_deferred("UpdateSizeData") # I am sure this won't cause problems /s

# Forces a recalculation of minimum required dimensions. If minimum required
# size is bigger than the current size, returns true and updates minimum dim.
# STILL REQUIRED TO TAKE ACTION
func _UpdateMinimumDimensions() -> bool:
	var newMinDim: Vector2 = Vector2(_GetMinWidth(), _GetMinHeight())
	if (HelperFuncs.IsVector2SmallerInAnyDim(_minDimensions, newMinDim )):
		_minDimensions = HelperFuncs.GrowVector2ToSmallestAllowed(_minDimensions, newMinDim)
		return true
	return false

# Gets the minimum allowed width of the Component
func _GetMinWidth() -> float:
	var children = get_children()
	var calWidth: float = 0.0
	if _isHorizontal:
		calWidth = padding.x # assume small buffer on either side
		for child in children:
			calWidth += child.Hsize.x
		return calWidth
	else:
		for child in children:
			if child.Hsize.x > calWidth:
				calWidth = child.Hsize.x
		return calWidth + padding.x

# Get Minimum allowed height of the Component
func _GetMinHeight() -> float:
	var children = get_children()
	var calHeight: float = 0.0
	
	if !_isHorizontal:
		calHeight = padding.y # assume small buffer on either size
		for child in children:
			calHeight += child.Hsize.y
		return calHeight
	else:
		for child in children:
			if child.Hsize.y > calHeight:
				calHeight = child.Hsize.y
		return calHeight + padding.y

# General method for repositioning children
# (Re) Writing this function was pain
func _RepositionChildren(parentSize: Vector2) -> void:
	var children = get_children()
	var childHSizes: Array = []; var childVSizes: Array = []
	
	for child in children:
		childHSizes.append(child.Hsize.x)
		childVSizes.append(child.Hsize.y)
	
	# total child sizes + padding is the same as parent minSize
	
	if isHorizontal:
		_RepositionChildren_H(parentSize, childHSizes, childVSizes, children)
	else:
		_RepositionChildren_V(parentSize, childHSizes, childVSizes, children)

func _RepositionChildren_H(parentSize: Vector2, childHs: Array, childVs: Array, children: Array):
	
	# Preallocate to reduce GC
	var gap: float
	var xPos: float; var yPos: float
	
	# special case when 1 child
	if children.size() == 1:
		children[0].position = Vector2(parentSize.x - childHs[0], parentSize.y - childVs[0]) / 2.0
		return
	
	match(widthAlignment):
		0: gap = 0.0
		1: gap = (parentSize.x - HelperFuncs.SumFloatArray(childHs)) / float(childHs.size() - 1)
		2: gap = (parentSize.x - HelperFuncs.SumFloatArray(childHs))
		
	
	for i in childHs.size():
		
		yPos = (parentSize.y - childVs[i]) / 2.0
		
		match(widthAlignment):
			0:
				xPos = HelperFuncs.SumFloatArrayAtIndex(childHs, i)
			1:
				xPos = HelperFuncs.SumFloatArrayAtIndex(childHs, i) + float(i * gap)
			2:
				xPos = HelperFuncs.SumFloatArrayAtIndex(childHs, i) + gap
		
		children[i].position = Vector2(xPos, yPos)

func _RepositionChildren_V(parentSize: Vector2, childHs: Array, childVs: Array, children: Array):
	
	# Preallocate to reduce GC
	var gap: float
	var xPos: float; var yPos: float

	# special case when 1 child
	if children.size() == 1:
		children[0].position = Vector2(parentSize.x - childHs[0], parentSize.y - childVs[0]) / 2.0
		return
	
	match(widthAlignment):
		0: gap = 0.0
		1: gap = (parentSize.y - HelperFuncs.SumFloatArray(childVs)) / float(childVs.size() - 1)
		2: gap = (parentSize.y - HelperFuncs.SumFloatArray(childVs))

	for i in childVs.size():
		
		xPos = (parentSize.x - childHs[i]) / 2.0
		
		match(widthAlignment):
			0:
				yPos = HelperFuncs.SumFloatArrayAtIndex(childVs, i)
			1:
				yPos = HelperFuncs.SumFloatArrayAtIndex(childVs, i) + float(i * gap)
			2:
				yPos = HelperFuncs.SumFloatArrayAtIndex(childVs, i) + gap
		
		children[i].position = Vector2(xPos, yPos)
 
# Update Visibility of a component. Relies on check in Hsize property
func _UpdateVisibility(newVisibility: int) -> void:
	
	if newVisibility == 0:
		visible = true
		for child in get_children():
			child.visible = true
	else:
		visible = false
		for child in get_children():
			child.visible = false

	if _visibility == 2 or newVisibility == 2: SizeChanged.emit(self)
	_visibility = newVisibility

####################################
############ Overrides #############
####################################

# these do nothing, all inheriting nodes must override these!

func _Activation(_settings: Dictionary):
	@warning_ignore("assert_always_false")
	assert(false, "_Activation function not overriden correctly!")

func _GetData():
	@warning_ignore("assert_always_false")
	assert(false, "_GetData function not overriden correctly!")

