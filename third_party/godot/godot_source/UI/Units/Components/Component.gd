extends Panel
class_name Component

# Master class for all Components. Handles calculations that all must do (height and width) and
# storing their catagory

#TODO signal up if the sizing is changed within the component itself
#TODO add way to widen a component temporarily for better appearences

# Defaults
const DEF_PADDING: Vector2 = Vector2(10.0, 6.0)
const DEF_ISHORIZONTAL: bool = true

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
var Hsize: Vector2:
	get: return size
	set(v): RequestSizeChange(v)
var alignmentHeight: REF.HeightAlignmentSide:
	get: return _heightAlignment
	set(v):
		_heightAlignment = v
		_RepositionChildren(Hsize)
var alignmentWidth: REF.WidthAlignmentSide:
	get: return _widthAlignment
	set(v):
		_widthAlignment = v
		_RepositionChildren(Hsize)


#Private Vars
var _isActivated: bool = false
var _padding: Vector2
var _minDimensions: Vector2 = Vector2(0.0, 0.0) # preinit for func comparison calls
var _isHorizontal: bool
var _componentType: String # is filled in by the specific component activation
var _ID: String # Constant identification for the component
var _dataAvailable: bool = true # set false in activations of components without data
var _dataSignalAvailable: bool = true # ditto
var _widthAlignment := REF.WidthAlignmentSide.CENTER
var _heightAlignment := REF.HeightAlignmentSide.CENTER
var _runtimeSettableProperties = {
	"Hsize": TYPE_VECTOR2,
	"padding": TYPE_VECTOR2,
	"alignmentHeight": REF.HeightAlignmentSide,
	"alignmentWidth": REF.WidthAlignmentSide
}
# used to prevent multiple subcomponents from spamming requests all at once
var _requestingSizeChange: bool = false


# Call to ensure initialization with correct settings
func Activate(settings: Dictionary) -> void:
	if(_isActivated): return
	_ID = HelperFuncs.MustGet(settings, "ID")
	_padding = HelperFuncs.GetIfCan(settings, "padding", DEF_PADDING)
	_isHorizontal = HelperFuncs.GetIfCan(settings, "isHorizontal", DEF_ISHORIZONTAL)
	
	_requestingSizeChange = true # prevent spam requests to resize
	
	_Activation(settings) # activate specific component settings via virtual func

	_isActivated = true # prevent multiple activations
	
	call_deferred("UpdateSizeData") # apply resizing requests
	
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
		newSize = HelperFuncs.ClampVector2ToLargestAllowed(newSize, _minDimensions)
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
	SizeChanged.emit(self)

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
			if child.size.x > calWidth:
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
			if child.size.y > calHeight:
				calHeight = child.Hsize.y
		return calHeight + padding.y

# General method for repositioning children
# Writing this function was pain
func _RepositionChildren(parentSize: Vector2) -> void:
	var children = get_children()
	var childSizes: Array = []
	
	for child in children:
		childSizes.append(child.Hsize)
	
	# total child sizes is the same as parent minSize
	
	var totalGap: float
	var betweenGap: float
	var PreviousValue: float
	var otherDimension: float
	
	if isHorizontal:
		# Horizontal
		totalGap = parentSize.x - minimumSize.x
		betweenGap = totalGap / float(childSizes.size() + 1)
		
		for i in childSizes.size():
			
			otherDimension = (minimumSize.y - childSizes[i].y) / 2.0
			
			match(_widthAlignment):
				REF.WidthAlignmentSide.LEFT:
					if (i == 0): 
						PreviousValue = padding.x / 2.0
					else:
						PreviousValue = PreviousValue + childSizes[i - 1].x
					children[i].position = Vector2(PreviousValue, otherDimension)
						
				REF.WidthAlignmentSide.RIGHT:
					if (i == 0): 
						PreviousValue = totalGap - (padding.x / 2.0)
					else:
						PreviousValue = PreviousValue + childSizes[i - 1].x
					children[i].position = Vector2(PreviousValue, otherDimension)
						
				REF.WidthAlignmentSide.CENTER:
					if (i == 0): 
						PreviousValue = betweenGap + (padding.x / 2.0)
					else:
						PreviousValue = PreviousValue + childSizes[i - 1].x + betweenGap
					children[i].position = Vector2(PreviousValue, otherDimension)
	
	else:
		# Vertical
		totalGap = parentSize.y - minimumSize.y
		betweenGap = totalGap / float(childSizes.size() + 1)
		
		for i in childSizes.size():
			
			otherDimension = (minimumSize.x - childSizes[i].x) / 2.0
			
			match(_heightAlignment):
				REF.HeightAlignmentSide.TOP:
					if (i == 0): 
						PreviousValue = 0.0
					else:
						PreviousValue = PreviousValue + childSizes[i - 1].y
					children[i].position = Vector2(otherDimension, PreviousValue)
						
				REF.eightAlignmentSide.BOTTOM:
					if (i == 0): 
						PreviousValue = totalGap
					else:
						PreviousValue = PreviousValue + childSizes[i - 1].y
					children[i].position = Vector2(otherDimension, PreviousValue)
						
				REF.HeightAlignmentSide.CENTER:
					if (i == 0): 
						PreviousValue = betweenGap
					else:
						PreviousValue = PreviousValue + childSizes[i - 1].y + betweenGap
					children[i].position = Vector2(otherDimension, PreviousValue)

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

