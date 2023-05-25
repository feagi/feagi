extends Panel
class_name Unit

# Base class all Units inherent from

const DEF_PADDING: Vector2 = Vector2(5.0,5.0)
const DEF_ISVERTICAL: bool = true
const DEF_SPAWNPOINT: Vector2 = Vector2(0.0, 0.0)
const DEF_POSITION = Vector2(0.0,0.0)
const DEF_ISSUBUNIT = false
const DEF_TITLEBARTITLE = "UNNAMED TITLE"
const DEF_ENABLETITLEBAR = false
const DEF_ENABLECLOSEBUTTON = false
const DEF_WIDTHALIGNMENT = 1
const DEF_HEIGHTALIGNMENT = 1

var ID: String:
	get: return _ID
var isHorizontal: bool:
	get: return _isHorizontal
var padding: Vector2:
	get: return _padding
	set(v):
		_padding = v
		UpdateSizeData(true)
var componentsSpawnPoint: Vector2:
	get: return _componentsSpawnPoint
var minimumSize: Vector2:
	get: return _minDimensions
var componentData: Dictionary:
	get: return _GetComponentsData()
	set(v): _SetComponentsData(v)
var componentIDs: Array:
	get: return _GetArrayOfCompIDs()
var componentRefs: Dictionary:
	get: return _GetComponentReferencesByID()
var Hsize: Vector2:
	get: return size
	set(v): RequestSizeChange(v)
var dataSignalAvailable: bool:
	get: return _dataSignalAvailable
var isSubUnit: bool:
	get: return _isSubUnit
var widthAlignment: int:
	get: return _widthAlignment
	set(v):
		_widthAlignment = v
		_RepositionChildren(Hsize)
var heightAlignment: int:
	get: return _heightAlignment
	set(v):
		_heightAlignment = v
		_RepositionChildren(Hsize)

signal DataUp(customData: Dictionary, compRef, unitRef)
signal SizeChanged(selfRef)
signal UserClosedUnit(ID: String, cause: Dictionary)

var _componentsDicts: Array
var _components: Array
var _isHorizontal: bool
var _padding: Vector2
var _componentsSpawnPoint: Vector2
var _minDimensions: Vector2
var _ID: String
var _dataSignalAvailable: bool = true
var _isSubUnit: bool

# used to prevent multiple components from spamming requests all at once
var _requestingSizeChange: bool = false
var _widthAlignment: int
var _heightAlignment: int 

var _fieldScene: PackedScene = preload("res://UI/Units/Components/Field/field.tscn")
var _counterScene: PackedScene = preload("res://UI/Units/Components/Counter/counter.tscn")
var _toggleScene: PackedScene = preload("res://UI/Units/Components/Toggle/Toggle.tscn")
var _dropdownScene: PackedScene = preload("res://UI/Units/Components/DropDown/DropDown.tscn")
var _headerScene: PackedScene = preload("res://UI/Units/Components/Header/header.tscn")
var _buttonScene: PackedScene = preload("res://UI/Units/Components/Button/button.tscn")
var _unitScene: PackedScene = preload("res://UI/Units/unit.tscn")

# Setup Unit for use
func Activate(activationDict : Dictionary):
	
	# Init Vars
	_ID = HelperFuncs.MustGet(activationDict, "ID")
	_componentsDicts = HelperFuncs.MustGet(activationDict, "components")

	_componentsSpawnPoint = HelperFuncs.GetIfCan(activationDict, "componentSpawnPoint", DEF_SPAWNPOINT)
	position = HelperFuncs.GetIfCan(activationDict, "position", DEF_POSITION) #TODO some units cannot set their own pos
	_padding = HelperFuncs.GetIfCan(activationDict, "padding", DEF_PADDING)
	_isHorizontal = !HelperFuncs.GetIfCan(activationDict, "isVertical", DEF_ISVERTICAL)
	_isSubUnit = HelperFuncs.GetIfCan(activationDict, "isSubUnit", DEF_ISSUBUNIT)
	_widthAlignment = HelperFuncs.GetIfCan(activationDict, 'widthAlignment', DEF_WIDTHALIGNMENT)
	_heightAlignment = HelperFuncs.GetIfCan(activationDict, 'heightAlignment', DEF_HEIGHTALIGNMENT)
	
	
	# title bar stuff
	if HelperFuncs.GetIfCan(activationDict, "enableTitleBar", DEF_ENABLETITLEBAR):
		# title bar is enabled
		# TODO: This is dumb.
		
		var titleBarTitle: String = HelperFuncs.GetIfCan(activationDict, "titleBarTitle", DEF_TITLEBARTITLE)
		var enableCloseButton: bool = HelperFuncs.GetIfCan(activationDict, "enableCloseButton", DEF_ENABLECLOSEBUTTON)
		var currentLang: String = get_node("/root/Official_World/Core/GlobalUISystem").currentLanguageISO
		var titleBarActivationDict = {
			"ID": "TITLEBAR",
			"isVertical": false,
			"type": "unit",
			"isSubUnit": true
		}
		
		
		if enableCloseButton:
			titleBarActivationDict.merge( 
			{"components": 
				[
					{"type": "header", "ID": "TITLE", "label": titleBarTitle},
					{"type": "button", "ID": "CLOSEBUTTON", "label": "x"}
				]}
			)
		else:
			titleBarActivationDict.merge( 
			{"components": 
				[
					{"type": "header", "ID": "TITLE", "label": titleBarTitle}
				]}
			)
		_componentsDicts.push_front(titleBarActivationDict)
	
	
	_requestingSizeChange = true # avoid resize spam
	
	AddMultipleComponents(_componentsDicts)
	
	
	call_deferred("UpdateSizeData") # apply resizing requests
	
	


# Handles Spawning of components one at a time
func AddComponent(component: Dictionary) -> void:
	
	var newComponent
	match component["type"]:
		"field":
			newComponent = _fieldScene.instantiate()
		"counter":
			newComponent = _counterScene.instantiate()
		"toggle":
			newComponent = _toggleScene.instantiate()
		"dropdown":
			newComponent = _dropdownScene.instantiate()
		"header":
			newComponent = _headerScene.instantiate()
		"button":
			newComponent = _buttonScene.instantiate()
		"unit":
			newComponent = _unitScene.instantiate()

	# Add the new Component to the Unit, Activate it, Connect Signals, Store a Reference
	add_child(newComponent)
	
	# Apply inherited defaults (before activation to allow per comp overrides)
	# Remember, merge by default DOES NOT overwrite existing keys
	
	var ValuesToInheritByDefault: Dictionary = {
		"heightAlignment": heightAlignment,
		"widthAlignment": widthAlignment
	}
	
	component.merge(ValuesToInheritByDefault)
	
	# Activate
	newComponent.Activate(component)
	
	# Connect Component To Unit
	if(newComponent.dataSignalAvailable and !newComponent.DataUp.is_connected(_PassThroughSignalFromComponent)):
		newComponent.DataUp.connect(_PassThroughSignalFromComponent)
	if (!newComponent.SizeChanged.is_connected(_RecieveSizeChangeNotificationFromBelow)):
		newComponent.SizeChanged.connect(_RecieveSizeChangeNotificationFromBelow)
	_components.append(newComponent)

# Takes an array of dictionaries describing new components, adds them in in order
func AddMultipleComponents(components: Array) -> void:
	for c in components:
		AddComponent(c)

# Attepts to relay Dictionary of input
func RelayInputDataToComps(input: Dictionary) -> void:
	for key in input.keys():
		if !(key in componentIDs):
			# the ID trying to be pushed does not exist
			print("The component ID ", key, " does not exist in Unit ", ID)
			continue
		
		if !(is_instance_of(input[key], TYPE_DICTIONARY)):
			# Inputted data is not a dictionary
			print("The input for component ", key, " for Unit ", ID, " is not a dictionary!")
			continue
		
		componentRefs[key].ApplyPropertiesFromDict(input[key])

# Private functions

####################################
########## Sizing Systems ##########
####################################

# Can be called externally from above to request a size change.
# Returns false if requested size smaller than min size
# If requested size is smaller than allowed size, it will be expanded
func  RequestSizeChange(newSize: Vector2) -> bool:
	var output := true
	if (HelperFuncs.IsVector2SmallerInAnyDim(newSize, _minDimensions)):
		newSize = HelperFuncs.ClampVector2ToLargestAllowed(newSize, _minDimensions)
		output = false
	
	if(isHorizontal):
		_GrowChildren_Vertically(newSize.y)
	else:
		_GrowChildren_Horizontally(newSize.x)
	_RepositionChildren(newSize)
	size = newSize
	return output

# Similar to Request Size Change, but forces size expansion to at least the
# minimum allowed size instead of something user defined. Can also be optionally
# forced to run size / child positioning updates regardless
func UpdateSizeData(forceUpdate: bool = false) -> void:
	_requestingSizeChange = false
	if(_UpdateMinimumDimensions() || forceUpdate):
		Hsize = HelperFuncs.GrowVector2ToSmallestAllowed(Hsize, _minDimensions)
		_RepositionChildren(_minDimensions)
		size = _minDimensions
	SizeChanged.emit(self)

# Forces a recalculation of minimum required dimensions. If minimum required
# size is bigger than the current size, returns true and updates minimum dim.
# STILL REQUIRED TO TAKE ACTION
func _UpdateMinimumDimensions() -> bool:
	var newMinDim: Vector2 = Vector2(_GetMinWidth(), _GetMinHeight())
	if (HelperFuncs.IsVector2SmallerInAnyDim(_minDimensions, newMinDim )):
		_minDimensions = HelperFuncs.GrowVector2ToSmallestAllowed(_minDimensions, newMinDim)
		return true
	return false

# General method for repositioning children
# (Re) Writing this function was pain
func _RepositionChildren(parentSize: Vector2) -> void:
	var children: Array = get_children()
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
	
	match(widthAlignment):
		0: gap = 0.0
		1: gap = (parentSize.x - HelperFuncs.SumFloatArray(childHs)) / float(childHs.size() - 1)
		2: gap = (parentSize.x - HelperFuncs.SumFloatArray(childHs))
	
	if (gap == NAN) or (gap == INF):
		gap = (parentSize.x - HelperFuncs.SumFloatArray(childHs)) / 2.0
	
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
	
	match(widthAlignment):
		0: gap = 0.0
		1: gap = (parentSize.y - HelperFuncs.SumFloatArray(childVs)) / float(childVs.size() - 1)
		2: gap = (parentSize.y - HelperFuncs.SumFloatArray(childVs))
	
	if (gap == NAN) or (gap == INF):
		gap = (parentSize.y - HelperFuncs.SumFloatArray(childVs)) / 2.0
	
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
 
# Gets the minimum allowed width of the Unit
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

# Get Minimum allowed height of the Unit
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

func _GetTallestComponentHeight() -> float:
	var tallest: float = 0.0
	for i in range(_components.size() - 1):
		if(_components[i].Hsize.y > tallest):
			tallest = _components[i].Hsize.y
	return tallest + padding.y

func _GetWidestComponentWidth() -> float:
	var widest: float = 0.0
	for i in range(_components.size() - 1):
		if(_components[i].Hsize.x > widest):
			widest = _components[i].Hsize.x
	return widest + padding.x

# Sets new width for children. Ensure this width is valid before running this method
func _GrowChildren_Horizontally(newWidth: float) -> void:
	
	var children: Array = get_children()
	for child in children:
		child.Hsize = Vector2(newWidth, child.Hsize.y)

# Sets new height for children. Ensure this height is valid before running this method
func _GrowChildren_Vertically(newHeight: float) -> void:
	
	var children: Array = get_children()
	for child in children:
		child.Hsize = Vector2(child.Hsize.x, newHeight)

####################################
##### Component Data Signaling #####
####################################

# Assembles an output dictionary with keys being component IDs and
# values being that component's values. Skips over components like headers
func _GetComponentsData() -> Dictionary:
	var output := {}
	for comp in _components:
		if !comp.dataAvailable:
			continue # Skip over anything with no data available
		output[comp.ID] = comp.data
	return output
	
# using a data dict of ID references, sets variables to defined variables.
# dataIn is formatted as such:
# { ComponentID:
#   { variableName: Value }
# }
func _SetComponentsData(dataIn: Dictionary) -> void:
	var compReferences: Dictionary = _GetComponentReferencesByID()
	
	for ComponentID in dataIn.keys():
			for variableName in dataIn[ComponentID].keys():
				# Programming warcrime
				compReferences[ComponentID][variableName] = dataIn[ComponentID][variableName]

# Returns array of all Component IDs
func _GetArrayOfCompIDs() -> Array:
	var output := []
	for comp in _components:
		output.append(comp.ID)
	return output

# returns a dictionary of all Component object references by their ID
func _GetComponentReferencesByID() -> Dictionary:
	var children: Array = get_children()
	var output: Dictionary = {}
	var i = 0
	for compID in componentIDs:
		output[compID] = children[i]
		i = i + 1 # this is stupid
	return output

# passes through signal from components to above.
# custom Data Structure:
# "compID" : String ID of the component signaling
# "unitID" : String ID of the component signaling
# "type" : String type of the component signaling
# ... : any other data is dependent on the type of comp signaling. See docs
func _PassThroughSignalFromComponent(customData: Dictionary, changedCompReference):
	customData["compID"] = changedCompReference.ID
	customData["unitID"] = ID
	DataUp.emit(customData, changedCompReference, self)

# This function relays signal input through a deffered call.
# This allows default Godot UI resizing behvior to apply and for us to read it
# rapidly afterwards.
# This is not particuarly efficient. Too bad!
func _RecieveSizeChangeNotificationFromBelow(_objectReference) -> void:
	if _requestingSizeChange: return # avoid repitition
	_requestingSizeChange = true
	call_deferred("UpdateSizeData") # I am sure this won't cause problems /s

