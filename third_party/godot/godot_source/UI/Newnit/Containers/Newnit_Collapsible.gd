extends BoxContainer
class_name Newnit_Collapsible

# This base class is used to construct all Elements, which are parallel in 
# implmentation to Newnits but insteadof holding containers, are UI elements

######################## START Newnit Parallel - this section must match that of Newnit_Base ########################

const NEWNIT_CORE = preload("res://UI/Newnit/PreloadScripts/Newnit_core.gd")

signal DataUp(data: Dictionary, originatingID: StringName, originatingRef: Node)

var ID: StringName:
	get: return _ID

var parent: Node:
	get: return _parent

var parentID: StringName:
	get: return NEWNIT_CORE.Get_ParentID(self)

var childrenIDs: Array:
	get: return NEWNIT_CORE.Func_GetChildIDs(children)

var data: Dictionary:
	get: return NEWNIT_CORE.Get_data(self)

var type: StringName:
	get: return _type

var panelRef: Node:
	get: return _panelRef

var marginRef: Node:
	get: return _marginRef

var hasNewnitParent: bool:
	get: return _hasNewnitParent

var draggable: bool

var _ID: StringName
var _isActivated := false
var _isTopLevel := true
var _runtimeSettableProperties := NEWNIT_CORE.settableProperties
var _type: StringName
var _panelRef: Node = null
var _parent: Node = null
var _hasNewnitParent: bool = false
var _marginRef: Node = null

func Activate(settings: Dictionary) -> void:
	NEWNIT_CORE.Func_Activate(settings, self)

# Set Properties from dictionary
func SetData(input: Dictionary) -> void:
	NEWNIT_CORE.Func_SetData(input, self)

func GetReferenceByID(searchID: StringName): # returns either a bool or a Node
	if searchID == ID: return self
	for child in children:
		var result = child.GetReferenceByID(searchID)
		if typeof(result) != TYPE_BOOL:
			return result
	return false

func UpdatePosition(newPosition: Vector2) -> void:
	if panelRef != null: _panelRef.position = newPosition; return
	if marginRef != null: _marginRef.position = newPosition; return
	else: position = newPosition

func UpdateMargins(TopRightBottomLeftMargins: Array) -> void:
	NEWNIT_CORE.Func_UpdateMargin(self, TopRightBottomLeftMargins)

func _ResizePanel() -> void:
	if marginRef != null:
		#panelRef.size = marginRef.size
		panelRef.size = size + Vector2(20,20)
		return
	_panelRef.size = size

func _get_drag_data(at_position: Vector2):
	if draggable: UpdatePosition(at_position)

func _notification(what):
	if (what == NOTIFICATION_PREDELETE):
		if panelRef != null: panelRef.queue_free()

################################################ END Newnit Parallel ################################################

################### START Containers Parallel - this section must match that of other Newnit Containers ##############

const NEWNIT_CONTAINER_CORE = preload("res://UI/Newnit/PreloadScripts/Container_Core.gd")

func SpawnChild(childActivationSettings: Dictionary) -> void:
	NEWNIT_CONTAINER_CORE.Func_SpawnChild(childActivationSettings, self)

func SpawnMultipleChildren(childrenActivationSettings: Array) -> void:
	NEWNIT_CONTAINER_CORE.Func_SpawnMultipleChildren(childrenActivationSettings, self)

func _ActivationPrimary(settings: Dictionary) -> void:
	if(_AlternateActivationPath(settings)): return
	NEWNIT_CONTAINER_CORE.Func__ActivationPrimary(settings, self)

func _getChildData() -> Dictionary:
	return NEWNIT_CONTAINER_CORE.Func__getChildData(self)

func _DataUpProxy(data: Dictionary, recievedID: String, reference: Node) -> void:
	DataUp.emit(data, recievedID, reference)

################################################# END Newnit Containers Parallel #######################

### Start Box Container Unique

const _COLLAPSE_BUTTON := {
	"type": "header",
	"sideButtonText": "Toggle",
	"text": "COLLAPSABLE HEADER TITLE",
	"ID": "COLLAPSE_BUTTON"
}


var text: String:
	get: return _collapseHeader.text
	set(v): _collapseHeader.text = v

var children: Array:
	get: return NEWNIT_CONTAINER_CORE.Get_children(self)

var isCollapsed: bool:
	get: return _isCollapsed
	set(v):
		_isCollapsed = v
		ToggleCollapseSection(v)
		if(v):
			_collapseHeader.sideButtonText = "Show"
		else:
			_collapseHeader.sideButtonText = "Hide"

var specificSettableProps := {
	"alignment": TYPE_INT,
	"vertical": TYPE_INT,
	"CollapsableSectionText": TYPE_STRING
}

var _collapseHeader: Element_Label:
	get: return children[0]

var _isCollapsed: bool = false

func ToggleCollapseSection(collapsed: bool) -> void:
	# toggle whether a section is collapsed
	
	var childrenCache: Array = children
	for i in range(1,len(children)):
		childrenCache[i].visible = !collapsed

func _AlternateActivationPath(settings: Dictionary) -> bool:
	# Add a title bar and continue
	
	var childrenActivations: Array = settings["components"]
	childrenActivations.push_front(_COLLAPSE_BUTTON)
	settings["components"] = childrenActivations
	NEWNIT_CONTAINER_CORE.Func__ActivationPrimary(settings, self)
	return true

func _ActivationSecondary(settings: Dictionary) -> void:
	alignment = HelperFuncs.GetIfCan(settings, "alignment", NEWNIT_CONTAINER_CORE.D_alignment)
	vertical = HelperFuncs.GetIfCan(settings, "vertical", NEWNIT_CONTAINER_CORE.D_vertical)
	text = HelperFuncs.GetIfCan(settings, "text", NEWNIT_CONTAINER_CORE.D_Title)
	isCollapsed = HelperFuncs.GetIfCan(settings, "isCollapsed", false)
	
	_runtimeSettableProperties.merge(specificSettableProps)
	
	_collapseHeader.DataUp.connect(_UICollapseButton)
	type = "collapsible"

func _UICollapseButton(data: Dictionary, originatingID: String, reference: Node) -> void:
	if originatingID != "COLLAPSE_BUTTON": return
	isCollapsed = !isCollapsed
