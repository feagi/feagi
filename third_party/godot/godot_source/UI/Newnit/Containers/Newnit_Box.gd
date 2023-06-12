extends BoxContainer
class_name Newnit_Box

# This base class is used to construct all Elements, which are parallel in 
# implmentation to Newnits but insteadof holding containers, are UI elements

######################## START Newnit Parallel - this section must match that of Newnit_Base ########################

const NEWNIT_CORE = preload("res://UI/Newnit/PreloadScripts/Newnit_core.gd")

signal DataUp(data, originatingID, originatingRef)

var ID: StringName:
	get: return _ID

var parent: Node:
	get: return get_node("../")

var parentID: StringName:
	get: return NEWNIT_CORE.Get_ParentID(self)

var childrenIDs: Array:
	get: return NEWNIT_CORE.Func_GetChildIDs(children)

var data: Dictionary:
	get: return NEWNIT_CORE.Get_data(self)

var type: StringName:
	get: return _type

var _ID: StringName
var _isActivated := false
var _isTopLevel := true
var _runtimeSettableProperties := NEWNIT_CORE.settableProperties
var _type: StringName

func Activate(settings: Dictionary) -> void:
	NEWNIT_CORE.Func_Activate(settings, self)

# Set Properties from dictionary
func SetData(input: Dictionary) -> void:
	NEWNIT_CORE.Func_SetData(input, self)

# Does an indepth search of all children by ID, and returns a node reference to
# a matching ID. If none are found, returns false
func GetReferenceByID(searchID: StringName): # returns either a bool or a Node
	NEWNIT_CORE.Func_GetReferenceByID(searchID, self)

################################################ END Newnit Parallel ################################################

################### START Containers Parallel - this section must match that of other Newnit Containers ##############

const NEWNIT_CONTAINER_CORE = preload("res://UI/Newnit/PreloadScripts/Container_Core.gd")

var children: Array:
	get: return NEWNIT_CONTAINER_CORE.Get_children(self)

func SpawnChild(childActivationSettings: Dictionary) -> void:
	NEWNIT_CONTAINER_CORE.Func_SpawnChild(childActivationSettings, self)

func SpawnMultipleChildren(childrenActivationSettings: Array) -> void:
	NEWNIT_CONTAINER_CORE.Func_SpawnMultipleChildren(childrenActivationSettings, self)

func _ActivationPrimary(settings: Dictionary) -> void:
	NEWNIT_CONTAINER_CORE.Func__ActivationPrimary(settings, self)

func _getChildData() -> Dictionary:
	return NEWNIT_CONTAINER_CORE.Func__getChildData(self)

################################################# END Newnit Containers Parallel #######################

### Start Box Container Unique

func _ActivationSecondary(settings: Dictionary) -> void:
	alignment = HelperFuncs.GetIfCan(settings, "alignment", NEWNIT_CONTAINER_CORE.D_alignment)
	vertical = HelperFuncs.GetIfCan(settings, "vertical", NEWNIT_CONTAINER_CORE.D_vertical)
	
