# This script is to be preloaded into Newnit instances
# Since this is universal, it should be kept to rather global functions to 
# avoid overcoupling


static func Func_GetChildIDs(children: Array) -> Array:
	var output := []
	for child in children:
		output.append(child.ID)
	return output



static func Func_Activate(settings: Dictionary, NewnitObject) -> void:
	if NewnitObject._isActivated: return
	
	# Convert JSON Vector Parts into Vectors
	settings = HelperFuncs.RemapVector2FloatsToVector2("position", settings)
	settings = HelperFuncs.RemapVector2FloatsToVector2("size", settings)
	settings = HelperFuncs.RemapVector2FloatsToVector2("mSize", settings)
	settings = HelperFuncs.RemapRGBToVector3i("color", settings)
	settings = HelperFuncs.RemapVector3FloatsToVector3("vectorValue", settings)
	
	# Apply Control Properties
	NewnitObject.custom_minimum_size = HelperFuncs.GetIfCan(settings, "mSize", D_custom_minimum_size)
	NewnitObject.grow_horizontal = HelperFuncs.GetIfCan(settings, "grow_horizontal", D_grow_horizontal)
	NewnitObject.grow_vertical = HelperFuncs.GetIfCan(settings, "grow_vertical", D_grow_vertical)
	NewnitObject.mouse_filter = HelperFuncs.GetIfCan(settings, "mouse_filter", D_mouse_filter)
	NewnitObject.offset_bottom = HelperFuncs.GetIfCan(settings, "offset_bottom", D_offset_bottom)
	NewnitObject.offset_left = HelperFuncs.GetIfCan(settings, "offset_left", D_offset_left)
	NewnitObject.offset_right = HelperFuncs.GetIfCan(settings, "offset_right", D_offset_right)
	NewnitObject.offset_top = HelperFuncs.GetIfCan(settings, "offset_top", D_offset_top)
	NewnitObject.position = HelperFuncs.GetIfCan(settings, "position", D_position)
	NewnitObject.size = HelperFuncs.GetIfCan(settings, "size", D_size)
	NewnitObject.size_flags_stretch_ratio = HelperFuncs.GetIfCan(settings, "size_flags_stretch_ratio", D_size_flags_stretch_ratio)
	NewnitObject.tooltip_text = HelperFuncs.GetIfCan(settings, "tooltip_text", D_tooltip_text)
	
	# Apply Custom Properties
	NewnitObject._ID = HelperFuncs.GetIfCan(settings, "ID", D_ID)
	NewnitObject._type = HelperFuncs.GetIfCan(settings, "type", D_type)
	NewnitObject.name = Func__GetUIChildName(NewnitObject.type, NewnitObject)
	
	if(HelperFuncs.GetIfCan(settings, "enablePanel", D_EnablePanel)):
		Func_AddPanel(NewnitObject)
	
	NewnitObject._ActivationPrimary(settings)
	NewnitObject._isActivated = true

# Set Properties from dictionary
static func Func_SetData(input: Dictionary, NewnitObject) -> void:
	
	#childrenIDs
	#ID
	#_runtimeProperties
	
	for key in input.keys():
		
		if key in NewnitObject.childrenIDs:
			# dig deeper
			# TODO: This is not particuarly efficient. Too Bad!
			var search: int = NewnitObject.childrenIDs.find(key)
			NewnitObject.children[search].SetData(input[key])
			continue
		
		# check if key is referring to property of this object
		if key in NewnitObject._runtimeSettableProperties.keys():
			
			var inputVar = input[key]
			
			# confirm type matches!
			if typeof(inputVar) != typeof(NewnitObject[key]):
				# see if we can convert before giving up
				if (typeof(inputVar) == 3) && (typeof(NewnitObject[key]) == 2):
					# auto convert from float to int
					inputVar = int(inputVar)
				else:
					print("Input is of type ", typeof(input[key]), " when expected ", typeof(NewnitObject[key]), "! Skipping!")
					continue
			
			NewnitObject[key] = inputVar # This is blasphemy
			continue
	
		# On the odd case where the key is the ID of this object
		if key == NewnitObject.ID:
			# This may be a mistake, but we can use recursion to deal with this
			print("Recursive ID path detected, readjusting...\nYou may want to fix this...")
			NewnitObject.SetData(input[key])
			continue
		
		# Key not found!
		print("Property ", key, " does not exist!")
		continue

static func Func_AddPanel(NewnitObject: Node) -> void:
	NewnitObject._isUsingPanel = true
	var panel: Panel = Panel.new()
	panel.size_flags_horizontal = Control.SIZE_EXPAND_FILL 
	NewnitObject._childRoot.add_child(panel)
	NewnitObject._panelRef = panel
	NewnitObject._childRoot = panel

static func Func__GetUIChildName(compType: StringName, NewnitObject) -> StringName:
	return compType + "_" + NewnitObject.ID


static func Get_data(NewnitObject: Node) -> Dictionary:
	var o: Dictionary = {"ID": NewnitObject.ID}
	o.merge(NewnitObject._getChildData())
	return o

static func Get_ParentID(NewnitObject: Node) -> StringName:
	if "ID" in NewnitObject.parent:
		return NewnitObject.parent.ID
	return StringName("No ID Found!")

	


# Defaults and other constants
const D_ID = "ERROR_NO_ID"
const D_custom_minimum_size = Vector2(0,0)
const D_grow_horizontal = 1
const D_grow_vertical = 1
const D_mouse_filter = 1
const D_offset_bottom = 0.0
const D_offset_left = 0.0
const D_offset_right = 0.0
const D_offset_top = 0.0
const D_position = Vector2(0.0,0.0)
const D_size = Vector2(50.0,50.0)
const D_size_flags_stretch_ratio = 1.0
const D_tooltip_text = ""
const D_type = "ERROR_UNKNOWN_TYPE"
const D_EnablePanel = false
const settableProperties := {
	"custom_minimum_size": TYPE_VECTOR2,
	"grow_horizontal": TYPE_INT,
	"grow_vertical": TYPE_INT,
	"mouse_filter": TYPE_INT,
	"offset_bottom": TYPE_FLOAT,
	"offset_left": TYPE_FLOAT,
	"offset_right": TYPE_FLOAT,
	"offset_top": TYPE_FLOAT,
	"position": TYPE_VECTOR2,
	"size": TYPE_VECTOR2,
	"size_flags_stretch_ratio": TYPE_FLOAT,
	"tooltip_text": TYPE_STRING,
}


# Other Properties & Methods:
# https://docs.godotengine.org/en/stable/classes/class_container.html#class-container
# custom_minimum_size: Vector2 - The minimum size of the object
# grow_horizontal: int - direction to grow
# grow_vertical: int - direction to grow
# mouse_filter: int
# offset_bottom, offset_left, offset_right, offset_top: float - "padding"
# position: Vector2
# size: Vector2 - size, should probably to be changed for non-top members
# size_flags_stretch_ratio: float
# tooltip_text: String
# void reset_size() -> resets to minimum size
