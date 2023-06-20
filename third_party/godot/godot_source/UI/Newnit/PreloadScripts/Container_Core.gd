# This script is to be preloaded into Newnit CONTAINER instances
# Since this is universal, it should be kept to rather global functions to 
# avoid overcoupling

static func Func_SpawnChild(childActivationSettings: Dictionary, ContainerObject) -> void:
	# TODO: (optional) Property Inheritance
	
	var newChild
	
	match childActivationSettings["type"]:
		"counter": newChild = Element_Counter.new()
		"button": newChild = Element_Button.new()
		"checkbox": newChild = Element_CheckBox.new()
		"checkbutton": newChild = Element_CheckButton.new()
		"dropdown": newChild = Element_DropDown.new()
		"field": newChild = Element_Field.new()
		"floatfield": newChild = Element_FloatField.new()
		"header": newChild = Element_Label.new()
		"vector3": newChild = Element_Vector3.new()
		"box": newChild = Newnit_Box.new()
		"tab": newChild = Newnit_Tabs.new()
		"scrollbar": newChild = Newnit_Scroll.new()
		"list": newChild = Element_List.new()
		"popup": newChild = Newnit_Popup.new()
		"collapsible": newChild = Newnit_Collapsible.new()
		# TODO: More types!
		_:
			print("Invalid child of type ", childActivationSettings["type"], " attempted to spawn. Skipping...")
			return
	newChild._hasNewnitParent = true
	newChild._parent = ContainerObject
	newChild.Activate(childActivationSettings)
	newChild.DataUp.connect(ContainerObject._DataUpProxy)
	ContainerObject.add_child(newChild)

static func Func_SpawnMultipleChildren(childrenActivationSettings: Array, ContainerObject) -> void:
	for c in childrenActivationSettings:
		Func_SpawnChild(c, ContainerObject)

static func Func__ActivationPrimary(settings: Dictionary, ContainerObject) -> void:
	Func_SpawnMultipleChildren(settings["components"], ContainerObject)
	ContainerObject._ActivationSecondary(settings)

static func Func__getChildData(ContainerObject) -> Dictionary:
	var output := {}
	for child in ContainerObject.children:
		output.merge(child.data)
	return output

static func Get_children(ContainerObject: Node) -> Array:
	var childrens := []
	for child in ContainerObject.get_children():
		childrens.append(_GetNewnitChild(child))
	return childrens

static func _GetNewnitChild(checkingChild: Node) -> Node:
	if checkingChild.get_class() == "Panel":
		_GetNewnitChild(checkingChild.get_child(0))
	return checkingChild

static func PreAppendElementToComponents(settings: Dictionary, 
newElement: Dictionary) -> Dictionary:
	
	var components: Array = settings["components"]
	components.push_front(newElement)
	settings["components"] = components
	return settings
	

# Defaults and other constants
const D_vertical = true
const D_alignment = 0

const D_current_tab = 0
const D_tab_alignment = 1
const D_use_hidden_tabs_for_min_size = true
const D_tab_titles = []
const D_Title := "NO TITLE GIVEN"
const D_COLLAPSIBLELABER := "NO COLLAPSE LABEL GIVEN"
