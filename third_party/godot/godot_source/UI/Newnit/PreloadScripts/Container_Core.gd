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
		# TODO: More types!
		_:
			print("Invalid child of type ", childActivationSettings["type"], " attempted to spawn. Skipping...")
			return
	
	ContainerObject.add_child(newChild)
	newChild.Activate(childActivationSettings)

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

static func Get_children(ContainerObject) -> Array:
	return ContainerObject.get_children()

# Defaults and other constants
const D_vertical = true
const D_alignment = 0