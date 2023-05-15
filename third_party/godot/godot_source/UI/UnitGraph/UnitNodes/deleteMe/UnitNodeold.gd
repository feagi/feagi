extends GraphNode
class_name UnitNodeOld


# var title
# var show_close 
var unitNodeMinSize: Vector2:
	get: return _unitNodeMinSize


# load in Units
# have generators for inputs and Outputs

var _unitNodeMinSize: Vector2
var _UnitScene: PackedScene = preload("res://UI/Units/unit.tscn")
var _itemCount: int = 0

# Activates the Unit Node
# Units is an array of Units to be stored on the top
# inputs is an array of component definitions (or strings) to generate inputs
# ditto for outputs
func Activation(units: Array, inputs: Array, outputs: Array,
	nodeTitle: String, closable: bool):
	
	title = nodeTitle
	show_close = closable
	
	_AddUnits(units)
	_AddConnector(true, inputs)
	_AddConnector(false, outputs)
	
	

# Adds Units to the top of the UnitNode
func _AddUnits(units: Array):
	_itemCount += units.size()
	for u in units:
		var newUnit = _UnitScene.instantiate()
		add_child(newUnit)
		#TODO positioning

func _AddConnector(isInput: bool, arr: Array):
	for a in arr:
		var compDef: Dictionary
		if a is String:
			compDef = _GenerateHeaderComponentDefinition(isInput, a)
		else:
			# assume component definition
			compDef = a
		var newComp = _CreateConnector(isInput, compDef)
		add_child(newComp)
		if isInput:
			set_slot_enabled_left(_itemCount, true)
		else:
			set_slot_enabled_right(_itemCount, true)
		_itemCount = _itemCount + 1


func _GenerateHeaderComponentDefinition(isInput: bool, headerText: String):
	var side: int
	if isInput: side = 0
	else: side = 2
	return {"side" : side, "label": headerText, "type": "header"}

func _CreateConnector(isInput: bool, componentDefinition : Dictionary, ID: int = 0):
	#TODO this is REALLY bad, refactor this
	var _fieldScene: PackedScene = preload("res://UI/Units/Components/Field/field.tscn")
	var _counterScene: PackedScene = preload("res://UI/Units/Components/Counter/counter.tscn")
	var _toggleScene: PackedScene = preload("res://UI/Units/Components/Toggle/Toggle.tscn")
	var _dropdownScene: PackedScene = preload("res://UI/Units/Components/DropDown/DropDown.tscn")
	var _headerScene: PackedScene = preload("res://UI/Units/Components/Header/header.tscn")

	
	var newComponent
	match componentDefinition["type"]:
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
	return newComponent
	






