extends ScrollContainer
class_name List_Sub

signal value_edited(dict) # in this case calls depending on interaction

func Activate(prefabAct: Dictionary, isUsingCachedData: bool) -> void:
	_scrollingBoxContainer = BoxContainer.new()
	add_child(_scrollingBoxContainer)
	
	usingCachedData = isUsingCachedData
	prefabActivation = prefabAct
	
	AppendItem(false)
	
	size = Vector2(200,200) # VERY TEMP!

func Update(index: int) -> void:
	pass

func UpdateAll() -> void:
	for i in range(len(children)):
		Update(i)

func AppendItem(includingSideButton: bool = true) -> void:
	var subList = Newnit_SubList.new()
	
	var actDict := prefabActivation
	if !includingSideButton:
		actDict["includeSideButton"] = false
	
	var fullActivation = {
		"ID": "TEST0",
		"components": actDict
		}
	
	subList.Activate(fullActivation)
	pass

func RemoveItem(index: int) -> void:
	children[index].queue_free()
	pass

func Export() -> Array:
	return []

func _DataUpProxy(data) -> void:
	pass


var children: Array:
	get: return _scrollingBoxContainer.get_children()

var vertical: bool:
	get: return _vertical
	set(v): _vertical = v
var usingCachedData: bool = true
var prefabActivation: Dictionary
var prefabCachedData: Dictionary = {}
var _scrollingBoxContainer: BoxContainer
var _vertical: bool = false




# built in vars
# text: String
# size: Vector2
# signal pressed
