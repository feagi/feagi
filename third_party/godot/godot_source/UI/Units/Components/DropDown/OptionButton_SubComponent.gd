extends OptionButton
class_name OptionButton_SubComponent
# adding some sorely needed features to OptionButton

signal SizeChanged(selfReference)
signal ValueManuallyChanged(newString: String, selfReference)

var Hsize: Vector2:
	get: return size
	set(v): 
		size = v
		SizeChanged.emit(self)


var options: Array:
	get: return _DropDownItems
	set(v): 
		_SetDropDownArray(v)
var index: int:
	get: return selected
	set(v): 
		select(v)
var value: String:
	get: 
		if index == -1: return ""
		return get_item_text(index)
	set(v): _SetToExistingString(v)


var _DropDownItems: Array

func GetStringFromIndex(ind: int) -> String:
	return _DropDownItems[ind]

func _SetDropDownArray( strArr: Array) -> void:
	_DropDownItems = strArr
	var currentSelection = value
	clear()
	for item in strArr:
		add_item(item)
	if _DropDownItems.find(currentSelection) == -1:
		# Seems that the previously selected item is unavailable
		# set to unknown
		select(-1)
		ValueManuallyChanged.emit("", self)

# use to set the value of the drop down directly via string
func _SetToExistingString( option: String) -> void:
	var location: int = _DropDownItems.find(option)
	if location == -1:
		print("No Option found with name" + option)
		ValueManuallyChanged.emit("", self)
		return
	select(location)
	ValueManuallyChanged.emit(option, self)


