extends OptionButton
class_name OptionButton_Sub

signal value_edited(stringAndIndex: Dictionary)

const INTERNAL_WIDTH_PADDING := 45.0 # Large due to existance of drop down icon

var shouldScaleWithLongestOption := true

var options: Array:
	get: return _DropDownItems
	set(v): 
		SetDropDownArray(v)
		_UpdateSizeAsPerOptions()
var index: int:
	get: return selected
	set(v): 
		select(v)
var value: String:
	get: 
		if index == -1: return ""
		return get_item_text(index)
	set(v): SetToExistingString(v)

var editable: bool:
	get: return !disabled
	set(v): disabled = !v

var _DropDownItems: Array

#func GetStringFromIndex(ind: int) -> String:
#	return _DropDownItems[ind]

func SetDropDownArray( strArr: Array, emitIfCurrentOptionNonExistant: bool = false) -> void:
	_DropDownItems = strArr
	var currentSelection = value
	clear()
	for item in strArr:
		add_item(item)
	if _DropDownItems.find(currentSelection) == -1:
		# Seems that the previously selected item is unavailable
		# set to unknown
		select(-1)
		if(emitIfCurrentOptionNonExistant): value_edited.emit({"value": "", "selectedIndex": -1})

# use to set the value of the drop down directly via string
func SetToExistingString( option: String, emitWhenCalled: bool = false) -> void:
	var location: int = _DropDownItems.find(option)
	if location == -1:
		print("No Option found with name" + option)
		if(emitWhenCalled): value_edited.emit({"value": "", "selectedIndex": -1})
		return
	select(location)
	if(emitWhenCalled): value_edited.emit({"value": option, "selectedIndex": location})

# Gets the width of the widest option
func _GetLongestOptionWidth() -> float:
	var widest := 0.0
	for element in _DropDownItems:
		if get_theme_font("font").get_string_size(element).x > widest:
			widest = get_theme_font("font").get_string_size(element).x
	return widest

# Updates the size of of the dropdown as per the listed options
func _UpdateSizeAsPerOptions() -> void:
	size = Vector2(_GetLongestOptionWidth() + INTERNAL_WIDTH_PADDING, size.y)

func _ready():
	item_selected.connect(_ProxySelected)

func _ProxySelected(pIndex: int):
	value_edited.emit({"selectedIndex": pIndex})
