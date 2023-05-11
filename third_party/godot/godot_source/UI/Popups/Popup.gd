extends Panel
class_name PopUp


const DEF_PADDING: Vector2 = Vector2(5.0,5.0)
const DEF_AUTOSIZE: bool = true
const DEF_POSITION: Vector2 = Vector2(0.0,0.0)
const DEF_HASCLOSEBUTTON: bool = true
const DEF_BOTTOMBUTTONS: Array = ["Ok"]
const DEF_TITLE: String = "NO TITLE FOUND"

var popUpID: String:
	get: return _ID
var padding: Vector2:
	get: return _padding
	set(v):
		_padding = v
		#TODO resize Event if activated
var title: String:
	get: return _Label.Htext
	set(v): _Label.Htext

signal buttonClosed(popupReference)

var _Label: Label_SubComponent
var _Xbutton: Button_SubComponent
var _bottomButtons: Array
var _Unit: Unit

var _activated: bool = false
var _ID: String
var _padding: Vector2
var _hasCloseButton: bool


func Activate(activationDict: Dictionary) -> void:
	
	_Label = $Button_SubComponent
	_Xbutton = $Label_SubComponent
	
	_ID = HelperFuncs.MustGet(activationDict, "popupID")
	var unitActivation: Dictionary = HelperFuncs.MustGet(activationDict, "unitActivation")
	padding = HelperFuncs.GetIfCan(activationDict, "padding", DEF_PADDING)
	title = HelperFuncs.GetIfCan(activationDict, "title", DEF_TITLE)
	_hasCloseButton = HelperFuncs.GetIfCan(activationDict, "hasCloseButton", DEF_HASCLOSEBUTTON)
	_bottomButtons = HelperFuncs.GetIfCan(activationDict, "unitActivation",DEF_BOTTOMBUTTONS)
	

	
	if(!_hasCloseButton):
		_Xbutton.queue_free()
	
	var _unitScene: PackedScene = preload("res://UI/Units/unit.tscn")
	_Unit = _unitScene.instantiate()
	add_child(_Unit)
	_Unit.Activate(unitActivation)
	
	#var _buttonScene: PackedScene = preload("res://UI/") TODO add button Comp
	

func resize(newSize: Vector2) -> void:
	size = newSize
	# no need to signal up, no dependents care
	if _hasCloseButton:
		_Xbutton.position = (newSize / 2.0) + padding
