extends Control

# Example: Loading from Godot Memory

# example of data in memory that you want to pass through
const CorticalAreas = { "CORTICALAREAS" : { "options" : ["IPU", "Visual", "OPU"]}}


var _UnitScene: PackedScene = preload("res://UI/Units/unit.tscn")
var TopBar

func _respondToSignal(dataDict: Dictionary, compRef, unitRed):
	print("Got Signal!")
	
	if dataDict["type"] == "dropdown":
		if dataDict.has("button"):
			# button was pressed
			print("Button Pressed!")
		else:
			# dropdown changed
			print("Dropdown changed to " + dataDict["selected"])
	
	if dataDict["type"] == "counter":
		# counter was changed
		print("counter changed to " + str(dataDict["number"]))


func _ready():
	
	# Create Bar Object
	TopBar = _UnitScene.instantiate()
	add_child(TopBar)
	
	# Structure JSON file
	var structfile = HelperFuncs.readTextFile("res://circuit_build/TopBar_Example/TopBar.JSON")
	# file with all the english
	var engfile = HelperFuncs.readTextFile("res://circuit_build/TopBar_Example/lang/eng.JSON")
	
	var UnitDict = HelperFuncs.UnitFromJSONS(structfile, engfile, CorticalAreas)
	
	TopBar.Activate(UnitDict)
	
	print(TopBar.componentData)
	
	TopBar.dataUp.connect(_respondToSignal)
	
	var dictToApply = {
		"TITLE": {"label": "New Test"}
	}
	
	TopBar.RelayInputDataToComps(dictToApply)
