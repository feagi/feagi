extends Node



var loadDict: Dictionary = {
	"type" : "field",
	"label" : "Field 1",
	"fieldWidth" : 300,
}

var loadDict2: Dictionary = {
	"type" : "counter",
	"label" : "Field 2",
}

var loadDict3: Dictionary = {
	"type" : "dropdown",
	"label" : "dd",
	"options" : ["a", "b", "c"],
	"hasButton" : true
}

var loadDict4: Dictionary = {
	"type" : "header",
	"label" : "wordswordswords",
}

var complist = [loadDict2]
var _UnitNodeScene: PackedScene = preload("res://UI/UnitGraph/UnitNodes/unitNode.tscn")

func _ready():
	var UN = _UnitNodeScene.instantiate()
	var UN2 = _UnitNodeScene.instantiate()
	add_child(UN)
	UN.Activation([],["in"],["out"], "title", false)
	add_child(UN2)
	UN2.Activation([],["in"],["out"], "title2", false)

