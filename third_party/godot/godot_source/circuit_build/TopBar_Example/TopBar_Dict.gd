extends Control

# Example: Loading from Godot Memory

const comp1: Dictionary = {
	"type" : "header",
	"ID" : "TITLE",
	"label" : "Bar Test"
}

const comp2: Dictionary = {
	"type" : "counter",
	"ID" : "NEURONCOUNTER",
	"label" : "Number Of Neurons",
}

const comp3: Dictionary = {
	"type" : "dropdown",
	"label" : "Cortical Areas",
	"ID" : "CORTICALAREAS",
	"options" : ["IPU", "Visual", "OPU"],
	"hasButton" : true
}

const isVertical = false

var _UnitScene: PackedScene = preload("res://UI/Units/unit.tscn")
var TopBar

func _ready():
	
	var listOfComponents = [comp1, comp2, comp3]
	
	TopBar = _UnitScene.instantiate()
	add_child(TopBar)
	
	var BarDict = {"components": listOfComponents, "ID" : "BAR"}
	
	TopBar.Activate(BarDict)
	
