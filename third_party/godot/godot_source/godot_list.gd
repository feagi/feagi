extends Node

var godot_list = {}
var godot_list_boolean = false


func _ready():
	godot_list["\'data\'"] = {}
	godot_list["\'data\'"]["\'direct_stimulation\'"] = {} ##this godot deadass include slash which we dont see it in python but this does in godot only. I HAD MULTIPLE ISSUES
	#This just include single quote but yeah, so everytime I do dict in godot..I HAVE to include those
	#even i need to put backslash in name too. it doe
