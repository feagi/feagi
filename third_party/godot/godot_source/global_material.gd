extends Node


onready var white = preload("res://white.material")
onready var deselected = preload("res://cortical_area.material")
onready var selected = preload("res://selected.material")

func ready():
	print("material created and ready now.")
