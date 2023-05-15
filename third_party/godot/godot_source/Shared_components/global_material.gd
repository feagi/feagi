extends Node


@onready var white = preload("res://brain_visualizer_source/white.material")
@onready var deselected = preload("res://brain_visualizer_source/cortical_area.material")
@onready var selected = preload("res://brain_visualizer_source/selected.material")

func ready():
	print("material created and ready now.")
