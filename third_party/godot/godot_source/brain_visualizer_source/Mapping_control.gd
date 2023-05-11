extends Control


func _ready():
	Autoload_variable.MappingControl_Core = get_parent().get_parent().get_parent().get_parent().get_node("Core")
	load_options()
	visible = false

func load_options():
	Autoload_variable.MappingControl_Core.Update_MorphologyList()

