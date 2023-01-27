extends Control


# Declare member variables here. Examples:
# var a = 2
# var b = "text"


# Called when the node enters the scene tree for the first time.
func _ready():
	visible = false

func _process(_delta):
	if $inner_box/morphology_name.text != "" and $inner_box/morphology_type.selected != 0 and $inner_box/TextEdit.text != "":
		$create.disabled = false
	else:
		$create.disabled = true
