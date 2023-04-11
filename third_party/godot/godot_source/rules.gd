extends ColorRect

var flag = false

# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.
	
func _process(_delta):
	if $morphology_definition/associations_data.text != "":
		$delete.disabled = true
	else:
		$delete.disabled = false
	if $morphology_definition/composite_label/morphology_name.get_selected_id() == 0:
		$save.disabled = true
	else:
		$save.disabled = false
