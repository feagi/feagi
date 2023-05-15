extends GridMap

var x_input
var y_input
var z_input

# Called when the node enters the scene tree for the first time.
func _ready():
	pass

# Called every frame. 'delta' is the elapsed time since the previous frame.
func run(x_input,y_input,z_input):
	set_cell_item( Vector3(x_input,y_input,z_input) ,0)
	await get_tree().create_timer(0.01).timeout
#	clear()
	queue_free()
