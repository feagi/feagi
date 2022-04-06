extends GridMap


var timer = false


# This will clear red voxels automatically without halt the process of godot
func time_to_clear():
	yield(get_tree().create_timer(1), "timeout")
	clear()
	timer = false

func _process(delta):##We will need to createa something to make this more scalable later.
	if timer:
		time_to_clear()
	else:
		pass
