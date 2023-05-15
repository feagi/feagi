extends Button


# Called when the node enters the scene tree for the first time.
func _ready():
	pass # Replace with function body.


# Called every frame. 'delta' is the elapsed time since the previous frame.
func _process(delta):
	pass


func _on_pressed():
	if $".."/Core/GlobalUISystem/Brain_Visualizer/Node3D/Camera3D.current:
		$".."/AnimationPlayer/Camera3D.make_current()
		$".."/AnimationPlayer.play("animation")
	elif $".."/AnimationPlayer/Camera3D.current:
		$".."/Core/GlobalUISystem/Brain_Visualizer/Node3D/Camera3D.make_current()
