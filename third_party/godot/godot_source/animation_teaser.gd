extends Button

func _on_pressed():
	if $".."/Core/GlobalUISystem/Brain_Visualizer/Node3D/Camera3D.current:
		$".."/AnimationPlayer/Camera3D.make_current()
		$".."/AnimationPlayer.play("animation")
	elif $".."/AnimationPlayer/Camera3D.current:
		$".."/Core/GlobalUISystem/Brain_Visualizer/Node3D/Camera3D.make_current()
