extends Node3D

func ready():
	$Button.position.y = $Menu/information_menu.position.y
	

func change_scene():
	var BV = $Core/GlobalUISystem/Brain_Visualizer
	var CB = $Core/GlobalUISystem
	$scene_transition/AnimationPlayer.play("dissolve")
	await $scene_transition/AnimationPlayer.animation_finished
	if BV.visible:
		BV.visible = false
		$Menu.visible = false
		CB.visible = true
		$Button.text = "Brain Visualizer"
	elif CB.visible:
		BV.visible = true
		$Menu.visible = true
		CB.visible = false
		$Button.text = "Circuit Builder"
	$scene_transition/AnimationPlayer.play_backwards("dissolve", -1)

func _on_button_pressed():
	change_scene()
	$Button.release_focus()
#	print("Here: ", $Control.TopBar.componentData)
