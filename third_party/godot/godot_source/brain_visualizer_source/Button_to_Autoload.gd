extends Node2D

func _ready():
	pass # Replace with function body.

func _on_leak_mouse_entered():
	Godot_list.Node_2D_control = true

func _on_leak_Vtext_mouse_entered():
	Godot_list.Node_2D_control = true

func _on_count_spinbox_mouse_entered():
	Godot_list.Node_2D_control = true

func _on_count_spinbox_mouse_exited():
	Godot_list.Node_2D_control = false

func _on_type_text_changed(_new_text):
	Godot_list.Node_2D_control = true

func _on_TextEdit_text_entered(_new_text):
	Godot_list.Node_2D_control = false
