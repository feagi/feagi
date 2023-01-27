extends SpinBox

onready var line = get_line_edit()

func _ready():
	line.connect("text_entered", self, "_on_text_entered")


func _on_X_mouse_exited():
	line.release_focus()

func _on_Y_mouse_exited():
	line.release_focus()


func _on_Z_mouse_exited():
	line.release_focus()


func _on_W_mouse_exited():
	line.release_focus()


func _on_D_mouse_exited():
	line.release_focus()


func _on_H_mouse_exited():
	line.release_focus()

func _on_neuron_count_mouse_exited():
	line.release_focus()

func _on_syn_mouse_exited():
	line.release_focus()

func _on_pst_syn_mouse_exited():
	line.release_focus()

func _on_pst_syn_max_mouse_exited():
	line.release_focus()

func _on_fire_mouse_exited():
	line.release_focus()

func _on_refa_mouse_exited():
	line.release_focus()

func _on_cfr_mouse_exited():
	line.release_focus()

func _on_snze_mouse_exited():
	line.release_focus()

func _on_dege_mouse_exited():
	line.release_focus()

func _on_plst_mouse_exited():
	line.release_focus()

func _on_over1_mouse_exited():
	line.release_focus()

func _on_over2_mouse_exited():
	line.release_focus()

func _on_over3_mouse_exited():
	line.release_focus()

func _on_count_spinbox_mouse_exited():
	line.release_focus()

func _on_X_SpinBox_mouse_exited():
	line.release_focus()

func _on_Y_Spinbox_mouse_exited():
	line.release_focus()

func _on_Z_Spinbox_mouse_exited():
	line.release_focus()

func _on_D_Spinbox_mouse_exited():
	line.release_focus()

func _on_W_Spinbox_mouse_exited():
	line.release_focus()

func _on_y_spinbox_mouse_exited():
	line.release_focus()

func _on_z_spinbox_mouse_exited():
	line.release_focus()

func _on_x_spinbox_mouse_exited():
	line.release_focus()
