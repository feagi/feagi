extends LineEdit

var value : float = 0.0 # export as needed

func _ready():
	visible = true

func _on_burst_value_text_changed(new_text): # "text_changed" signal handler
	if new_text.is_valid_float():
		value = float(new_text)
#	else: # optional rollback to last good one
#		self.text = str(value)
	if new_text == "":
		set_caret_column(1)


func _on_burst_value_focus_exited():
	release_focus()
	if text == "":
		text = str(0.0)
