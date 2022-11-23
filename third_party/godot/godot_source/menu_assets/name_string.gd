extends LineEdit

func _ready():
	visible = true
	
func _process(_delta):
	if visible == false:
		$name_string.value = ""
