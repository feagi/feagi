extends ColorRect

var flag = false
var current = 0
var setpoint = false
var projectResolution: Vector2

func _ready():
	visible = false
#	projectResolution=Vector2(ProjectSettings.get_setting("display/window/size/width"), ProjectSettings.get_setting("display/window/size/width"))

func _process(_delta):
	if visible:
		if flag == false:
			if $Label.text != "0":
				flag = true
				timer()
			else:
				$TextEdit.text = "FEAGI IS NOT STARTED NOR LOADED!!"
	if $Label.text == "200":
		$Label.add_color_override("font_color", "#1f9239")
	else:
		$Label.add_color_override("font_color", "#ff0000")
#	print("rect size.x: ", rect_size.x, " and current: ", current, " and rect position: ", rect_position.x)
#	if flag:
#		if rect_position.x != 690:
#			rect_position.x 
#	if setpoint:
#		current = rect_position.x
#		setpoint = false
#	print(flag)

func timer():
	yield(get_tree().create_timer(3), "timeout")
	flag=false
	visible = false
