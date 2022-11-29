extends Control

var pressed = false

func _ready():
	visible = true

func check_esc():
	if Input.is_action_just_pressed("esc"):
		if visible:
			visible = false
		else:
			visible = true

func _on_Button_pressed():
	if $menu_background.visible:
		$menu_background.visible = false
		$cortical_menu.visible = false

		
	else:
		$cortical_menu.visible = true
		$menu_background.visible = true


func _process(_delta):
	check_esc()

func _on_Add_it_pressed():
	print("x: ", $cortical_menu/X.value)
	print("y: ", $cortical_menu/Y.value)
	print("type: ", $cortical_menu/OptionButton.selected)
	print("name: ", $cortical_menu/name_string.text)
