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
		$add_cortical_button.visible = false
		$move_cortical.visible = false
		$add_cortical_button/cortical_menu.visible = false
		$move_cortical/cortical_menu.visible = false
		
	else:
		$add_cortical_button.visible = true
		$menu_background.visible = true
		$move_cortical.visible = true

func _process(_delta):
	check_esc()

func _on_Add_it_pressed():
	print("x: ", $add_cortical_button/cortical_menu/X.value)
	print("y: ", $add_cortical_button/cortical_menu/Y.value)
	print("type: ", $add_cortical_button/cortical_menu/OptionButton.selected)
	print("name: ", $add_cortical_button/cortical_menu/name_string.text)


func _on_add_cortical_button_pressed():
	if pressed != true:
		pressed = true
		$move_cortical.visible = false
		$add_cortical_button/cortical_menu.visible = true
	else:
		pressed = false
		$move_cortical.visible = true
		$add_cortical_button/cortical_menu.visible = false


func _on_move_cortical_pressed():
	if pressed != true:
		pressed = true
		$add_cortical_button.visible = false
		$move_cortical/cortical_menu.visible = true
	else:
		pressed = false
		$add_cortical_button.visible = true
		$move_cortical/cortical_menu.visible = false
