extends Node

# The URL we will connect to
var api_ip_address = "192.168.50.218" # For API and Websocket
var api_port_address = "8001"         # For API port
var websocket_port_address = "9050"   # For Websocket Port
export var websocket_url = ""
#export var websocket_url = ""
var k8_ip_address = OS.get_environment("k8_server")
var green_light = false #Moved from feagi to here
var one_frame = ""
#var config_file= "res://ip_address.cfg" 

# Our WebSocketClient instance
var _client = WebSocketClient.new()

func _ready():

	websocket_url = "ws://" + str(api_ip_address) + ":" + websocket_port_address
	print("result: ", websocket_url)
	# Connect base signals to get notified of connection open, close, and errors.
	_client.connect("connection_closed", self, "_closed")
	_client.connect("connection_error", self, "_closed")
	_client.connect("connection_established", self, "_connected")
	# This signal is emitted when not using the Multiplayer API every time
	# a full packet is received.
	# Alternatively, you could check get_peer(1).get_available_packets() in a loop.
	_client.connect("data_received", self, "_on_data")

	# Initiate connection to the given URL.
	var err = _client.connect_to_url(websocket_url)
	if err != OK:
		print("Unable to connect")
	else:
		print("Connected")
		#set_process(false)

func _closed(was_clean = false):
	# was_clean will tell you if the disconnection was correctly notified
	# by the remote peer before closing the socket.
	print("Closed, clean: ", was_clean)
	set_process(false)

func _connected(proto = ""):
	# This is called on connection, "proto" will be the selected WebSocket
	# sub-protocol (which is optional)
	print("Connected with protocol: ", proto)
	# You MUST always use get_peer(1).put_packet to send data to server,
	# and not put_packet directly when not using the Multiplayernetwork_setting.
	green_light = true
	_client.get_peer(1).put_packet("{}".to_utf8())

func _on_data():
	# Print the received packet, you MUST always use get_peer(1).get_packet
	# to receive data from server, and not get_packet directly when not
	# using the Multiplayernetwork_setting.
	one_frame = _client.get_peer(1).get_packet().get_string_from_utf8()
	#print("Got data from server: ", one_frame)


func _process(_delta):
	# Call this in _process or _physics_process. Data transfer, and signals
	# emission will only happen when calling this function.
	_client.poll()

func send(data):
	if green_light:
		_client.get_peer(1).put_packet(data.to_utf8())

func disconnect_from_host():
	_client.disconnect_from_host(1000, "Close per request")
