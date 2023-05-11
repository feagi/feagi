extends Node

var socket = WebSocketPeer.new()
var one_frame 
# The URL we will connect to
 # For API IP
var api_ip_address = "127.0.0.1"
# For API port
var api_port_address = "8000"
# For Websocket port
var websocket_ip_address = "127.0.0.1"
# For Websocket Port
var websocket_port_address = "9050"
var state = ""

func _ready():
	socket.set_max_queued_packets(10000000)
	socket.inbound_buffer_size = 10000000
	socket.connect_to_url("ws://" + str(websocket_ip_address) + ":" + str(websocket_port_address))

func _process(_delta):
	socket.poll()
	state = socket.get_ready_state()
	if state == WebSocketPeer.STATE_OPEN:
		pass
#		socket.send("Hello!".to_utf8_buffer())
		while socket.get_available_packet_count():
			var socket_data = socket.get_packet()
			if socket_data.get_string_from_utf8() == "updated":
				one_frame = "updated"
			else:
				one_frame = str_to_var(socket_data.get_string_from_ascii())

	elif state == WebSocketPeer.STATE_CLOSING:
	# Keep polling to achieve proper close.
		pass
	elif state == WebSocketPeer.STATE_CLOSED:
		var code = socket.get_close_code()
		var reason = socket.get_close_reason()
		print("WebSocket closed with code: %d, reason %s. Clean: %s" % [code, reason, code != -1])
		set_process(false) # Stop processing.

func send(data):
	if state == WebSocketPeer.STATE_OPEN:
		socket.send(data.to_ascii_buffer())
