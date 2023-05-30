extends Node

var socket = WebSocketPeer.new()
var one_frame 
# The URL we will connect to
var api_ip_address = "127.0.0.1" # Setting the global variabl3
# For API port
var api_port_address = "8000"
# For Websocket port
var websocket_ip_address = "127.0.0.1" # Setting the global variable
# For Websocket Port
var websocket_port_address = "9050"
var state = ""

func _ready():
	var ip_result = JavaScriptBridge.eval(""" 
		function getIPAddress() {
			var url_string = window.location.href;
			var url = new URL(url_string);
			const searchParams = new URLSearchParams(url.search);
			const ipAddress = searchParams.get("ip_address");
			return ipAddress;
		}
		getIPAddress();
		""")
	var port_disabled = JavaScriptBridge.eval(""" 
		function get_port() {
			var url_string = window.location.href;
			var url = new URL(url_string);
			const searchParams = new URLSearchParams(url.search);
			const ipAddress = searchParams.get("port_disabled");
			return ipAddress;
		}
		get_port();
		""")
	var full_url = JavaScriptBridge.eval(""" 
		function get_port() {
			var url_string = window.location.href;
			var url = new URL(url_string);
			const searchParams = new URLSearchParams(url.search);
			const ipAddress = searchParams.get("full_url");
			return ipAddress;
		}
		get_port();
		""")
	if ip_result == "" or ip_result == null:
		websocket_ip_address = "127.0.0.1"
		api_ip_address = "127.0.0.1"
	else:
		websocket_ip_address = ip_result
		api_ip_address = ip_result
	print("javascript ip: ", ip_result, " port: ", port_disabled, " full: ", full_url)
	socket.set_max_queued_packets(10000000)
	socket.inbound_buffer_size = 10000000
	if full_url != null:
		print("connecting to: ", full_url)
		socket.connect_to_url(full_url)
	elif port_disabled == "true":
		print("connecting to: ", "ws://" + str(websocket_ip_address))
		socket.connect_to_url("ws://" + str(websocket_ip_address))
	else:
		print("connecting to: ", "ws://" + str(websocket_ip_address) + ":" + str(websocket_port_address))
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
