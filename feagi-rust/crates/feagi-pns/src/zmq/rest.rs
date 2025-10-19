// REST stream for agent registration, heartbeat, and deregistration
// Uses ROUTER socket pattern for request-reply with agent identity tracking

use crate::registration::{RegistrationHandler, RegistrationRequest};
use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use std::sync::Arc;
use std::thread;

/// REST request from agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestRequest {
    pub method: String,
    pub path: String,
    pub body: Option<serde_json::Value>,
}

/// REST response to agent
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RestResponse {
    pub status: u16,
    pub body: serde_json::Value,
}

/// REST stream managing agent registration and lifecycle
#[derive(Clone)]
pub struct RestStream {
    context: Arc<zmq::Context>,
    bind_address: String,
    socket: Arc<Mutex<Option<zmq::Socket>>>,
    running: Arc<Mutex<bool>>,
    registration_handler: Arc<Mutex<Option<Arc<Mutex<RegistrationHandler>>>>>,
}

impl RestStream {
    /// Create a new REST stream
    pub fn new(context: Arc<zmq::Context>, bind_address: &str) -> Result<Self, String> {
        Ok(Self {
            context,
            bind_address: bind_address.to_string(),
            socket: Arc::new(Mutex::new(None)),
            running: Arc::new(Mutex::new(false)),
            registration_handler: Arc::new(Mutex::new(None)),
        })
    }

    /// Set the registration handler
    pub fn set_registration_handler(&mut self, handler: Arc<Mutex<RegistrationHandler>>) {
        *self.registration_handler.lock() = Some(handler);
    }

    /// Start the REST stream
    pub fn start(&self) -> Result<(), String> {
        if *self.running.lock() {
            return Err("REST stream already running".to_string());
        }

        // Create ROUTER socket
        let socket = self
            .context
            .socket(zmq::ROUTER)
            .map_err(|e| e.to_string())?;

        socket.set_linger(1000).map_err(|e| e.to_string())?;
        socket
            .set_router_mandatory(false)
            .map_err(|e| e.to_string())?;

        socket.bind(&self.bind_address).map_err(|e| e.to_string())?;

        *self.socket.lock() = Some(socket);
        *self.running.lock() = true;

        println!("🦀 [ZMQ-REST] Listening on {}", self.bind_address);

        // Start processing loop
        self.start_processing_loop();

        Ok(())
    }

    /// Stop the REST stream
    pub fn stop(&self) -> Result<(), String> {
        *self.running.lock() = false;
        *self.socket.lock() = None;
        Ok(())
    }

    /// Start the background processing loop
    fn start_processing_loop(&self) {
        let socket = Arc::clone(&self.socket);
        let running = Arc::clone(&self.running);
        let registration_handler = Arc::clone(&self.registration_handler);

        thread::spawn(move || {
            println!("🦀 [ZMQ-REST] Processing loop started");

            while *running.lock() {
                let sock_guard = socket.lock();
                let sock = match sock_guard.as_ref() {
                    Some(s) => s,
                    None => {
                        drop(sock_guard);
                        thread::sleep(std::time::Duration::from_millis(100));
                        continue;
                    }
                };

                // Poll for messages
                let poll_items = &mut [sock.as_poll_item(zmq::POLLIN)];
                if let Err(e) = zmq::poll(poll_items, 100) {
                    eprintln!("🦀 [ZMQ-REST] [ERR] Poll error: {}", e);
                    continue;
                }

                if !poll_items[0].is_readable() {
                    drop(sock_guard);
                    continue;
                }

                // Receive multipart message: [identity, delimiter, request_json]
                let mut msg_parts = Vec::new();
                let mut more = true;

                while more {
                    let mut msg = zmq::Message::new();
                    match sock.recv(&mut msg, 0) {
                        Ok(()) => {
                            msg_parts.push(msg);
                            more = sock.get_rcvmore().unwrap_or(false);
                        }
                        Err(e) => {
                            eprintln!("🦀 [ZMQ-REST] [ERR] Receive error: {}", e);
                            break;
                        }
                    }
                }

                drop(sock_guard);

                // Process request
                if msg_parts.len() >= 3 {
                    let identity = msg_parts[0].to_vec();
                    let request_json = String::from_utf8_lossy(&msg_parts[2]).to_string();

                    let response_json = Self::process_request(&registration_handler, &request_json);

                    if let Err(e) = Self::send_response(&socket, identity, response_json) {
                        eprintln!("🦀 [ZMQ-REST] [ERR] Failed to send response: {}", e);
                    }
                }
            }

            println!("🦀 [ZMQ-REST] Processing loop stopped");
        });
    }

    /// Process a request using the registration handler
    fn process_request(
        handler_mutex: &Arc<Mutex<Option<Arc<Mutex<RegistrationHandler>>>>>,
        request_json: &str,
    ) -> String {
        let handler_guard = handler_mutex.lock();
        let handler = match handler_guard.as_ref() {
            Some(h) => h,
            None => {
                return serde_json::json!({
                    "status": 503,
                    "body": {"error": "Service unavailable"}
                })
                .to_string();
            }
        };

        // Parse request
        let request: RestRequest = match serde_json::from_str(request_json) {
            Ok(req) => req,
            Err(e) => {
                return serde_json::json!({
                    "status": 400,
                    "body": {"error": format!("Invalid request: {}", e)}
                })
                .to_string();
            }
        };

        println!(
            "🦀 [ZMQ-REST] {} {}",
            request.method, request.path
        );

        // Route request
        let response = match (request.method.as_str(), request.path.as_str()) {
            ("POST", "/v1/agent/register") => {
                Self::handle_registration(handler, request.body)
            }
            ("POST", "/v1/agent/heartbeat") => {
                Self::handle_heartbeat(handler, request.body)
            }
            ("DELETE", "/v1/agent/deregister") => {
                Self::handle_deregistration(handler, request.body)
            }
            _ => {
                serde_json::json!({
                    "status": 404,
                    "body": {"error": "Not found"}
                })
            }
        };

        response.to_string()
    }

    /// Handle registration
    fn handle_registration(
        handler: &Arc<Mutex<RegistrationHandler>>,
        body: Option<serde_json::Value>,
    ) -> serde_json::Value {
        let body = match body {
            Some(b) => b,
            None => {
                return serde_json::json!({
                    "status": 400,
                    "body": {"error": "Missing request body"}
                });
            }
        };

        let request: RegistrationRequest = match serde_json::from_value(body) {
            Ok(r) => r,
            Err(e) => {
                return serde_json::json!({
                    "status": 400,
                    "body": {"error": format!("Invalid registration request: {}", e)}
                });
            }
        };

        match handler.lock().process_registration(request) {
            Ok(response) => serde_json::json!({
                "status": 200,
                "body": response
            }),
            Err(e) => serde_json::json!({
                "status": 500,
                "body": {"error": e}
            }),
        }
    }

    /// Handle heartbeat
    fn handle_heartbeat(
        handler: &Arc<Mutex<RegistrationHandler>>,
        body: Option<serde_json::Value>,
    ) -> serde_json::Value {
        let agent_id = body
            .and_then(|b| b.get("agent_id").and_then(|v| v.as_str()).map(String::from))
            .unwrap_or_default();

        match handler.lock().process_heartbeat(&agent_id) {
            Ok(_) => {
                serde_json::json!({
                    "status": 200,
                    "body": {"message": "ok"}
                })
            },
            Err(e) => {
                eprintln!("🦀 [PNS] Heartbeat failed for {}: {}", agent_id, e);
                serde_json::json!({
                    "status": 404,
                    "body": {"error": e}
                })
            },
        }
    }

    /// Handle deregistration
    fn handle_deregistration(
        handler: &Arc<Mutex<RegistrationHandler>>,
        body: Option<serde_json::Value>,
    ) -> serde_json::Value {
        let agent_id = body
            .and_then(|b| b.get("agent_id").and_then(|v| v.as_str()).map(String::from))
            .unwrap_or_default();

        match handler.lock().process_deregistration(&agent_id) {
            Ok(_) => serde_json::json!({
                "status": 200,
                "body": {"message": "ok"}
            }),
            Err(e) => serde_json::json!({
                "status": 404,
                "body": {"error": e}
            }),
        }
    }

    /// Send response
    fn send_response(
        socket_mutex: &Arc<Mutex<Option<zmq::Socket>>>,
        identity: Vec<u8>,
        response_json: String,
    ) -> Result<(), String> {
        let sock_guard = socket_mutex.lock();
        let sock = match sock_guard.as_ref() {
            Some(s) => s,
            None => return Err("Socket not available".to_string()),
        };

        sock.send(&identity, zmq::SNDMORE).map_err(|e| e.to_string())?;
        sock.send(&Vec::<u8>::new(), zmq::SNDMORE).map_err(|e| e.to_string())?;
        sock.send(response_json.as_bytes(), 0).map_err(|e| e.to_string())?;

        Ok(())
    }
}
