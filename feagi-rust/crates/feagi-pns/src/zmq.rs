// ZMQ Streams - manages all ZMQ communication

mod rest;
mod motor;
mod visualization;

pub use rest::RestStream;
pub use motor::MotorStream;
pub use visualization::VisualizationStream;

use crate::registration::RegistrationHandler;
use crate::PNSError;
use parking_lot::Mutex;
use std::sync::Arc;

/// ZMQ Streams coordinator
pub struct ZmqStreams {
    rest_stream: RestStream,
    motor_stream: MotorStream,
    viz_stream: VisualizationStream,
}

impl ZmqStreams {
    pub fn new(
        rest_address: &str,
        motor_address: &str,
        viz_address: &str,
        registration_handler: Arc<Mutex<RegistrationHandler>>,
    ) -> Result<Self, PNSError> {
        let context = Arc::new(zmq::Context::new());

        let mut rest_stream = RestStream::new(Arc::clone(&context), rest_address)
            .map_err(|e| PNSError::Zmq(format!("REST stream: {}", e)))?;

        // Set registration handler
        rest_stream.set_registration_handler(registration_handler);

        let motor_stream = MotorStream::new(Arc::clone(&context), motor_address)
            .map_err(|e| PNSError::Zmq(format!("Motor stream: {}", e)))?;

        let viz_stream = VisualizationStream::new(Arc::clone(&context), viz_address)
            .map_err(|e| PNSError::Zmq(format!("Viz stream: {}", e)))?;

        Ok(Self {
            rest_stream,
            motor_stream,
            viz_stream,
        })
    }

    pub fn start(&self) -> Result<(), PNSError> {
        self.rest_stream
            .start()
            .map_err(|e| PNSError::Zmq(format!("REST start: {}", e)))?;
        self.motor_stream
            .start()
            .map_err(|e| PNSError::Zmq(format!("Motor start: {}", e)))?;
        self.viz_stream
            .start()
            .map_err(|e| PNSError::Zmq(format!("Viz start: {}", e)))?;
        Ok(())
    }

    pub fn stop(&self) -> Result<(), PNSError> {
        self.rest_stream
            .stop()
            .map_err(|e| PNSError::Zmq(format!("REST stop: {}", e)))?;
        self.motor_stream
            .stop()
            .map_err(|e| PNSError::Zmq(format!("Motor stop: {}", e)))?;
        self.viz_stream
            .stop()
            .map_err(|e| PNSError::Zmq(format!("Viz stop: {}", e)))?;
        Ok(())
    }

    /// Publish visualization data to ZMQ subscribers
    pub fn publish_visualization(&self, data: &[u8]) -> Result<(), PNSError> {
        self.viz_stream
            .publish(data)
            .map_err(|e| PNSError::Zmq(format!("Viz publish: {}", e)))
    }
}

