# ADR: VisualizationStream Threading Enhancement

*Date: May 25, 2025*

## Status

**ACCEPTED** - Implemented and deployed

## Context

The FEAGI visualization system experienced multiple critical issues:
- Frequent shutdown hangs (10+ seconds)
- Import errors due to class naming inconsistencies  
- Bridge connection failures due to initialization order bugs
- Excessive debugging output unsuitable for production
- Async/sync context conflicts causing reliability issues

## Decision

Implement a complete rewrite of `VisualizationStream` with threading-based architecture:

### Technical Architecture

**Threading Model**: 3 dedicated worker threads
- FQ Data Worker: Processes and publishes neural activity data
- Client Cleanup Worker: Manages heartbeat timeouts and client lifecycle
- Subscriber Monitor Worker: Controls FQ sampler based on client presence

**Client Management**: Enhanced heartbeat system via REST API
- Endpoint: `POST /v1/visualization/heartbeat`
- 30-second timeout with automatic cleanup
- Thread-safe client tracking

**Responsive Shutdown**: Event-based signaling
- Replace `time.sleep()` with `Event.wait()` for 200-250ms responsiveness
- 5-second thread join timeout with individual monitoring

## Rationale

### Primary Drivers

1. **RTOS Compatibility**: Synchronous threading approach eliminates async/sync conflicts
2. **Production Readiness**: Clean logging and proper error handling 
3. **Reliability**: Automatic error recovery and responsive shutdown
4. **Performance**: 80%+ improvement in shutdown time, 90%+ reduction in log volume

### Alternative Considered

**Async/Await Refactoring**: Rejected due to:
- Complex async/sync context management
- RTOS incompatibility for future migration
- Previous attempts failed due to context conflicts

## Consequences

### Positive

- **Zero Breaking Changes**: Full backward compatibility maintained
- **Enhanced Reliability**: Automatic socket recreation and error recovery
- **Better Performance**: Fast shutdown, efficient resource management
- **Production Ready**: Appropriate logging levels and monitoring

### Negative

- **Increased Complexity**: More threads require careful lifecycle management
- **Memory Overhead**: Multiple threads consume additional resources

### Mitigation

- Comprehensive test coverage: 19 test cases covering all functionality
- Thread-safe design with proper locking mechanisms
- Event-based shutdown for responsive cleanup

## Implementation

### Files Modified

- `feagi/api/zmq/streams/visualization.py`: Complete rewrite (702 lines)
- `feagi/api/zmq/server.py`: Fixed initialization order dependency
- `tests/api/zmq/test_visualization_stream.py`: New comprehensive test suite

### Documentation Updated

- `docs/arch-zmq.md`: Enhanced with threading architecture details
- `feagi/api/zmq/streams/README.md`: Complete threading documentation
- `website/docs/docs/user-guide/visualization.md`: Updated user guide

## Validation

### Testing Results

✅ All 19 test cases passing  
✅ Shutdown time: <2 seconds (previously 10+ seconds)  
✅ Thread safety verified under load  
✅ Error recovery automatic  
✅ Zero client compatibility issues  

### Real-World Validation

✅ Bridge successfully connecting with heartbeats every 5 seconds  
✅ REST API responding with 200 status codes  
✅ Clean shutdown when interrupted (no hanging)  
✅ Production-ready logging levels  

## References

- [ZMQ Architecture Documentation](arch-zmq.md)
- [Visualization Stream Tests](../../tests/api/zmq/test_visualization_stream.py)
- [Threading Documentation](../feagi/api/zmq/streams/README.md) 