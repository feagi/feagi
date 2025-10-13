

class SensorDevice:
    def __init__(self, parent: "SensorCache", rust_register_func_name: str, rust_send_data_func_name: str, sensor_type=None):
        self.parent = parent
        self._rust_register_func_name = rust_register_func_name
        self._rust_send_data_func_name = rust_send_data_func_name
        self._sensor_type = sensor_type

        def register(*args, **kwargs):
            if self.parent._rust_cache:
                try:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.debug(f"[CACHE] Calling {self._rust_register_func_name}(*{args}, **{kwargs})")
                    # New API (sensor_*_absolute methods) are already sensor-specific, don't pass sensor_type
                    # Old API needs sensor_type as first arg
                    if self._rust_register_func_name.startswith('sensor_'):
                        # New API - sensor_type not needed (method is already sensor-specific)
                        result = getattr(self.parent._rust_cache, self._rust_register_func_name)(*args, **kwargs)
                    elif self._sensor_type is not None:
                        # Old API - pass sensor_type as first arg
                        result = getattr(self.parent._rust_cache, self._rust_register_func_name)(self._sensor_type, *args, **kwargs)
                    else:
                        result = getattr(self.parent._rust_cache, self._rust_register_func_name)(*args, **kwargs)
                    logger.debug(f"[CACHE] Registration successful: {self._rust_register_func_name}")
                    return result
                except AttributeError as e:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.error(f"[CACHE] Registration failed: {e}")
                    raise

        def send_data(*args, **kwargs):
            if self.parent._rust_cache:
                try:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.debug(f"[CACHE] Calling {self._rust_send_data_func_name} with {len(args)} args")
                    
                    # Debug: log arg types and first arg value (usually cortical_group)
                    if args:
                        logger.debug(f"[CACHE] Args: group={args[0] if len(args) > 0 else 'N/A'}, channel={args[1] if len(args) > 1 else 'N/A'}, data_type={type(args[2]).__name__ if len(args) > 2 else 'N/A'}")
                    
                    # New API (sensor_*_absolute methods) are already sensor-specific, don't pass sensor_type
                    # Old API needs sensor_type as first arg
                    if self._rust_send_data_func_name.startswith('sensor_'):
                        # New API - sensor_type not needed (method is already sensor-specific)
                        result = getattr(self.parent._rust_cache, self._rust_send_data_func_name)(*args, **kwargs)
                    elif self._sensor_type is not None:
                        # Old API - pass sensor_type as first arg
                        result = getattr(self.parent._rust_cache, self._rust_send_data_func_name)(self._sensor_type, *args, **kwargs)
                    else:
                        result = getattr(self.parent._rust_cache, self._rust_send_data_func_name)(*args, **kwargs)
                    logger.debug(f"[CACHE] Store successful: {self._rust_send_data_func_name}, result={result}")
                    return result
                except AttributeError as e:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.error(f"[CACHE] Store failed - method not found: {e}")
                    raise
                except Exception as e:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.error(f"[CACHE] Store failed with error: {e}")
                    raise

        def update_stage(*args, **kwargs):
            """Update stage properties for this sensor device.
            
            This method allows dynamic update of processing stage properties
            like segmentation parameters, as shown in the segmented_autogaze.py sample.
            
            Args are typically:
                cortical_group: int
                device_channel: int
                stage_index: int
                stage_properties: Stage properties object (e.g., ImageSegmentorStageProperties)
            """
            if self.parent._rust_cache:
                try:
                    # Try sensor-specific update method (new API style)
                    # e.g., sensor_register_segmented_vision_absolute -> sensor_update_stage_segmented_vision_absolute
                    update_func_name = self._rust_register_func_name.replace('register', 'update_stage')
                    if hasattr(self.parent._rust_cache, update_func_name):
                        func = getattr(self.parent._rust_cache, update_func_name)
                        return func(*args, **kwargs)
                    
                    # Try old API style
                    update_func_name = f"update_{self._rust_register_func_name.replace('register_', '')}_stage"
                    if hasattr(self.parent._rust_cache, update_func_name):
                        func = getattr(self.parent._rust_cache, update_func_name)
                        return func(*args, **kwargs)
                    
                    # Try generic update_stage method
                    if hasattr(self.parent._rust_cache, 'update_stage'):
                        func = getattr(self.parent._rust_cache, 'update_stage')
                        if self._sensor_type is not None:
                            return func(self._sensor_type, *args, **kwargs)
                        return func(*args, **kwargs)
                except AttributeError:
                    import logging
                    logger = logging.getLogger(__name__)
                    logger.debug(f"update_stage not available for {self._rust_register_func_name}")

        setattr(self, "register", register)
        setattr(self, "store", send_data)
        setattr(self, "write", send_data)  # Alias for store (used in samples)
        setattr(self, "update_stage", update_stage)
