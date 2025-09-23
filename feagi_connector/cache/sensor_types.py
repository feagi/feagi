

class SensorDevice:
    def __init__(self, parent: "SensorCache", rust_register_func_name: str, rust_send_data_func_name: str, sensor_type=None):
        self.parent = parent
        self._rust_register_func_name = rust_register_func_name
        self._rust_send_data_func_name = rust_send_data_func_name
        self._sensor_type = sensor_type

        def register(*args, **kwargs):
            if self.parent._rust_cache:
                try:
                    if self._sensor_type is not None:
                        return getattr(self.parent._rust_cache, self._rust_register_func_name)(self._sensor_type, *args, **kwargs)
                    return getattr(self.parent._rust_cache, self._rust_register_func_name)(*args, **kwargs)
                except AttributeError:
                    pass

        def send_data(*args, **kwargs):
            if self.parent._rust_cache:
                try:
                    if self._sensor_type is not None:
                        return getattr(self.parent._rust_cache, self._rust_send_data_func_name)(self._sensor_type, *args, **kwargs)
                    return getattr(self.parent._rust_cache, self._rust_send_data_func_name)(*args, **kwargs)
                except AttributeError:
                    pass

        setattr(self, "register", register)
        setattr(self, "store", send_data)
