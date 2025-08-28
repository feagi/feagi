

class SensorDevice:
    def __init__(self, parent: "SensorCache", rust_register_func_name: str, rust_send_data_func_name: str):
        self.parent = parent
        self._rust_register_func_name = rust_register_func_name
        self._rust_send_data_func_name = rust_send_data_func_name

        def register(*args, **kwargs):
            return getattr(self.parent._rust_cache, self._rust_register_func_name)(*args, **kwargs)

        def send_data(*args, **kwargs):
            return getattr(self.parent._rust_cache, self._rust_send_data_func_name)(*args, **kwargs)

        setattr(self, "register", register)
        setattr(self, "store", send_data)
