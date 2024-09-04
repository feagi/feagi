#
# Copyright 2016-Present Neuraville Inc. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# ==============================================================================

import os
from pydantic import BaseSettings

env_ip = os.getenv("ip_server")
if not env_ip:
    env_ip = "placeholder"
env_ip_api = "http://" + env_ip + ":" + "8000"
env_ip_gazebo = "http://" + env_ip + ":" + "6080"
env_ip_godot = "http://" + env_ip + ":" + "6081"


class Settings(BaseSettings):
    description: str = """FEAGI REST API will help you integrate FEAGI into other applications and provides a 
    programmatic method to interact with 
    FEAGI."""
    title: str = "FEAGI API Documentation"
    version: str = "1"
    terms_of_service: str = "http://feagi.org"
    favicon_path: str = "favicon.svg"
    contact: dict = {
        "name": "FEAGI Community",
        "url": "http://feagi.org",
        "email": "info@feagi.org",
    }
    license_info: dict = {
        "name": "Apache 2.0",
        "url": "https://www.apache.org/licenses/LICENSE-2.0.html",
    }
    origins: list = ["http://localhost:6080",
                     "http://localhost:6081",
                     "http://localhost:3000",
                     "http://localhost:8000",
                     "http://localhost:8080",
                     env_ip_api,
                     env_ip_gazebo,
                     env_ip_godot,
                     "http://godot-user1/",
                     "http://godot-user1:8000",
                     "http://godot-user1:80",
                     "http://godot-user1:6081",
                     "http://godot-user1:8080",
                     "http://127.0.0.1:6080",
                     "http://127.0.0.1:6081",
                     "http://127.0.0.1:3000",
                     "http://127.0.0.1:8000",
                     "http://127.0.0.1:8080"
                     ]

    # overriding origin for fixing CORS problems
    origins: list = ["*"]

settings = Settings()
