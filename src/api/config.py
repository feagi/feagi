
import os
from inf import runtime_data
from pydantic import BaseSettings

k8_server = os.getenv("ip_server")
if not k8_server:
    k8_server = "placeholder"
k8_server_api = "http://" + k8_server + ":" + "8000"
k8_server_gazebo = "http://" + k8_server + ":" + "6080"
k8_server_gadot = "http://" + k8_server + ":" + "6081"


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
        "email": "info@neuraville.com",
    }
    license_info: dict = {
        "name": "Apache 2.0",
        "url": "https://www.apache.org/licenses/LICENSE-2.0.html",
    }
    origins: list = ["http://localhost:6080",
                     "http://localhost:6081",
                     "http://localhost:3000",
                     "http://localhost:8000",
                     k8_server_api,
                     k8_server_gazebo,
                     k8_server_gadot]


settings = Settings()
