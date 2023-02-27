
import os
from inf import runtime_data
from pydantic import BaseSettings


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
                     "http://127.0.0.1:6080",
                     "http://127.0.0.1:6081",
                     "http://127.0.0.1:3000",
                     "http://127.0.0.1:8000",
                     ]

    # overriding origin for fixing CORS problems
    origins: list = ["*"]

settings = Settings()
