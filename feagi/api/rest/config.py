"""
Copyright 2025 Neuraville Inc.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
"""

"""
Configuration settings for the REST API.
"""

from pydantic import Field
from pydantic_settings import BaseSettings


class Settings(BaseSettings):
    """
    Configuration settings for the REST API.
    
    Attributes:
        app_name: Name of the application
        debug: Enable debug mode
        api_prefix: Prefix for API routes
        version: API version
        allowed_hosts: List of allowed hosts for CORS
        title: Title for FastAPI metadata
        description: Description for FastAPI metadata
        terms_of_service: Terms of service for FastAPI metadata
        contact: Contact information for FastAPI metadata
        license_info: License information for FastAPI metadata
        favicon_path: Path to favicon for FastAPI metadata
        origins: List of allowed origins for CORS
    """
    app_name: str = Field(default="FEAGI API", env="FEAGI_APP_NAME")
    debug: bool = Field(default=False, env="FEAGI_DEBUG")
    api_prefix: str = Field(default="/api", env="FEAGI_API_PREFIX")
    version: str = Field(default="v1", env="FEAGI_API_VERSION")
    allowed_hosts: list[str] = Field(
        default=["http://localhost", "http://localhost:3000", "http://127.0.0.1", "http://127.0.0.1:3000"],
        env="FEAGI_ALLOWED_HOSTS",
    )
    # FastAPI metadata attributes
    title: str = Field(default="FEAGI REST API")
    description: str = Field(default="FEAGI REST API for neural simulation and integration.")
    terms_of_service: str = Field(default="https://neuraville.com/terms/")
    contact: dict = Field(default={"name": "Neuraville Inc.", "url": "https://neuraville.com", "email": "info@neuraville.com"})
    license_info: dict = Field(default={"name": "Apache 2.0", "url": "https://www.apache.org/licenses/LICENSE-2.0.html"})
    favicon_path: str = Field(default="/static/favicon.ico")
    origins: list[str] = Field(default=["*"])

    class Config:
        env_file = ".env"
        case_sensitive = False


# Create a global settings instance
settings = Settings() 