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
    """
    app_name: str = Field(default="FEAGI API", env="FEAGI_APP_NAME")
    debug: bool = Field(default=False, env="FEAGI_DEBUG")
    api_prefix: str = Field(default="/api", env="FEAGI_API_PREFIX")
    version: str = Field(default="v1", env="FEAGI_API_VERSION")
    allowed_hosts: list[str] = Field(
        default=["http://localhost", "http://localhost:3000", "http://127.0.0.1", "http://127.0.0.1:3000"],
        env="FEAGI_ALLOWED_HOSTS",
    )

    class Config:
        env_file = ".env"
        case_sensitive = False


# Create a global settings instance
settings = Settings() 