"""FastAPI application for FEAGI."""
from typing import Dict, List, Optional

from fastapi import FastAPI, HTTPException, Depends
from pydantic import BaseModel

from feagi.core.feagi import FEAGI

class ModelRequest(BaseModel):
    """Request model for creating a new model."""
    name: str
    model_type: str = "default"

class TrainingRequest(BaseModel):
    """Request model for training a model."""
    data: List[Dict]
    epochs: int = 10

class PredictionRequest(BaseModel):
    """Request model for making predictions."""
    data: List[Dict]

class ModelResponse(BaseModel):
    """Response model for model information."""
    name: str
    model_type: str
    created_at: str
    updated_at: Optional[str] = None

def get_feagi() -> FEAGI:
    """
    Get the FEAGI instance.
    
    Returns:
        FEAGI instance.
    """
    return FEAGI()

def create_app() -> FastAPI:
    """
    Create a FastAPI application for FEAGI.
    
    Returns:
        FastAPI application.
    """
    app = FastAPI(
        title="FEAGI API",
        description="API for the Framework for Evolutionary Artificial General Intelligence",
        version="0.1.0",
    )

    @app.get("/")
    async def root():
        """Root endpoint."""
        return {"message": "Welcome to FEAGI API"}
    
    @app.get("/models", response_model=List[str])
    async def list_models(feagi: FEAGI = Depends(get_feagi)):
        """List all models."""
        return feagi.list_models()
    
    @app.post("/models", response_model=ModelResponse)
    async def create_model(request: ModelRequest, feagi: FEAGI = Depends(get_feagi)):
        """Create a new model."""
        model = feagi.create_model(request.name, request.model_type)
        return {
            "name": model.name,
            "model_type": model.model_type,
            "created_at": model.metadata["created_at"],
        }
    
    @app.get("/models/{name}", response_model=ModelResponse)
    async def get_model(name: str, feagi: FEAGI = Depends(get_feagi)):
        """Get model by name."""
        model = feagi.get_model(name)
        if model is None:
            raise HTTPException(status_code=404, detail=f"Model {name} not found")
        
        return {
            "name": model.name,
            "model_type": model.model_type,
            "created_at": model.metadata["created_at"],
            "updated_at": model.metadata.get("updated_at"),
        }
    
    @app.post("/models/{name}/train")
    async def train_model(name: str, request: TrainingRequest, feagi: FEAGI = Depends(get_feagi)):
        """Train a model."""
        model = feagi.get_model(name)
        if model is None:
            raise HTTPException(status_code=404, detail=f"Model {name} not found")
        
        metrics = model.train(request.data, request.epochs)
        return {"message": f"Model {name} trained successfully", "metrics": metrics}
    
    @app.post("/models/{name}/predict")
    async def predict(name: str, request: PredictionRequest, feagi: FEAGI = Depends(get_feagi)):
        """Make predictions using a model."""
        model = feagi.get_model(name)
        if model is None:
            raise HTTPException(status_code=404, detail=f"Model {name} not found")
        
        predictions = model.predict(request.data)
        return {"predictions": predictions.tolist()}
    
    @app.delete("/models/{name}")
    async def delete_model(name: str, feagi: FEAGI = Depends(get_feagi)):
        """Delete a model."""
        success = feagi.remove_model(name)
        if not success:
            raise HTTPException(status_code=404, detail=f"Model {name} not found")
        
        return {"message": f"Model {name} deleted successfully"}
    
    return app 