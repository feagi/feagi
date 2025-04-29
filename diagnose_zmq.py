#!/usr/bin/env python
"""
ZMQ Diagnostics Tool

This script diagnoses issues with PyZMQ installation to help troubleshoot
the "module 'zmq' has no attribute 'Context'" error.
"""
import os
import sys
import importlib
import importlib.util
import site
import subprocess
import logging
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger("zmq_diagnostics")

def check_python_version():
    """Check Python version and executable path."""
    logger.info(f"Python executable: {sys.executable}")
    logger.info(f"Python version: {sys.version}")
    logger.info(f"Python path: {sys.path}")
    
    # Check if running in a virtual environment
    in_venv = hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)
    logger.info(f"Running in virtual environment: {in_venv}")
    if in_venv:
        logger.info(f"Virtual environment path: {sys.prefix}")
    
    return in_venv

def check_pyzmq_installation():
    """Check PyZMQ installation details."""
    try:
        logger.info("Checking PyZMQ installation...")
        
        # Method 1: Use pkg_resources
        try:
            import pkg_resources
            zmq_dist = pkg_resources.get_distribution("pyzmq")
            logger.info(f"PyZMQ version (pkg_resources): {zmq_dist.version}")
            logger.info(f"PyZMQ location: {zmq_dist.location}")
        except Exception as e:
            logger.warning(f"Failed to get PyZMQ info via pkg_resources: {e}")
        
        # Method 2: Use importlib to find the module location
        try:
            zmq_spec = importlib.util.find_spec("zmq")
            if zmq_spec:
                logger.info(f"ZMQ module location: {zmq_spec.origin}")
                logger.info(f"ZMQ submodule locations: {[Path(p).parent for p in (zmq_spec.submodule_search_locations or [])]}")
        except Exception as e:
            logger.warning(f"Failed to find ZMQ module location: {e}")
        
        # Method 3: Try pip list
        try:
            result = subprocess.run(
                [sys.executable, "-m", "pip", "show", "pyzmq"], 
                capture_output=True, 
                text=True
            )
            if result.returncode == 0:
                logger.info(f"Pip package info:\n{result.stdout}")
            else:
                logger.warning(f"Pip couldn't find pyzmq package: {result.stderr}")
        except Exception as e:
            logger.warning(f"Failed to run pip: {e}")
            
        return True
    except Exception as e:
        logger.error(f"Error checking PyZMQ installation: {e}")
        return False

def check_zmq_internals():
    """Check ZMQ module internals and why Context may be missing."""
    logger.info("Checking ZMQ module internals...")
    
    try:
        # Try importing zmq
        import zmq
        logger.info(f"ZMQ successfully imported from: {getattr(zmq, '__file__', 'unknown')}")
        logger.info(f"ZMQ version: {getattr(zmq, 'pyzmq_version', 'unknown')}")
        
        # Check if Context exists
        if hasattr(zmq, 'Context'):
            logger.info(f"ZMQ.Context exists: {zmq.Context}")
            logger.info("✅ ZMQ Context is available")
        else:
            logger.warning("❌ ZMQ Context attribute is missing!")
            
            # Try to find where Context should be
            logger.info("Looking for Context in ZMQ submodules...")
            
            # Check zmq.sugar module
            if hasattr(zmq, 'sugar'):
                logger.info("ZMQ sugar module exists")
                if hasattr(zmq.sugar, 'context') and hasattr(zmq.sugar.context, 'Context'):
                    logger.info("Found Context in zmq.sugar.context")
            
            # Check zmq.core module
            if hasattr(zmq, 'core'):
                logger.info("ZMQ core module exists")
                if hasattr(zmq.core, 'context') and hasattr(zmq.core.context, 'Context'):
                    logger.info("Found Context in zmq.core.context")
            
            # Try direct import
            try:
                from zmq import Context
                logger.info("Successfully imported Context directly from zmq")
            except ImportError:
                logger.warning("Could not import Context directly from zmq")
                
            # Try all possible paths
            for module_name in ["zmq.sugar.context", "zmq.core.context"]:
                try:
                    module = importlib.import_module(module_name)
                    if hasattr(module, 'Context'):
                        logger.info(f"Found Context in {module_name}")
                except ImportError:
                    logger.warning(f"Could not import {module_name}")
        
        # Log all attributes of zmq module
        logger.info(f"ZMQ attributes: {dir(zmq)}")
        
        return True
    except ImportError as e:
        logger.error(f"Failed to import ZMQ: {e}")
        return False
    except Exception as e:
        logger.error(f"Error checking ZMQ internals: {e}")
        return False

def test_zmq_context_creation():
    """Test ZMQ Context creation with different methods."""
    logger.info("Testing ZMQ Context creation...")
    
    # Method 1: Standard import and context creation
    try:
        import zmq
        context = zmq.Context()
        logger.info(f"✅ Successfully created Context using standard method: {context}")
    except AttributeError:
        logger.warning("❌ Failed to create Context using standard method: AttributeError")
    except Exception as e:
        logger.warning(f"❌ Failed to create Context using standard method: {type(e).__name__}: {e}")
    
    # Method 2: Direct import of Context
    try:
        from zmq import Context
        context = Context()
        logger.info(f"✅ Successfully created Context using direct import: {context}")
    except ImportError:
        logger.warning("❌ Failed to import Context directly")
    except Exception as e:
        logger.warning(f"❌ Failed to create Context using direct import: {type(e).__name__}: {e}")
    
    # Method 3: Try from sugar module
    try:
        import zmq.sugar.context
        context = zmq.sugar.context.Context()
        logger.info(f"✅ Successfully created Context from sugar module: {context}")
    except (ImportError, AttributeError):
        logger.warning("❌ Failed to create Context from sugar module")
    except Exception as e:
        logger.warning(f"❌ Failed to create Context from sugar module: {type(e).__name__}: {e}")
    
    # Method 4: Try from core module
    try:
        import zmq.core.context
        context = zmq.core.context.Context()
        logger.info(f"✅ Successfully created Context from core module: {context}")
    except (ImportError, AttributeError):
        logger.warning("❌ Failed to create Context from core module")
    except Exception as e:
        logger.warning(f"❌ Failed to create Context from core module: {type(e).__name__}: {e}")

def test_cythonized_components():
    """Test if Cython components are properly built and loaded."""
    logger.info("Testing Cython components...")
    
    try:
        import zmq
        
        # Check for _device attribute which is often a Cython extension
        if hasattr(zmq, '_device'):
            logger.info("✅ ZMQ has _device attribute (Cython extension)")
        else:
            logger.warning("❌ ZMQ missing _device attribute (potential Cython issue)")
        
        # Try to import known Cython modules
        cython_modules = [
            "zmq.backend.cython", 
            "zmq.backend.cython.socket",
            "zmq.backend.cython.context",
            "zmq.backend.cython.message"
        ]
        
        for module_name in cython_modules:
            try:
                module = importlib.import_module(module_name)
                logger.info(f"✅ Successfully imported {module_name}")
            except ImportError as e:
                logger.warning(f"❌ Failed to import {module_name}: {e}")
        
        return True
    except Exception as e:
        logger.error(f"Error testing Cython components: {e}")
        return False

def suggest_fixes():
    """Suggest possible fixes based on diagnostic results."""
    logger.info("\n=== Possible fixes ===")
    
    # Check if in virtual environment
    in_venv = hasattr(sys, 'real_prefix') or (hasattr(sys, 'base_prefix') and sys.base_prefix != sys.prefix)
    
    if in_venv:
        logger.info("1. Try reinstalling PyZMQ in your virtual environment:")
        logger.info("   pip uninstall -y pyzmq")
        logger.info("   pip install pyzmq")
    else:
        logger.info("1. Create and use a virtual environment:")
        logger.info("   python -m venv .venv")
        logger.info("   source .venv/bin/activate  # On Unix/macOS")
        logger.info("   .venv\\Scripts\\activate     # On Windows")
        logger.info("   pip install -r requirements.txt")
    
    logger.info("\n2. Check for conflicts with system PyZMQ:")
    logger.info("   Make sure you're using the Python from the virtual environment")
    
    logger.info("\n3. Docker container solution:")
    logger.info("   Consider using a Docker container with a clean environment")
    logger.info("   See the documentation for containerization instructions")
    
    logger.info("\n4. Run FEAGI with `--debug` flag for more detailed logging:")
    logger.info("   ./.venv/bin/feagi --debug")

def main():
    """Run all diagnostic checks."""
    logger.info("=== ZMQ Diagnostics Tool ===")
    
    in_venv = check_python_version()
    logger.info("\n=== PyZMQ Installation ===")
    check_pyzmq_installation()
    logger.info("\n=== ZMQ Module Internals ===")
    check_zmq_internals()
    logger.info("\n=== ZMQ Context Creation Tests ===")
    test_zmq_context_creation()
    logger.info("\n=== Cython Component Tests ===")
    test_cythonized_components()
    
    logger.info("\n=== Summary ===")
    suggest_fixes()
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 