"""Data structure utilities for FEAGI.

This module provides utilities for creating data structures that are more compatible
with Rust's ownership model and easier to port to Rust.
"""
from dataclasses import dataclass, field, asdict, is_dataclass
from typing import Dict, List, Optional, Any, TypeVar, Generic, Type, Union, Tuple, ClassVar
import json
import inspect
from enum import Enum, auto


# Type variable for generic types
T = TypeVar('T')


class OwnershipType(Enum):
    """Ownership type for fields in a Rust-compatible dataclass."""
    
    # The field owns the data (equivalent to T in Rust)
    OWNED = auto()
    
    # The field borrows the data (equivalent to &T in Rust)
    BORROWED = auto()
    
    # The field mutably borrows the data (equivalent to &mut T in Rust)
    MUTABLE_BORROWED = auto()


@dataclass
class RustField:
    """Metadata for a field in a Rust-compatible dataclass."""
    
    name: str
    type_hint: Type
    ownership: OwnershipType = OwnershipType.OWNED
    doc: str = ""
    thread_safe: bool = False


@dataclass
class RustCompatible:
    """Mixin for dataclasses that are designed to be Rust-compatible."""
    
    # Class variable to store Rust-specific field metadata
    __rust_fields__: ClassVar[Dict[str, RustField]] = field(default_factory=dict)
    
    def __post_init__(self):
        """Initialize Rust-specific field metadata."""
        # This is called after the dataclass is initialized
        if not hasattr(self.__class__, "__rust_fields__"):
            self.__class__.__rust_fields__ = {}
        
        # If not already populated, analyze the class
        if not self.__class__.__rust_fields__:
            self._analyze_fields()
    
    def _analyze_fields(self):
        """Analyze fields to extract Rust-specific metadata."""
        for field_name, field_info in self.__dataclass_fields__.items():
            ownership = OwnershipType.OWNED  # Default ownership
            thread_safe = False
            
            # Check for Rust-specific metadata in field.metadata
            if hasattr(field_info, "metadata"):
                metadata = field_info.metadata
                if "ownership" in metadata:
                    ownership = metadata["ownership"]
                if "thread_safe" in metadata:
                    thread_safe = metadata["thread_safe"]
            
            # Get field type hint
            type_hint = field_info.type
            
            # Try to get docstring from the class's annotations or __doc__
            doc = ""
            class_doc = inspect.getdoc(self.__class__)
            if class_doc:
                # Simple heuristic to extract field docs: look for "field_name: docstring"
                for line in class_doc.split("\n"):
                    if line.strip().startswith(f"{field_name}:"):
                        doc = line.split(":", 1)[1].strip()
                        break
            
            # Store field metadata
            self.__class__.__rust_fields__[field_name] = RustField(
                name=field_name,
                type_hint=type_hint,
                ownership=ownership,
                doc=doc,
                thread_safe=thread_safe,
            )
    
    def to_dict(self) -> Dict:
        """Convert to dictionary."""
        return asdict(self)
    
    def to_json(self) -> str:
        """Convert to JSON string."""
        return json.dumps(self.to_dict())
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'RustCompatible':
        """Create an instance from a dictionary."""
        return cls(**data)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'RustCompatible':
        """Create an instance from a JSON string."""
        data = json.loads(json_str)
        return cls.from_dict(data)
    
    @classmethod
    def generate_rust_struct(cls) -> str:
        """Generate Rust struct definition for this dataclass."""
        if not is_dataclass(cls):
            raise TypeError(f"{cls.__name__} is not a dataclass")
        
        lines = [f"// Auto-generated Rust struct for {cls.__name__}"]
        
        # Add derives
        lines.append("#[derive(Debug, Clone, PartialEq)]")
        
        # Add struct definition
        lines.append(f"pub struct {cls.__name__} {{")
        
        # Add fields
        for field_name, field_info in cls.__rust_fields__.items():
            # Format type based on ownership
            rust_type = _python_type_to_rust_type(field_info.type_hint)
            if field_info.ownership == OwnershipType.BORROWED:
                rust_type = f"&{rust_type}"
            elif field_info.ownership == OwnershipType.MUTABLE_BORROWED:
                rust_type = f"&mut {rust_type}"
            
            # Add field documentation if available
            if field_info.doc:
                lines.append(f"    /// {field_info.doc}")
            
            # Add thread-safety annotation if applicable
            if field_info.thread_safe:
                lines.append(f"    // This field is thread-safe")
            
            # Add field declaration
            lines.append(f"    pub {field_name}: {rust_type},")
        
        # Close struct definition
        lines.append("}")
        
        # Add implementation block
        lines.append("")
        lines.append(f"impl {cls.__name__} {{")
        lines.append(f"    /// Create a new {cls.__name__}")
        lines.append("    pub fn new(")
        
        # Add constructor parameters
        params = []
        for field_name, field_info in cls.__rust_fields__.items():
            rust_type = _python_type_to_rust_type(field_info.type_hint)
            if field_info.ownership == OwnershipType.BORROWED:
                rust_type = f"&{rust_type}"
            elif field_info.ownership == OwnershipType.MUTABLE_BORROWED:
                rust_type = f"&mut {rust_type}"
            params.append(f"        {field_name}: {rust_type}")
        
        lines.append(",\n".join(params))
        lines.append("    ) -> Self {")
        lines.append(f"        {cls.__name__} {{")
        
        # Add field initializers
        for field_name in cls.__rust_fields__.keys():
            lines.append(f"            {field_name},")
        
        lines.append("        }")
        lines.append("    }")
        lines.append("}")
        
        return "\n".join(lines)


def rust_field(*, ownership: OwnershipType = OwnershipType.OWNED, thread_safe: bool = False):
    """Create a field with Rust-specific metadata."""
    return field(metadata={"ownership": ownership, "thread_safe": thread_safe})


def _python_type_to_rust_type(py_type: Type) -> str:
    """
    Convert a Python type to an equivalent Rust type.
    
    Args:
        py_type: Python type
        
    Returns:
        Equivalent Rust type as a string
    """
    # Basic types
    if py_type is int:
        return "i32"
    elif py_type is float:
        return "f64"
    elif py_type is bool:
        return "bool"
    elif py_type is str:
        return "String"
    elif py_type is bytes:
        return "Vec<u8>"
    
    # Check for Optional types (e.g., Optional[int])
    origin = getattr(py_type, "__origin__", None)
    if origin is Union:
        args = getattr(py_type, "__args__", ())
        if len(args) == 2 and args[1] is type(None):
            inner_type = _python_type_to_rust_type(args[0])
            return f"Option<{inner_type}>"
    
    # Check for List types
    if origin is list:
        args = getattr(py_type, "__args__", ())
        if len(args) == 1:
            inner_type = _python_type_to_rust_type(args[0])
            return f"Vec<{inner_type}>"
    
    # Check for Dict types
    if origin is dict:
        args = getattr(py_type, "__args__", ())
        if len(args) == 2:
            key_type = _python_type_to_rust_type(args[0])
            value_type = _python_type_to_rust_type(args[1])
            if key_type == "String":
                return f"HashMap<{key_type}, {value_type}>"
            else:
                return f"BTreeMap<{key_type}, {value_type}>"
    
    # Check for Tuple types
    if origin is tuple:
        args = getattr(py_type, "__args__", ())
        inner_types = [_python_type_to_rust_type(arg) for arg in args]
        return f"({', '.join(inner_types)})"
    
    # For other types, use the type name
    return py_type.__name__


# Example usage of RustCompatible dataclass
@dataclass
class ExampleStruct(RustCompatible):
    """
    Example struct demonstrating Rust-compatible dataclass.
    
    name: Name of the example
    value: Numeric value
    tags: List of string tags
    """
    
    name: str
    value: int = 0
    tags: List[str] = field(default_factory=list)
    # Demonstrate a field with Rust-specific metadata
    reference_count: int = rust_field(ownership=OwnershipType.MUTABLE_BORROWED, thread_safe=True)


# Example of using custom field with thread-safety annotation
@dataclass
class ThreadSafeCounter(RustCompatible):
    """A thread-safe counter."""
    
    value: int = rust_field(thread_safe=True)
    
    def increment(self) -> int:
        """Increment the counter."""
        self.value += 1
        return self.value 