"""
Text Inputs

Text/language inputs for FEAGI (language learning, NLP, etc).
"""

from typing import Literal, Optional
from feagi.pns.inputs.base import BaseInput

# Type hints
TextTokenizer = Literal["char", "word", "byte"]


class TextStream(BaseInput):
    """
    Text stream input for language learning.
    
    Converts text to neuron activations for FEAGI to process.
    Supports character-level, word-level, or byte-level tokenization.
    
    Args:
        tokenizer: "char", "word", or "byte"
        max_length: Maximum text length
        encoding: Encoding mode
    
    Example:
        from feagi.pns.inputs import TextStream
        from feagi.pns import brain_input
        
        text_in = TextStream.register(
            tokenizer="char",
            max_length=100
        )
        
        brain_input.configure(feagi_host="localhost")
        brain_input.connect()
        
        while True:
            user_input = input("You: ")
            text_in.set_text(user_input)
            brain_input.send()
    """
    
    def __init__(
        self,
        tokenizer: TextTokenizer = "char",
        max_length: int = 100,
        encoding: Literal["absolute", "incremental"] = "absolute"
    ):
        super().__init__()
        self.tokenizer = tokenizer
        self.max_length = max_length
        self.encoding = encoding
        
        # Current text
        self._current_text: Optional[str] = None
        
        # Vocabulary (for tokenization)
        self._vocab = None
        self._init_vocab()
    
    def _init_vocab(self):
        """Initialize vocabulary based on tokenizer"""
        if self.tokenizer == "char":
            # ASCII printable characters
            self._vocab = {chr(i): i - 32 for i in range(32, 127)}
        elif self.tokenizer == "byte":
            # Raw bytes (0-255)
            self._vocab = {bytes([i]): i for i in range(256)}
        elif self.tokenizer == "word":
            # Word-level requires dynamic vocabulary
            # For now, use a simple space-split
            self._vocab = {}
    
    @classmethod
    def register(
        cls,
        tokenizer: TextTokenizer = "char",
        max_length: int = 100,
        encoding: Literal["absolute", "incremental"] = "absolute"
    ) -> 'TextStream':
        """
        Register a new text stream input.
        
        Args:
            tokenizer: "char" (character-level), "word", or "byte"
            max_length: Maximum text length to process
            encoding: Encoding mode
        
        Returns:
            TextStream instance
        """
        from feagi.pns import brain_input
        
        stream = cls(tokenizer, max_length, encoding)
        brain_input.register_input(stream)
        return stream
    
    def set_text(self, text: str):
        """
        Set current text input.
        
        Args:
            text: Text string to send to FEAGI
        """
        if len(text) > self.max_length:
            text = text[:self.max_length]
        self._current_text = text
    
    def _tokenize(self, text: str) -> list:
        """Tokenize text based on tokenizer type"""
        if self.tokenizer == "char":
            return [self._vocab.get(c, 0) for c in text]
        elif self.tokenizer == "byte":
            return [b for b in text.encode('utf-8')]
        elif self.tokenizer == "word":
            words = text.split()
            # TODO: Implement proper word tokenization
            return [hash(w) % 1000 for w in words]
        return []
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # TODO: Implement text input registration in Rust
        # For now, this is a placeholder
        pass
    
    def _write_to_cache(self, cache):
        """Write current text to Rust IOCache"""
        if self._current_text is None:
            return
        
        # TODO: Implement text encoding and writing to cache
        # This will require Rust support for text inputs
        # For now, tokenize and prepare data structure
        tokens = self._tokenize(self._current_text)
        # Write tokens to cache (when Rust support is added)
        pass

