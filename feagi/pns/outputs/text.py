"""
Text Outputs

Text/language outputs for FEAGI (language generation, NLP, etc).
"""

from typing import Literal, Optional
from feagi.pns.outputs.base import BaseOutput

# Type hints
TextTokenizer = Literal["char", "word", "byte"]


class TextStream(BaseOutput):
    """
    Text stream output for language generation.
    
    Converts FEAGI neuron activations to text output.
    Supports character-level, word-level, or byte-level detokenization.
    
    Args:
        tokenizer: "char", "word", or "byte"
        max_length: Maximum text length
    
    Example:
        from feagi.pns.outputs import TextStream
        from feagi.pns import brain_output
        
        text_out = TextStream.register(
            tokenizer="char",
            max_length=100
        )
        
        brain_output.configure(feagi_host="localhost")
        brain_output.connect()
        
        while True:
            brain_output.receive()
            generated_text = text_out.get_text()
            print(f"FEAGI: {generated_text}")
    """
    
    def __init__(
        self,
        tokenizer: TextTokenizer = "char",
        max_length: int = 100
    ):
        super().__init__()
        self.tokenizer = tokenizer
        self.max_length = max_length
        
        # Current generated text
        self._current_text: str = ""
        
        # Vocabulary (for detokenization)
        self._vocab = None
        self._init_vocab()
    
    def _init_vocab(self):
        """Initialize vocabulary based on tokenizer"""
        if self.tokenizer == "char":
            # ASCII printable characters (reverse mapping)
            self._vocab = {i - 32: chr(i) for i in range(32, 127)}
        elif self.tokenizer == "byte":
            # Raw bytes (0-255)
            self._vocab = {i: bytes([i]) for i in range(256)}
        elif self.tokenizer == "word":
            # Word-level requires dynamic vocabulary
            self._vocab = {}
    
    @classmethod
    def register(
        cls,
        tokenizer: TextTokenizer = "char",
        max_length: int = 100
    ) -> 'TextStream':
        """
        Register a new text stream output.
        
        Args:
            tokenizer: "char" (character-level), "word", or "byte"
            max_length: Maximum text length to generate
        
        Returns:
            TextStream instance
        """
        from feagi.pns import brain_output
        
        stream = cls(tokenizer, max_length)
        brain_output.register_output(stream)
        return stream
    
    def get_text(self) -> str:
        """
        Get current generated text from FEAGI.
        
        Returns:
            Generated text string
        """
        return self._current_text
    
    def _detokenize(self, tokens: list) -> str:
        """Detokenize token list based on tokenizer type"""
        if self.tokenizer == "char":
            return ''.join(self._vocab.get(t, '?') for t in tokens)
        elif self.tokenizer == "byte":
            byte_data = b''.join(self._vocab.get(t, b'?') for t in tokens)
            try:
                return byte_data.decode('utf-8', errors='replace')
            except Exception:
                return ""
        elif self.tokenizer == "word":
            # TODO: Implement proper word detokenization
            return ' '.join(str(t) for t in tokens)
        return ""
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        # TODO: Implement text output registration in Rust
        # For now, this is a placeholder
        pass
    
    def _read_from_cache(self, cache):
        """Read generated text from Rust IOCache"""
        # TODO: Implement text decoding from cache
        # This will require Rust support for text outputs
        # For now, just keep empty text
        pass

