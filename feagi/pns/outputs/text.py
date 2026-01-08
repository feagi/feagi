"""
Text Outputs

Text/language outputs for FEAGI (language generation, NLP, etc).
"""

from typing import Literal
from feagi.pns.outputs.base import BaseOutput

# Type hints
TextTokenizer = Literal["gpt2"]


class TextStream(BaseOutput):
    """
    Text stream output for language generation.
    
    Streams FEAGI output tokens (oten) and decodes them into text using a GPT-2
    tokenizer.
    
    Args:
        tokenizer: "gpt2"
        tokenizer_json_path: Path to the pinned GPT-2 `tokenizer.json`
        max_length: Maximum text length
        depth: Bitplane depth for oten (default 16; must match cortical area
            topology)
    
    Example:
        from feagi.pns.outputs import TextStream
        from feagi.pns import brain_output
        
        text_out = TextStream.register(
            tokenizer="gpt2",
            tokenizer_json_path="/path/to/tokenizer.json",
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
        tokenizer: TextTokenizer = "gpt2",
        tokenizer_json_path: str = "",
        max_length: int = 100,
        depth: int = 16,
    ):
        super().__init__()
        self.tokenizer = tokenizer
        self.tokenizer_json_path = tokenizer_json_path
        self.max_length = max_length
        self.depth = depth
        
        # Current generated text
        self._current_text: str = ""

        self._tokenizer = None

    def _ensure_tokenizer(self) -> None:
        if self._tokenizer is not None:
            return
        if self.tokenizer != "gpt2":
            raise ValueError(f"Unsupported tokenizer: {self.tokenizer}")
        if not self.tokenizer_json_path:
            raise ValueError(
                "tokenizer_json_path must be provided for tokenizer='gpt2'."
            )
        import feagi_rust_py_libs as frpl
        data_types = frpl.connector_core.data_types
        self._tokenizer = data_types.Gpt2Tokenizer.from_file(
            self.tokenizer_json_path
        )
    
    @classmethod
    def register(
        cls,
        tokenizer: TextTokenizer = "gpt2",
        tokenizer_json_path: str = "",
        max_length: int = 100,
        depth: int = 16,
    ) -> 'TextStream':
        """
        Register a new text stream output.
        
        Args:
            tokenizer: "gpt2"
            tokenizer_json_path: Path to the pinned GPT-2 `tokenizer.json`
            max_length: Maximum text length to generate
            depth: Bitplane depth for oten (default 16)
        
        Returns:
            TextStream instance
        """
        from feagi.pns import brain_output
        
        stream = cls(
            tokenizer=tokenizer,
            tokenizer_json_path=tokenizer_json_path,
            max_length=max_length,
            depth=depth,
        )
        brain_output.register_output(stream)
        return stream
    
    def get_text(self) -> str:
        """
        Get current generated text from FEAGI.
        
        Returns:
            Generated text string
        """
        return self._current_text
    
    def _register_with_cache(self, cache, group_id: int):
        """Register with Rust IOCache"""
        import feagi_rust_py_libs as frpl
        self._ensure_tokenizer()
        dims = frpl.connector_core.data_types.descriptors.MiscDataDimensions(
            1,
            1,
            int(self.depth),
        )
        frame = (
            frpl.data_structures.genomic.cortical_area.FrameChangeHandling.Absolute
        )
        cache.motor_text_english_output_register(
            group=group_id,
            number_channels=1,
            frame_change_handling=frame,
            misc_data_dimensions=dims,
        )

    def _get_cortical_id(self) -> str:
        """Return the base64 CorticalID for this output stream (oten).

        Used for agent registration / motor subscription.
        """
        if self.group_id is None:
            raise RuntimeError(
                "TextStream output is not registered (group_id is None)."
            )

        import feagi_rust_py_libs as frpl

        genomic = frpl.data_structures.genomic
        FrameChangeHandling = genomic.cortical_area.FrameChangeHandling
        MotorCorticalUnit = genomic.MotorCorticalUnit

        frame = FrameChangeHandling.Absolute
        ids = MotorCorticalUnit.text_english_output_cortical_ids(
            frame_change_handling=frame,
            group=int(self.group_id),
        )
        if not ids:
            raise RuntimeError(
                "Failed to generate oten CorticalID list (empty)."
            )
        return ids[0].as_base_64()
    
    def _read_from_cache(self, cache):
        """Read generated text from Rust IOCache"""
        import feagi_rust_py_libs as frpl

        self._ensure_tokenizer()
        misc = cache.motor_text_english_output_read_postprocessed_cache_value(
            group=self.group_id,
            channel_index=0,
        )
        data_types = frpl.connector_core.data_types
        token_id = data_types.TextTokenCodec.decode_from_misc_data(misc)
        if token_id is None:
            return

        chunk = self._tokenizer.decode([int(token_id)], True)
        if not chunk:
            return

        self._current_text = (self._current_text + chunk)[
            -self.max_length :
        ]

