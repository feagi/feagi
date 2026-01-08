"""
Text Inputs

Text/language inputs for FEAGI (language learning, NLP, etc).
"""

from collections import deque
from typing import Deque, Literal, Optional
from feagi.pns.inputs.base import BaseInput

# Type hints
TextTokenizer = Literal["gpt2"]


class TextStream(BaseInput):
    """
    Text stream input for language learning.
    
    Streams text into FEAGI as a GPT-2 token stream (one token per FEAGI tick).
    
    Args:
        tokenizer: "gpt2"
        tokenizer_json_path: Path to the pinned GPT-2 `tokenizer.json`
        max_length: Maximum text length
        depth: Bitplane depth for iten (default 16; must match cortical area
            topology)
    
    Example:
        from feagi.pns.inputs import TextStream
        from feagi.pns import brain_input
        
        text_in = TextStream.register(
            tokenizer="gpt2",
            tokenizer_json_path="/path/to/tokenizer.json",
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
        
        # Current text (latest submitted)
        self._current_text: Optional[str] = None

        # Tokenizer + pending token queue
        # (one token emitted per brain_input.send() tick)
        self._tokenizer = None
        self._pending_token_ids: Deque[int] = deque()

    def _ensure_tokenizer(self) -> None:
        """Lazy-load GPT-2 tokenizer via feagi_rust_py_libs."""
        if self._tokenizer is not None:
            return
        if self.tokenizer != "gpt2":
            raise ValueError(f"Unsupported tokenizer: {self.tokenizer}")
        if not self.tokenizer_json_path:
            raise ValueError(
                "tokenizer_json_path must be provided for tokenizer='gpt2'."
            )
        try:
            import feagi_rust_py_libs as frpl
        except ImportError as e:
            raise ImportError(
                "feagi_rust_py_libs is required for tokenizer='gpt2'."
            ) from e
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
        Register a new text stream input.
        
        Args:
            tokenizer: "gpt2"
            tokenizer_json_path: Path to the pinned GPT-2 `tokenizer.json`
            max_length: Maximum text length to process
            depth: Bitplane depth for iten (default 16)
        
        Returns:
            TextStream instance
        """
        from feagi.pns import brain_input
        
        stream = cls(
            tokenizer=tokenizer,
            tokenizer_json_path=tokenizer_json_path,
            max_length=max_length,
            depth=depth,
        )
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
        self._enqueue_text(text)
    
    def _enqueue_text(self, text: str) -> None:
        """Encode text into token IDs and queue them for streaming.

        One token is emitted per `brain_input.send()` tick.
        """
        self._ensure_tokenizer()
        token_ids = self._tokenizer.encode(text)
        for token_id in token_ids:
            self._pending_token_ids.append(int(token_id))
    
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
        cache.sensor_text_english_input_register(
            group=group_id,
            number_channels=1,
            frame_change_handling=frame,
            misc_data_dimensions=dims,
        )

    def get_registration_capabilities(self) -> dict:
        """Return structured capabilities for agent registration (iten).

        This is intended to be merged into the agent registration payload so
        missing cortical areas can be auto-created deterministically.
        """
        if self.group_id is None:
            raise RuntimeError(
                "TextStream input is not registered (group_id is None)."
            )

        import feagi_rust_py_libs as frpl

        genomic = frpl.data_structures.genomic
        FrameChangeHandling = genomic.cortical_area.FrameChangeHandling
        SensoryCorticalUnit = genomic.SensoryCorticalUnit

        frame = FrameChangeHandling.Absolute
        ids = SensoryCorticalUnit.text_english_input_cortical_ids(
            frame_change_handling=frame,
            group=int(self.group_id),
        )
        if not ids:
            raise RuntimeError(
                "Failed to generate iten CorticalID list (empty)."
            )

        cortical_id_b64 = ids[0].as_base_64()
        return {
            "sensory": {
                "rate_hz": 30.0,
                "shm_path": None,
                "cortical_mappings": {cortical_id_b64: 0},
            }
        }
    
    def _write_to_cache(self, cache):
        """Write current text to Rust IOCache"""
        import feagi_rust_py_libs as frpl

        if not self._pending_token_ids:
            # Gap semantics: emit no token (send an all-zero MiscData buffer).
            blank = frpl.connector_core.data_types.MiscData(
                1,
                1,
                int(self.depth),
            )
            cache.sensor_text_english_input_write(
                group=self.group_id,
                channel_index=0,
                data=blank,
            )
            return

        token_id = int(self._pending_token_ids.popleft())
        data_types = frpl.connector_core.data_types
        misc = data_types.TextTokenCodec.encode_to_misc_data(
            token_id,
            int(self.depth),
        )
        cache.sensor_text_english_input_write(
            group=self.group_id,
            channel_index=0,
            data=misc,
        )

