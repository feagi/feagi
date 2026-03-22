# FEAGI Brain Visualizer Runtime

This package provides platform-specific Brain Visualizer binaries for the FEAGI SDK.
It is intended to be installed as an optional dependency:

```bash
pip install "feagi[bv]"
```

> **Note:** Use quotes for zsh users (macOS default shell).

The FEAGI CLI launches Brain Visualizer with:

```bash
feagi bv start --config feagi_configuration.toml
```

Python usage:

```python
from feagi_bv import BrainVisualizer

bv = BrainVisualizer()
bv.load_config("feagi_configuration.toml")
pid = bv.start()
```

## Directory Layout

```
feagi_bv/
└── bin/
    ├── linux/
    │   ├── BrainVisualizer
    │   └── (embedded PCK)
    ├── macos/
    │   └── BrainVisualizer.app/
    └── windows/
        ├── BrainVisualizer.exe
        └── BrainVisualizer.pck
        └── *.dll (GDExtension runtime)
```

## Notes

- Binaries are platform-specific and provided by the release pipeline.
- This package intentionally contains no Python runtime logic beyond data packaging.
