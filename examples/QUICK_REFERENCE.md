# Quick Reference Card

## Activate Environment
```bash
cd /Users/nadji/code/FEAGI-2.0/feagi-connector/examples
source activate.sh
```

## Run Examples

| File | When to Use | Output |
|------|-------------|--------|
| `example_2_2_standalone.py` | No FEAGI needed | ✅ Full test output |
| `example_2_2_debug.py` | Testing with FEAGI | ✅ Progress + errors |
| `example_2_2.py` | Production | ❌ No output |

## Common Commands

```bash
# Test without FEAGI
python example_2_2_standalone.py

# Test with FEAGI (with debug output)
python example_2_2_debug.py

# Check if FEAGI is running
lsof -i :30000

# Deactivate environment
deactivate
```

## Troubleshooting One-Liners

```bash
# Script hangs?
→ Use example_2_2_debug.py instead

# Module not found?
→ source activate.sh

# Connection timeout?
→ Start FEAGI first

# Need to rebuild Rust libs?
→ cd ../../feagi-rust-py-libs && maturin develop --release
```

## File Structure
```
examples/
├── example_venv/          # Virtual environment
├── example_2_2.py         # Production (silent)
├── example_2_2_debug.py   # Debug (verbose) ⭐
├── example_2_2_standalone.py  # No FEAGI needed ⭐
├── activate.sh            # Quick activation
├── requirements.txt       # Dependencies
├── SETUP.md              # Full setup guide
├── TROUBLESHOOTING.md    # Problem solving
└── README_EXAMPLES.md    # Detailed docs
```

## Quick Test Sequence

```bash
# 1. Activate
source activate.sh

# 2. Test locally (no FEAGI)
python example_2_2_standalone.py
# Should see: "✓ All local functionality working correctly!"

# 3. Test with FEAGI (in another terminal, start FEAGI first)
python example_2_2_debug.py
# Should see: "✓ Connected successfully!"
```

---
**Pro Tip:** Always use `example_2_2_debug.py` when testing. Only use `example_2_2.py` in production when you know FEAGI is running.

