# FEAGI Process Management Scripts

This directory contains utility scripts for managing FEAGI processes.

## FEAGI Process Killer

Two versions of the FEAGI process killer are available:

### 1. Bash Version (`kill_feagi.sh`)
- **Platform**: Unix/Linux/macOS
- **Dependencies**: Standard Unix tools (ps, grep, kill)
- **Usage**: 
  ```bash
  ./scripts/kill_feagi.sh [OPTIONS]
  ```

### 2. Python Version (`kill_feagi.py`)
- **Platform**: Cross-platform (Windows, macOS, Linux)
- **Dependencies**: Python 3.6+ with `psutil` library
- **Usage**: 
  ```bash
  python3 scripts/kill_feagi.py [OPTIONS]
  ```

## Options

Both scripts support the following options:

- `--force`, `-f`: Kill processes without confirmation
- `--help`, `-h`: Show help message

## Examples

```bash
# Interactive mode (asks for confirmation)
./scripts/kill_feagi.sh

# Force kill without confirmation
./scripts/kill_feagi.sh --force

# Python version with force flag
python3 scripts/kill_feagi.py --force

# Show help
./scripts/kill_feagi.sh --help
```

## How It Works

1. **Process Discovery**: Scans system for FEAGI-related processes by looking for:
   - Process names containing "feagi" or "FEAGI"
   - Command lines containing FEAGI module paths
   - Python processes running FEAGI code

2. **Graceful Shutdown**: First attempts to terminate processes gracefully using SIGTERM
   - Waits 5 seconds for processes to shut down cleanly

3. **Force Kill**: If processes don't respond to SIGTERM, uses SIGKILL to force termination

4. **Verification**: Performs final check to ensure all FEAGI processes are terminated

## Safety Features

- **Confirmation**: Asks for user confirmation unless `--force` flag is used
- **Process Details**: Shows detailed information about processes before killing
- **Graceful First**: Always tries graceful shutdown before force killing
- **Self-Exclusion**: Excludes the killer script itself from being terminated
- **Error Handling**: Handles access denied and already-terminated processes gracefully

## When to Use

- **Development**: Clean restart of FEAGI during development
- **Testing**: Reset system state between test runs
- **Debugging**: Force stop stuck or unresponsive FEAGI instances
- **Deployment**: Clean shutdown before system restart

## Notes

- Requires appropriate permissions to kill processes owned by other users
- On some systems, you may need to run with `sudo` for system-wide process termination
- The Python version provides more detailed process information and better cross-platform support 