# FEAGI SDK Release Guide

This guide is for maintainers who publish new versions of the FEAGI SDK to PyPI.

---

## Release Process Overview

FEAGI uses **Git releases** to trigger automated publishing to PyPI. The version number is extracted from the Git tag, ensuring the tag is the single source of truth.

### Quick Summary

1. **Merge changes** to appropriate branch (staging or main)
2. **Create GitHub release** with version tag
3. **GitHub Action automatically** publishes to PyPI

---

## Release Types

### Pre-releases (Beta, RC)

**When:** Testing new features, breaking changes, or major updates before stable release

**Branch:** `staging`

**Version format:** `2.1.0b1`, `2.1.0b2`, `2.1.0rc1`

**Git tag format:** `v2.1.0b1`, `v2.1.0b2`, `v2.1.0rc1`

**PyPI behavior:**
- NOT installed by default with `pip install feagi`
- Requires explicit version: `pip install feagi==2.1.0b1`
- Or pre-release flag: `pip install --pre feagi`

### Stable Releases

**When:** Production-ready releases

**Branch:** `main`

**Version format:** `2.1.0`, `2.2.0`, `3.0.0`

**Git tag format:** `v2.1.0`, `v2.2.0`, `v3.0.0`

**PyPI behavior:**
- Default install: `pip install feagi` gets this version
- Becomes the recommended version for all users

---

## Semantic Versioning

FEAGI follows [Semantic Versioning](https://semver.org/):

```
MAJOR.MINOR.PATCH
  |     |     |
  |     |     +-- Bug fixes, no new features (2.1.1)
  |     +-------- New features, backward compatible (2.2.0)
  +-------------- Breaking changes (3.0.0)
```

### Pre-release Identifiers

- `a` - Alpha (early testing, unstable)
- `b` - Beta (feature complete, testing phase)
- `rc` - Release Candidate (final testing before stable)

**Examples:**
- `2.1.0a1` - Alpha 1
- `2.1.0b1` - Beta 1
- `2.1.0b2` - Beta 2
- `2.1.0rc1` - Release Candidate 1
- `2.1.0` - Stable release

---

## Step-by-Step: Publishing a Pre-release

### 1. Prepare the Release

Ensure all changes are merged to `staging` branch:

```bash
git checkout staging
git pull origin staging
```

### 2. Create GitHub Pre-release

Go to: https://github.com/feagi/feagi-python-sdk/releases/new

**Fill in the form:**

- **Choose a tag:** `v2.1.0b1` (create new tag)
- **Target:** `staging`
- **Release title:** `FEAGI SDK v2.1.0 Beta 1`
- **Description:** Write release notes (see template below)
- ✅ **Check "Set as a pre-release"**
- ✅ **Check "Set as the latest release"** (optional, for pre-releases)
- Click **"Publish release"**

### 3. Automated Publishing

GitHub Action automatically:
1. Extracts version from tag: `v2.1.0b1` → `2.1.0b1`
2. Updates `pyproject.toml` and `feagi/__init__.py`
3. Builds package with bundled binaries
4. Publishes to PyPI

**Monitor progress:**
- Go to: Actions → Publish FEAGI SDK to PyPI
- Watch the workflow run
- Check for any errors

### 4. Verify Publication

Once published (usually 2-5 minutes):

```bash
# Check PyPI page
https://pypi.org/project/feagi/2.1.0b1/

# Test installation
pip install feagi==2.1.0b1

# Verify version
python -c "import feagi; print(feagi.__version__)"
```

---

## Step-by-Step: Publishing a Stable Release

### 1. Merge Staging to Main

```bash
# Ensure staging is ready
git checkout staging
git pull origin staging

# Merge to main
git checkout main
git pull origin main
git merge staging

# Push to main
git push origin main
```

### 2. Create GitHub Release

Go to: https://github.com/feagi/feagi-python-sdk/releases/new

**Fill in the form:**

- **Choose a tag:** `v2.1.0` (create new tag)
- **Target:** `main`
- **Release title:** `FEAGI SDK v2.1.0`
- **Description:** Write comprehensive release notes (see template below)
- ❌ **Uncheck "Set as a pre-release"**
- ✅ **Check "Set as the latest release"**
- Click **"Publish release"**

### 3. Automated Publishing

Same as pre-release - GitHub Action handles everything automatically.

### 4. Verify Publication

```bash
# Check PyPI page (becomes default version)
https://pypi.org/project/feagi/

# Test default installation
pip install feagi

# Verify version
python -c "import feagi; print(feagi.__version__)"
```

---

## Release Notes Template

### Pre-release Template

```markdown
## FEAGI SDK v2.1.0 Beta 1

🚀 **Pre-release for testing**

This is a beta release for testing new features before the stable 2.1.0 release.

### ✨ New Features
- Cross-platform paths management system
- Configuration auto-generation with `feagi init`
- Default config handling for zero-config start

### 🔧 Improvements
- Enhanced `feagi bv start` to work without config argument
- Smart path resolution for genomes and connectomes
- Platform-specific directory structure (Linux/macOS/Windows)

### 📚 Documentation
- Added comprehensive DEPLOY.md guide
- Updated README with simplified quick start

### 🐛 Bug Fixes
- (List any bug fixes)

### 📦 Installation

**Test this pre-release:**
```bash
pip install feagi==2.1.0b1
```

**Feedback:**
Please report issues at: https://github.com/feagi/feagi-python-sdk/issues

**Next Steps:**
If no major issues are found, stable 2.1.0 will be released.
```

### Stable Release Template

```markdown
## FEAGI SDK v2.1.0

🎉 **Stable Release**

### ✨ New Features
- **Cross-platform paths management** - Automatic directory structure for Linux, macOS, and Windows
- **Zero-config quick start** - `pip install feagi[bv] && feagi bv start` just works
- **Smart path resolution** - Genomes and connectomes auto-resolve from standard directories
- **Configuration auto-generation** - `feagi init` creates complete environment

### 🔧 Improvements
- Enhanced CLI with `feagi init` command
- Default configuration handling throughout SDK
- Platform-aware directory structure
  - Hidden: `~/.feagi/config/`, `~/.feagi/logs/`
  - Visible: `~/Documents/FEAGI/Genomes/` (macOS/Windows) or `~/FEAGI/genomes/` (Linux)

### 📚 Documentation
- **NEW:** [DEPLOY.md](./DEPLOY.md) - Comprehensive deployment guide
- Updated README with simplified quick start
- Platform-specific installation notes

### 🐛 Bug Fixes
- (List any bug fixes from beta testing)

### 📦 Installation

```bash
pip install feagi[bv]
feagi bv start
```

### 🔗 Links
- [Documentation](https://docs.feagi.org)
- [Deployment Guide](./DEPLOY.md)
- [Report Issues](https://github.com/feagi/feagi-python-sdk/issues)

### 🙏 Contributors
Thanks to all contributors who made this release possible!
```

---

## Version Numbering Strategy

### When to bump MAJOR (X.0.0)

Breaking changes that require user action:
- API changes (removed/renamed methods)
- Configuration format changes
- Python version requirement changes
- Breaking behavior changes

**Example:** `2.1.0` → `3.0.0`

### When to bump MINOR (x.Y.0)

New features, backward compatible:
- New CLI commands
- New API methods
- New optional features
- Significant improvements

**Example:** `2.1.0` → `2.2.0`

### When to bump PATCH (x.y.Z)

Bug fixes, no new features:
- Bug fixes
- Documentation fixes
- Performance improvements (no API changes)
- Dependency updates

**Example:** `2.1.0` → `2.1.1`

---

## Beta Release Cycle

Typical flow for major/minor releases:

```
2.0.6 (stable)
  ↓
2.1.0b1 (first beta) - major new features
  ↓
2.1.0b2 (second beta) - bug fixes from b1
  ↓
2.1.0rc1 (release candidate) - final testing
  ↓
2.1.0 (stable) - production ready
```

**Guidelines:**
- Use beta for new features (at least 1-2 weeks of testing)
- Use RC when confident, need final validation
- Skip directly to stable for minor bug fixes

---

## Manual Publishing (Emergency)

If automated workflow fails, manually trigger:

1. Go to: Actions → Publish FEAGI SDK to PyPI
2. Click "Run workflow"
3. Enter version (e.g., `2.1.0b1`)
4. Check "Dry run" to test first
5. Run again without dry run to publish

---

## Troubleshooting

### Version Already Exists on PyPI

**Error:** `File already exists`

**Solution:**
- Cannot overwrite existing PyPI versions
- Bump to next version (e.g., `2.1.0b1` → `2.1.0b2`)
- Create new tag and release

### Workflow Fails - Missing Binaries

**Error:** `Missing binaries: linux-x86_64/feagi`

**Solution:**
- Binaries must be committed before release
- Check `feagi/bin/` directory has all platform binaries
- See main README for binary building instructions

### Wrong Version Published

**Problem:** Published wrong version to PyPI

**Solution:**
- Cannot delete/modify PyPI releases
- Immediately publish correct version with incremented number
- Example: If `2.1.0` was wrong, publish `2.1.1` with fix
- Mark wrong version as yanked on PyPI (doesn't delete, but warns users)

### PyPI Token Issues

**Error:** `Invalid credentials`

**Solution:**
- Check `PYPI_API_TOKEN` secret in repository settings
- Token must have upload permissions for `feagi` package
- Generate new token at: https://pypi.org/manage/account/token/

---

## Checklist

### Before Creating Release

- [ ] All PRs merged to target branch (staging or main)
- [ ] CI tests passing
- [ ] CHANGELOG updated (if maintained)
- [ ] Version bump planned (follows semantic versioning)
- [ ] Binaries are up to date and committed
- [ ] Documentation updated

### Creating Release

- [ ] Correct tag format (`v2.1.0b1` or `v2.1.0`)
- [ ] Target branch correct (staging for pre-release, main for stable)
- [ ] Release notes complete and clear
- [ ] Pre-release checkbox set correctly
- [ ] "Latest release" checked appropriately

### After Release

- [ ] GitHub Action completed successfully
- [ ] Package visible on PyPI
- [ ] Test installation works
- [ ] Version number correct
- [ ] Announce release (Discord, Twitter, etc.)

---

## Support

Questions about releasing?
- **Discord**: [Join our community](https://discord.gg/PTVC8fyGN8)
- **Issues**: [GitHub Issues](https://github.com/feagi/feagi-python-sdk/issues)

---

**Copyright 2016-2025 Neuraville Inc. All Rights Reserved.**
