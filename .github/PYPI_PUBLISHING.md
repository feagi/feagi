# PyPI Publishing Setup

This document explains how the FEAGI SDK is automatically published to Test PyPI.

## Automated Publishing

### Staging → Test PyPI

When code is merged to the `staging` branch, GitHub Actions automatically:
1. ✅ Builds the `feagi` package
2. ✅ Publishes to https://test.pypi.org
3. ✅ Makes it available for testing

**Workflow**: `.github/workflows/publish-feagi-staging.yml`

**Triggers on**:
- Push/merge to `staging` branch
- Changes to `feagi/`, `pyproject.toml`, or workflow file

## Required Secrets

### TEST_PYPI_API_TOKEN

1. **Generate Token**:
   - Go to https://test.pypi.org/manage/account/token/
   - Create new API token
   - Scope: Project `feagi` (or entire account)
   - Copy token (starts with `pypi-`)

2. **Add to GitHub**:
   - Go to repository Settings → Secrets and variables → Actions
   - Click "New repository secret"
   - Name: `TEST_PYPI_API_TOKEN`
   - Value: Paste your token
   - Click "Add secret"

## Manual Publishing (if needed)

### Build Package Locally
```bash
cd feagi-python-sdk

# Install build tools
pip install build twine

# Build package
python -m build

# Check build
twine check dist/*
```

### Publish to Test PyPI
```bash
# Using environment variables
export TWINE_USERNAME=__token__
export TWINE_PASSWORD=<your-test-pypi-token>

# Publish
python -m twine upload --repository testpypi dist/*
```

### Publish to Production PyPI
```bash
# Using environment variables
export TWINE_USERNAME=__token__
export TWINE_PASSWORD=<your-pypi-token>

# Publish
python -m twine upload dist/*
```

## Installation

### From Test PyPI
```bash
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ feagi
```

### From Production PyPI (when ready)
```bash
pip install feagi
```

## Version Management

Version is defined in `pyproject.toml`:

```toml
[project]
name = "feagi"
version = "3.0.0"  # ← Update this before merging to staging
```

**Important**: 
- ⚠️ Test PyPI does NOT allow re-uploading the same version
- ⚠️ Bump version before each merge to staging
- ✅ Use semantic versioning: MAJOR.MINOR.PATCH

### Version Bumping Strategy

For `staging` (test.pypi.org):
- Use pre-release versions: `3.0.0.dev1`, `3.0.0.dev2`, etc.
- Or date-based: `3.0.0.20250116`

For `main` (pypi.org):
- Use release versions: `3.0.0`, `3.0.1`, `3.1.0`, etc.

## Workflow Details

### Trigger Conditions
```yaml
on:
  push:
    branches:
      - staging
    paths:
      - 'feagi/**'
      - 'pyproject.toml'
      - 'setup.py'
      - '.github/workflows/publish-feagi-staging.yml'
```

### Build Process
1. Checkout code
2. Setup Python 3.11
3. Install `build` and `twine`
4. Extract version from `pyproject.toml`
5. Build source distribution and wheel
6. Check package integrity
7. Upload to test.pypi.org

### Success Output
The workflow provides:
- ✅ Package URL on test.pypi.org
- ✅ Installation command
- ✅ Version number
- ✅ Build artifacts

## Troubleshooting

### "File already exists"
- Version already published to test.pypi.org
- Bump version in `pyproject.toml`

### "Invalid credentials"
- Check `TEST_PYPI_API_TOKEN` secret
- Regenerate token if needed

### "Package not found after publishing"
- Test PyPI can take a few minutes to sync
- Check https://test.pypi.org/project/feagi/

### Build Errors
- Check Python version compatibility (>=3.10)
- Verify all dependencies in `pyproject.toml`
- Run `python -m build` locally first

## Production Release (Future)

When ready for production PyPI:

1. Create workflow: `.github/workflows/publish-feagi-production.yml`
2. Trigger on: `push` to `main` or GitHub release
3. Add secret: `PYPI_API_TOKEN` (from https://pypi.org)
4. Same process, but publish to https://pypi.org

## Links

- **Test PyPI**: https://test.pypi.org/project/feagi/
- **Production PyPI** (when ready): https://pypi.org/project/feagi/
- **Packaging Guide**: https://packaging.python.org/
- **Twine Docs**: https://twine.readthedocs.io/






