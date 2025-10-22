# Publishing FEAGI to PyPI

This guide is for maintainers who need to publish a new version of FEAGI to PyPI.

## Prerequisites

1. **GitHub Permissions**: Write access to the repository
2. **PyPI Account**: Member of the FEAGI PyPI project (https://pypi.org/project/feagi/)
3. **Trusted Publishing**: Configure GitHub Actions as a trusted publisher on PyPI (one-time setup)

## One-Time Setup: PyPI Trusted Publishing

This allows GitHub Actions to publish directly to PyPI without API tokens.

1. Go to https://pypi.org/manage/account/publishing/
2. Add a new publisher:
   - **PyPI Project Name**: `feagi`
   - **Owner**: `neuraville` (or your org)
   - **Repository**: `feagi`
   - **Workflow**: `build-wheels.yml`
   - **Environment**: `pypi`

3. For TestPyPI (optional, for testing):
   - Go to https://test.pypi.org/manage/account/publishing/
   - Add the same publisher configuration

## Publishing a New Release

### 1. Update Version Number

Edit `feagi_core/pyproject.toml`:

```toml
[project]
name = "feagi"
version = "0.2.0"  # ← Update this
```

### 2. Update CHANGELOG (if exists)

Document changes in the new version:

```bash
cd feagi_core
vim CHANGELOG.md  # Add release notes
```

### 3. Commit and Push

```bash
git add pyproject.toml CHANGELOG.md
git commit -m "Bump version to 0.2.0"
git push origin main
```

### 4. Create a GitHub Release

**Option A: Via GitHub Web UI**

1. Go to https://github.com/neuraville/feagi/releases/new
2. Click "Choose a tag"
3. Type: `v0.2.0` (must start with 'v')
4. Click "Create new tag: v0.2.0 on publish"
5. Release title: `FEAGI v0.2.0`
6. Description: Copy from CHANGELOG or write release notes
7. Click "Publish release"

**Option B: Via Command Line**

```bash
git tag -a v0.2.0 -m "Release version 0.2.0"
git push origin v0.2.0

# Then create the release on GitHub UI or use GitHub CLI:
gh release create v0.2.0 --title "FEAGI v0.2.0" --notes "Release notes here"
```

### 5. Monitor the Build

The `build-wheels.yml` workflow will automatically trigger:

1. Go to https://github.com/neuraville/feagi/actions
2. Find the "Build and Publish Wheels" workflow
3. Monitor progress (takes ~30-45 minutes)

**What's happening:**
- Building wheels on Ubuntu, macOS (Intel + ARM), Windows
- For Python 3.8, 3.9, 3.10, 3.11, 3.12
- Testing each wheel
- Publishing to PyPI

### 6. Verify Publication

After the workflow completes:

```bash
# Wait a few minutes for PyPI to update
sleep 120

# Check PyPI page
open https://pypi.org/project/feagi/

# Test installation
python3 -m venv test-venv
source test-venv/bin/activate
pip install feagi==0.2.0
python -c "import feagi; print(feagi.__version__)"
deactivate
rm -rf test-venv
```

## Testing Before Release (TestPyPI)

To test the build process without publishing to production PyPI:

### 1. Trigger Manual Workflow

1. Go to https://github.com/neuraville/feagi/actions/workflows/build-wheels.yml
2. Click "Run workflow"
3. Select branch: `main`
4. Check "Publish to TestPyPI" ✅
5. Click "Run workflow"

### 2. Test Installation from TestPyPI

```bash
pip install --index-url https://test.pypi.org/simple/ --extra-index-url https://pypi.org/simple/ feagi==0.2.0
```

Note: `--extra-index-url` is needed because dependencies (like `numpy`) aren't on TestPyPI.

## Troubleshooting

### Workflow Fails: "Rust toolchain not found"

Check the `CIBW_BEFORE_ALL_*` environment variables in `build-wheels.yml`. Ensure Rust installation is correct.

### Workflow Fails: "Publishing failed"

1. Verify trusted publishing is configured on PyPI
2. Check that the GitHub environment name matches: `pypi`
3. Ensure the workflow file path matches: `build-wheels.yml`

### Wheels Don't Install on User Machines

1. Check wheel platform tags: `unzip -l dist/*.whl | grep WHEEL`
2. Verify auditwheel/delocate repaired wheels correctly
3. Test on a clean VM or Docker container

### Version Already Exists on PyPI

PyPI doesn't allow overwriting versions. You must:
1. Increment the version number
2. Create a new release

## Wheel Details

Each release produces approximately 16-20 wheel files:

```
feagi-0.2.0-cp38-cp38-manylinux2014_x86_64.whl     # Linux, Python 3.8
feagi-0.2.0-cp39-cp39-manylinux2014_x86_64.whl     # Linux, Python 3.9
feagi-0.2.0-cp310-cp310-manylinux2014_x86_64.whl   # Linux, Python 3.10
feagi-0.2.0-cp311-cp311-manylinux2014_x86_64.whl   # Linux, Python 3.11
feagi-0.2.0-cp312-cp312-manylinux2014_x86_64.whl   # Linux, Python 3.12

feagi-0.2.0-cp38-cp38-macosx_10_9_x86_64.whl       # macOS Intel, Python 3.8
feagi-0.2.0-cp39-cp39-macosx_10_9_x86_64.whl       # macOS Intel, Python 3.9
... (same for cp310, cp311, cp312)

feagi-0.2.0-cp38-cp38-macosx_11_0_arm64.whl        # macOS ARM, Python 3.8
feagi-0.2.0-cp39-cp39-macosx_11_0_arm64.whl        # macOS ARM, Python 3.9
... (same for cp310, cp311, cp312)

feagi-0.2.0-cp38-cp38-win_amd64.whl                # Windows, Python 3.8
feagi-0.2.0-cp39-cp39-win_amd64.whl                # Windows, Python 3.9
... (same for cp310, cp311, cp312)

feagi-0.2.0.tar.gz                                  # Source distribution
```

Total artifact size: ~100-150 MB (all wheels combined)

## Build Times

Expect these approximate build times:

- **Ubuntu**: 10-15 minutes
- **macOS Intel**: 12-18 minutes
- **macOS ARM**: 12-18 minutes
- **Windows**: 15-20 minutes
- **Total**: 30-45 minutes for all platforms

## Support

For issues with the publishing process:
- Check GitHub Actions logs
- Review cibuildwheel documentation: https://cibuildwheel.readthedocs.io/
- Contact the DevOps team

