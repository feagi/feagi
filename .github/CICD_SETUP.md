# CI/CD Setup for FEAGI SDK

This document explains how to set up automated building and publishing of the FEAGI SDK with bundled binaries.

## Overview

When code is merged to `staging`, GitHub Actions automatically:
1. ✅ Builds FEAGI binary for 5 platforms (Linux x86/ARM, macOS x86/ARM, Windows)
2. ✅ Bundles all binaries into Python package
3. ✅ Publishes to test.pypi.org

**Result**: Users can `pip install feagi` without needing Rust or source code!

## Required Secrets

You need to configure two GitHub secrets:

### 1. FEAGI_ACCESS_TOKEN

**Purpose**: Allow workflow to checkout private FEAGI repository

**Steps**:
1. Go to https://github.com/settings/tokens
2. Click "Generate new token" → "Fine-grained token"
3. Configure:
   - **Token name**: `FEAGI CI/CD Access`
   - **Expiration**: 1 year (or custom)
   - **Repository access**: Only select `FEAGI-2.0` (private repo)
   - **Permissions**:
     - Repository permissions → Contents: **Read-only** ✅
4. Click "Generate token"
5. Copy the token (starts with `github_pat_...`)

**Add to Repository**:
1. Go to feagi-python-sdk repository
2. Settings → Secrets and variables → Actions
3. Click "New repository secret"
4. Name: `FEAGI_ACCESS_TOKEN`
5. Value: Paste your token
6. Click "Add secret"

### 2. TEST_PYPI_API_TOKEN

**Purpose**: Publish package to test.pypi.org

**Steps**:
1. Go to https://test.pypi.org/manage/account/token/
2. Click "Add API token"
3. Configure:
   - **Token name**: `feagi-sdk-ci`
   - **Scope**: Project: `feagi` (recommended) or Entire account
4. Click "Add token"
5. Copy the token (starts with `pypi-...`)

**Add to Repository**:
1. Go to feagi-python-sdk repository
2. Settings → Secrets and variables → Actions
3. Click "New repository secret"
4. Name: `TEST_PYPI_API_TOKEN`
5. Value: Paste your token
6. Click "Add secret"

## Workflow Files

### Main Workflow: `publish-with-binaries.yml`

Builds for all platforms and publishes with bundled binaries.

**Triggers on**:
- Push/merge to `staging` branch
- Changes to:
  - `feagi/**` (Python SDK code)
  - `pyproject.toml` (version/config)
  - Workflow file itself

**Platforms Built**:
- Linux x86_64 (ubuntu-latest)
- Linux ARM64 (cross-compilation)
- macOS x86_64 (macos-13)
- macOS ARM64 (macos-latest)
- Windows x86_64 (windows-latest)

**Duration**: ~15-20 minutes (parallel builds)

### Old Workflow: `publish-feagi-staging.yml`

Simple workflow without binary building (for Python-only changes).

**Status**: Keep disabled or remove (use `publish-with-binaries.yml` instead)

## Testing the Workflow

### Dry Run (Manual Trigger)

1. Go to repository → Actions tab
2. Select "Build FEAGI Binaries and Publish SDK"
3. Click "Run workflow"
4. Select `staging` branch
5. Click "Run workflow"

This allows you to test without merging to staging.

### Monitor Progress

1. Go to Actions tab
2. Click on the running workflow
3. Monitor each job:
   - `build-linux-x86_64`
   - `build-linux-aarch64`
   - `build-macos-x86_64`
   - `build-macos-aarch64`
   - `build-windows-x86_64`
   - `publish` (waits for all builds)

### View Results

After successful run:
- Check the workflow summary for installation instructions
- Visit test.pypi.org/project/feagi/ to see published package
- Test installation on different platforms

## Workflow Details

### Build Jobs (Parallel)

Each platform builds independently:

```yaml
build-linux-x86_64:
  - Checkout feagi-python-sdk (public)
  - Checkout FEAGI-2.0 (private, using FEAGI_ACCESS_TOKEN)
  - Setup Rust toolchain
  - Build FEAGI for target platform
  - Upload binary as artifact
```

### Publish Job (After All Builds)

```yaml
publish:
  needs: [all-build-jobs]
  - Checkout feagi-python-sdk
  - Download all binary artifacts
  - Organize into feagi/bin/ structure
  - Build Python package
  - Publish to test.pypi.org (using TEST_PYPI_API_TOKEN)
```

## Version Management

**Before merging to staging**:

```bash
# Bump version (required for test.pypi.org)
cd feagi-python-sdk
python scripts/bump_version.py patch  # 2.0.0 → 2.0.1
# OR
python scripts/bump_version.py minor  # 2.0.0 → 2.1.0
# OR
python scripts/bump_version.py dev    # 2.0.0 → 2.0.0.dev1

# Commit
git add pyproject.toml feagi/__init__.py
git commit -m "Bump version to X.Y.Z"
git push origin staging
```

**Important**: Test PyPI does NOT allow re-uploading the same version!

## Troubleshooting

### "Authentication failed" during FEAGI checkout

**Problem**: `FEAGI_ACCESS_TOKEN` is invalid or missing

**Solution**:
1. Verify secret exists: Repository Settings → Secrets → `FEAGI_ACCESS_TOKEN`
2. Check token hasn't expired
3. Regenerate token if needed

### "Invalid credentials" during publish

**Problem**: `TEST_PYPI_API_TOKEN` is invalid

**Solution**:
1. Verify secret exists: Repository Settings → Secrets → `TEST_PYPI_API_TOKEN`
2. Check token is for test.pypi.org (not pypi.org)
3. Regenerate token if needed

### "File already exists" error

**Problem**: Version already published to test.pypi.org

**Solution**:
```bash
python scripts/bump_version.py patch  # Bump version
git add pyproject.toml feagi/__init__.py
git commit -m "Bump version"
```

### Build fails on specific platform

**Problem**: Rust compilation error on one platform

**Solution**:
1. Check the specific job logs
2. Test locally on that platform if possible
3. Fix FEAGI Rust code in FEAGI-2.0 repo
4. Re-run workflow

### Binaries not included in package

**Problem**: `pyproject.toml` or `MANIFEST.in` not configured correctly

**Solution**: Verify files:

```toml
# pyproject.toml
[tool.setuptools.package-data]
feagi = [
    "bin/linux-x86_64/feagi",
    "bin/linux-aarch64/feagi",
    "bin/darwin-x86_64/feagi",
    "bin/darwin-aarch64/feagi",
    "bin/windows-x86_64/feagi.exe",
]
```

```
# MANIFEST.in
recursive-include feagi/bin *
```

### Large package size warnings

**Expected**: Package will be 100-150MB due to bundled binaries

**This is normal** for packages with native binaries (PyTorch, TensorFlow are similar)

## Security Considerations

### Access Control

- `FEAGI_ACCESS_TOKEN` has **read-only** access to FEAGI-2.0
- Tokens are stored as **encrypted secrets**
- Tokens are never exposed in logs

### Binary Verification

Optional: Add checksums to verify binaries:

```yaml
- name: Generate checksums
  run: |
    cd feagi/bin
    sha256sum */* > CHECKSUMS.txt
    cat CHECKSUMS.txt
```

### Code Signing

For production (not test.pypi):

**macOS**:
```yaml
- name: Sign macOS binary
  env:
    CERTIFICATE: ${{ secrets.MACOS_CERTIFICATE }}
  run: |
    codesign -s "Developer ID" feagi/bin/darwin-*/feagi
```

**Windows**:
```yaml
- name: Sign Windows binary
  env:
    CERTIFICATE: ${{ secrets.WINDOWS_CERTIFICATE }}
  run: |
    signtool sign /f cert.pfx feagi/bin/windows-x86_64/feagi.exe
```

## Production Deployment

When ready for production PyPI:

1. **Create new workflow**: `publish-production.yml`
2. **Change trigger**: Push to `main` or GitHub releases
3. **Add secret**: `PYPI_API_TOKEN` (from https://pypi.org)
4. **Update repository URL**: Remove `--repository testpypi`
5. **Test thoroughly** on test.pypi first!

## Cost Considerations

### GitHub Actions Minutes

- **Free tier**: 2,000 minutes/month (private repos)
- **This workflow**: ~15-20 minutes per run
- **Runs per month**: ~100-130 builds possible

**Optimize**:
- Only trigger on `feagi/**` changes (not docs)
- Use manual triggers for testing
- Cache Rust compilation (saves 5-10 minutes)

### Storage

- **Artifacts retention**: 1 day (default: 90 days)
- Saves storage by auto-deleting build artifacts

## FAQ

### Q: Can I build for more platforms?

**A**: Yes! Add more jobs:

```yaml
build-linux-armv7:
  runs-on: ubuntu-latest
  steps:
    # ... cross-compile for armv7 ...
```

### Q: Can I skip specific platforms?

**A**: Yes! Comment out or remove jobs from `needs:` in publish job.

### Q: How do I test locally before CI/CD?

**A**: Use the build script:

```bash
cd feagi-python-sdk
./scripts/build_binaries.sh  # Builds for current platform only
python -m build
pip install dist/*.whl
```

### Q: Can I publish to both test and production PyPI?

**A**: Yes! Create two workflows or add conditions:

```yaml
- name: Publish to Test PyPI
  if: github.ref == 'refs/heads/staging'
  
- name: Publish to PyPI
  if: github.ref == 'refs/heads/main'
```

## Summary

✅ **Setup Once**:
1. Add `FEAGI_ACCESS_TOKEN` secret
2. Add `TEST_PYPI_API_TOKEN` secret

✅ **Then Automatically**:
1. Merge code to `staging`
2. Workflow builds for all platforms
3. Package published to test.pypi.org
4. Users can install with `pip install feagi`

**Your Rust code stays private!** 🔒


