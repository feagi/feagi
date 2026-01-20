# Release Guide

This guide documents the release process for publishing FEAGI SDK packages to PyPI.

## Package Structure

As of version 2.1.0, the SDK is distributed as two packages:

- **feagi-core** - Core SDK without Brain Visualizer (~5MB)
- **feagi** - Meta-package including feagi-core + Brain Visualizer (~5MB meta-package)

Both packages use identical imports and are versioned together.

## Release Process

FEAGI uses Git releases to trigger automated publishing to PyPI. The version is extracted from the Git tag.

### Overview

1. Merge changes to appropriate branch
2. Create GitHub release with version tag
3. GitHub Action automatically publishes to PyPI

## Versioning

FEAGI follows [Semantic Versioning](https://semver.org/):

```
MAJOR.MINOR.PATCH[-PRERELEASE]
```

- **MAJOR** - Breaking changes requiring user action
- **MINOR** - New features, backward compatible
- **PATCH** - Bug fixes only

### Pre-release Identifiers

- `a` - Alpha (early testing, unstable)
- `b` - Beta (feature complete, testing phase)
- `rc` - Release Candidate (final testing)

Examples: `X.Y.Za1`, `X.Y.Zb1`, `X.Y.Zrc1`

## Release Types

### Pre-releases

**Branch:** staging

**Tag format:** `vX.Y.Zb1` (e.g., v2.1.0b1)

**PyPI behavior:**
- Not installed by default
- Requires explicit version or `--pre` flag

### Stable Releases

**Branch:** main

**Tag format:** `vX.Y.Z` (e.g., v2.1.0)

**PyPI behavior:**
- Default installation target
- Recommended for all users

## Publishing Pre-release

### 1. Prepare Branch

```bash
git checkout staging
git pull origin staging
```

Verify all changes are merged and tested.

### 2. Create GitHub Release

Navigate to repository releases and create new release:

- **Tag:** `vX.Y.Zb1` (create new)
- **Target:** staging
- **Title:** `FEAGI SDK vX.Y.Z Beta 1`
- **Pre-release:** Checked
- **Latest:** Optional

Add release notes describing changes.

### 3. Monitor Workflow

GitHub Action runs automatically:
- Extracts version from tag
- Updates version in pyproject.toml and __init__.py
- Builds both feagi-core and feagi packages
- Publishes to PyPI (feagi-core first, then feagi)

Monitor: Actions → Publish to PyPI - FEAGI SDK

### 4. Verify

```bash
pip install feagi==X.Y.Zb1
python -c "import feagi; print(feagi.__version__)"
```

Check PyPI: https://pypi.org/project/feagi/

## Publishing Stable Release

### 1. Merge to Main

```bash
git checkout staging
git pull origin staging

git checkout main
git pull origin main
git merge staging
git push origin main
```

### 2. Create GitHub Release

Navigate to repository releases and create new release:

- **Tag:** `vX.Y.Z` (create new)
- **Target:** main
- **Title:** `FEAGI SDK vX.Y.Z`
- **Pre-release:** Unchecked
- **Latest:** Checked

Add comprehensive release notes.

### 3. Monitor Workflow

GitHub Action publishes automatically. Monitor progress in Actions tab.

### 4. Verify

```bash
pip install feagi
python -c "import feagi; print(feagi.__version__)"
```

Confirm version appears as default on PyPI.

## Manual Publishing

If automated workflow fails:

1. Navigate to: Actions → Publish to PyPI - FEAGI SDK
2. Click "Run workflow"
3. Enter version (e.g., `X.Y.Zb1`)
4. Enable "Dry run" for testing
5. Review build output
6. Run again without dry run to publish

## Pre-requisites

Automated workflow requires:

1. **Bundled binaries** - All platform binaries must be committed:
   - `feagi/bin/linux-x86_64/feagi`
   - `feagi/bin/linux-aarch64/feagi`
   - `feagi/bin/darwin-x86_64/feagi`
   - `feagi/bin/darwin-aarch64/feagi`
   - `feagi/bin/windows-x86_64/feagi.exe`

2. **PyPI token** - `PYPI_PASSWORD_TOKEN` secret configured

3. **Permissions** - Workflow has `id-token: write` permission

## Release Checklist

Before creating release:

- [ ] All tests passing
- [ ] Linting passes (`ruff check`)
- [ ] Binaries committed for all platforms
- [ ] CHANGELOG updated (if maintained)
- [ ] Breaking changes documented
- [ ] Migration guide prepared (for breaking changes)

## Version Strategy

### When to Use Pre-releases

Use pre-releases for:
- New major features requiring testing
- Breaking API changes
- Significant architectural changes
- Changes affecting multiple subsystems

Skip pre-releases for:
- Minor bug fixes
- Documentation updates
- Small improvements

### Beta Cycle Example

```
X.Y.0 (current stable)
  ↓
X.Y+1.0b1 (initial beta)
  ↓
X.Y+1.0b2 (bug fixes)
  ↓
X.Y+1.0rc1 (release candidate)
  ↓
X.Y+1.0 (new stable)
```

## Troubleshooting

### Version Already Exists

PyPI does not allow replacing versions. Increment to next version (e.g., b1 → b2).

### Binary Missing

Workflow validates binaries before building. If missing, download from feagi-rs releases and commit.

### Import Errors After Publishing

Verify package structure:
```bash
python -m zipfile -l dist/*.whl | grep "feagi/"
```

### Publishing Order

For package split:
1. feagi-core must publish first
2. feagi meta-package publishes second

GitHub Action handles this automatically.

## Package Dependencies

### feagi-core (base)
- Contains all SDK code
- No dependency on feagi-bv

### feagi (meta)
- Depends on feagi-core==X.Y.Z (exact version)
- Depends on feagi-bv platform packages

### feagi-bv packages
- Depend on feagi-core>=X.Y.Z
- Published from brain-visualizer repository

## References

- PyPI Project: https://pypi.org/project/feagi/
- GitHub Actions: .github/workflows/publish_pypi_feagi_sdk.yml
- Semantic Versioning: https://semver.org/
- PEP 440 (Python versioning): https://peps.python.org/pep-0440/
