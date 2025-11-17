# FEAGI Release Checklist

Use this checklist before publishing your first release to PyPI.

## One-Time Setup (Do Once)

### 1. PyPI Account Setup
- [ ] Create account on https://pypi.org/
- [ ] Verify email address
- [ ] Add team members to project (after first upload)

### 2. Configure Trusted Publishing (Recommended)
- [ ] Go to https://pypi.org/manage/account/publishing/
- [ ] Add new trusted publisher:
  - PyPI Project Name: `feagi`
  - Owner: `neuraville` (your GitHub org)
  - Repository: `feagi`
  - Workflow: `build-wheels.yml`
  - Environment: `pypi`
- [ ] Save configuration

### 3. Alternative: API Token (If Not Using Trusted Publishing)
- [ ] Generate API token on PyPI
- [ ] Add token to GitHub Secrets as `PYPI_API_TOKEN`
- [ ] Update workflow to use token authentication

### 4. GitHub Environment Setup
- [ ] Go to repository Settings → Environments
- [ ] Create environment named `pypi`
- [ ] Add protection rules (optional but recommended):
  - Required reviewers: 1+
  - Restrict to main branch

### 5. Test Setup with TestPyPI (Optional but Recommended)
- [ ] Create account on https://test.pypi.org/
- [ ] Configure trusted publishing for TestPyPI
- [ ] Run test workflow: Actions → Build and Publish Wheels → "Run workflow" → Check TestPyPI
- [ ] Verify test installation works

## Pre-Release Checklist (Do Before Each Release)

### 1. Code Preparation
- [ ] All tests passing on main branch
- [ ] Architecture compliance tests passing
- [ ] No outstanding critical bugs
- [ ] Code review completed (if applicable)

### 2. Version Management
- [ ] Update version in `feagi_core/pyproject.toml`
  ```toml
  [project]
  version = "0.1.0"  # ← Update this
  ```
- [ ] Update CHANGELOG.md with release notes (create if doesn't exist)
- [ ] Update README.md if needed
- [ ] Commit changes: `git commit -m "Bump version to 0.1.0"`
- [ ] Push to main: `git push origin main`

### 3. Local Testing (Recommended)
- [ ] Run local wheel build test:
  ```bash
  cd feagi_core
  ./scripts/test_wheel_build.sh
  ```
- [ ] Verify wheel builds successfully
- [ ] Verify Rust extensions load
- [ ] Clean up: `rm -rf dist/ build/`

### 4. Documentation Review
- [ ] README.md installation instructions are current
- [ ] API documentation is up to date
- [ ] Example code works with new version

## Release Process

### 1. Create Git Tag
```bash
git tag -a v0.1.0 -m "Release version 0.1.0"
git push origin v0.1.0
```

### 2. Create GitHub Release
- [ ] Go to https://github.com/neuraville/feagi/releases/new
- [ ] Select tag: `v0.1.0`
- [ ] Release title: `FEAGI v0.1.0`
- [ ] Description: Copy from CHANGELOG or write release notes
- [ ] Check "Set as the latest release"
- [ ] Click "Publish release"

### 3. Monitor CI Build
- [ ] Go to Actions tab: https://github.com/neuraville/feagi/actions
- [ ] Find "Build and Publish Wheels" workflow
- [ ] Monitor build progress (~30-45 minutes)
- [ ] Check all platform builds succeed:
  - [ ] Ubuntu (Linux x86_64)
  - [ ] macOS Intel
  - [ ] macOS Apple Silicon
  - [ ] Windows

### 4. Verify Publication
- [ ] Wait 2-3 minutes for PyPI to update
- [ ] Check PyPI page: https://pypi.org/project/feagi/
- [ ] Verify version shows correctly
- [ ] Verify all wheels are present (~16-20 files)
- [ ] Check that README renders correctly on PyPI

### 5. Test Installation
```bash
# Create clean test environment
python3 -m venv test-install-venv
source test-install-venv/bin/activate

# Install from PyPI
pip install feagi==0.1.0

# Verify imports
python -c "import feagi; print(f'Version: {feagi.__version__}')"
python -c "from feagi import feagi_rust; print('Rust extensions OK')"

# Test basic functionality
python -m feagi.main --help

# Cleanup
deactivate
rm -rf test-install-venv
```

## Post-Release

### 1. Announcement
- [ ] Announce on project communication channels
- [ ] Update project website (if applicable)
- [ ] Post on social media (if applicable)
- [ ] Notify dependent projects

### 2. Monitoring
- [ ] Monitor PyPI download stats
- [ ] Watch for installation issues in issue tracker
- [ ] Check for platform-specific problems

### 3. Next Development Cycle
- [ ] Create development branch for next version (if using)
- [ ] Update version to next dev version (e.g., 0.2.0-dev)
- [ ] Plan features for next release

## Troubleshooting During Release

### Workflow Fails to Build Wheels
1. Check Actions logs for specific error
2. Look for Rust compilation errors
3. Check cibuildwheel configuration
4. Test locally with `test_wheel_build.sh`
5. Fix issues and push to main
6. Delete and recreate the release/tag

### Publishing to PyPI Fails
1. Verify trusted publishing configuration
2. Check GitHub environment name: `pypi`
3. Check workflow file name: `build-wheels.yml`
4. Verify tag format starts with 'v'
5. Check PyPI token if using token auth

### Wheels Install But Don't Work
1. Check platform compatibility
2. Verify Rust extensions compiled correctly
3. Check for missing dependencies
4. Test in clean Docker container
5. Consider issuing patch release

### Version Already Exists on PyPI
- PyPI doesn't allow overwriting versions
- Must increment version and create new release
- Delete GitHub release, bump version, try again

## Emergency Rollback

If a release has critical issues:

1. **Mark as Pre-release on GitHub**
   - Edit the GitHub release
   - Check "This is a pre-release"

2. **Yank from PyPI (doesn't delete, just warns)**
   - Go to https://pypi.org/project/feagi/
   - Manage releases → Yank version
   - Add reason for yanking

3. **Release Hotfix**
   - Create patch version (e.g., 0.1.1)
   - Fix critical issue
   - Follow release process again

## Support Contacts

- **CI/CD Issues**: DevOps team
- **PyPI Issues**: https://pypi.org/help/
- **Wheel Building**: cibuildwheel docs
- **GitHub Actions**: GitHub support

---

**First Release?** Start with the One-Time Setup section.

**Ready to Release?** Follow the Pre-Release Checklist → Release Process.

