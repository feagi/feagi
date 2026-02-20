# FEAGI Code Quality Guidelines

## Preventing Duplicate Method Definitions

Duplicate method definitions are a serious code quality issue that can lead to unexpected behavior, debugging difficulties, and maintenance problems. This document outlines guidelines and tools to prevent this issue.

## The Problem

When Python encounters duplicate method definitions in the same class:
- The **last defined method overwrites** previous definitions
- Earlier definitions become **dead code** (unreachable)
- Can cause **difficult-to-debug issues** where expected behavior doesn't occur
- Creates **maintenance confusion** about which implementation is active

### Example of the Problem

```python
class MyClass:
    def process_data(self, data):
        """First implementation - becomes dead code"""
        return data.upper()

    # ... 1000 lines of other code ...

    def process_data(self, data):
        """Second implementation - this one wins"""
        return data.lower()
```

## Prevention Strategies

### 1. Automated Detection Tools

We have implemented several automated tools to catch duplicates:

#### A. Duplicate Method Checker Script
```bash
# Run the simple duplicate checker
python3 duplicate_method_checker.py

# Run comprehensive code quality check
python3 scripts/code_quality_check.py --directory feagi
```

#### B. Pre-commit Hooks
Install pre-commit hooks to catch issues before commit:
```bash
pip install pre-commit
pre-commit install
```

#### C. CI/CD Integration
The code quality checker can be integrated into CI/CD pipelines:
```bash
# In CI/CD, fail on any quality issues
python3 scripts/code_quality_check.py --fail-on-warnings
```

### 2. Development Best Practices

#### A. File Organization
- Keep classes **small and focused** (< 1000 lines)
- Split large classes into **multiple specialized classes**
- Use **clear section comments** to organize methods

#### B. Method Organization
```python
class WellOrganizedClass:
    # =================================================================
    # INITIALIZATION METHODS
    # =================================================================

    def __init__(self, ...):
        pass

    # =================================================================
    # PUBLIC API METHODS
    # =================================================================

    def public_method_1(self, ...):
        pass

    def public_method_2(self, ...):
        pass

    # =================================================================
    # PRIVATE HELPER METHODS
    # =================================================================

    def _private_method_1(self, ...):
        pass
```

#### C. Code Review Checklist
Before merging any PR, verify:
- [ ] No duplicate method definitions
- [ ] Methods are reasonably sized (< 50 lines recommended)
- [ ] Clear method organization with section comments
- [ ] No syntax errors
- [ ] Reasonable file size (< 1000 lines recommended)

### 3. IDE Configuration

#### A. VS Code / Cursor
Add these settings to detect potential issues:
```json
{
    "python.linting.enabled": true,
    "python.linting.pylintEnabled": true,
    "python.analysis.typeCheckingMode": "strict"
}
```

#### B. PyCharm
Enable inspections:
- Code → Inspect Code
- Enable "Duplicate code fragment" inspection
- Enable "Unreachable code" inspection

## Automated Tools Usage

### Quick Duplicate Check
```bash
# Check specific file
python3 duplicate_method_checker.py

# Results show duplicates with line numbers
```

### Comprehensive Quality Check
```bash
# Check entire feagi directory
python3 scripts/code_quality_check.py

# Check with custom limits
python3 scripts/code_quality_check.py --max-method-lines 40 --max-file-lines 800

# Fail CI on any warnings
python3 scripts/code_quality_check.py --fail-on-warnings
```

### Integration with Git Hooks

#### Pre-commit Hook
Add to `.git/hooks/pre-commit`:
```bash
#!/bin/bash
echo "Running FEAGI code quality checks..."
python3 scripts/code_quality_check.py --fail-on-warnings
if [ $? -ne 0 ]; then
    echo "❌ Code quality issues found. Fix before committing."
    exit 1
fi
echo "✅ Code quality check passed."
```

#### Pre-push Hook
Add to `.git/hooks/pre-push`:
```bash
#!/bin/bash
echo "Running comprehensive FEAGI code quality audit..."
python3 scripts/code_quality_check.py --directory feagi --fail-on-warnings
if [ $? -ne 0 ]; then
    echo "❌ Code quality audit failed. Fix before pushing."
    exit 1
fi
echo "✅ Code quality audit passed."
```

## Emergency Recovery Procedures

If duplicate methods are discovered in production:

### 1. Immediate Assessment
```bash
# Run emergency duplicate detection
python3 duplicate_method_checker.py
```

### 2. Impact Analysis
- Identify which method definition is active (the last one)
- Check if any code depends on the "dead" method behavior
- Review recent changes that might have introduced the duplicate

### 3. Fix Strategy
```python
# Option A: Remove the duplicate (if identical)
# Delete the earlier definition

# Option B: Merge implementations (if different)
def combined_method(self, ...):
    """Combined implementation addressing both use cases"""
    # Merge logic from both methods
    pass

# Option C: Rename methods (if serving different purposes)
def method_variant_a(self, ...):
    """Original behavior"""
    pass

def method_variant_b(self, ...):
    """New behavior"""
    pass
```

## Quality Metrics

### Target Goals
- **Zero duplicate methods** in production code
- **< 50 lines per method** (warnings at 50, errors at 75)
- **< 1000 lines per file** (warnings at 1000, errors at 1500)
- **< 6 levels of nesting** (warnings at 6, errors at 8)

### Monitoring
Run quality checks regularly:
```bash
# Weekly quality audit
python3 scripts/code_quality_check.py --directory feagi > quality_report.txt

# Track improvements over time
git log --oneline --grep="quality\|duplicate\|refactor"
```

## Team Guidelines

### For Developers
1. **Always run quality checks** before committing
2. **Refactor large methods** into smaller, focused functions
3. **Use descriptive method names** to avoid naming conflicts
4. **Add section comments** to organize large files

### For Code Reviewers
1. **Check for duplicates** manually during review
2. **Verify quality tool passes** before approval
3. **Suggest refactoring** for methods > 50 lines
4. **Question large files** > 1000 lines

### For CI/CD
1. **Fail builds** on duplicate methods (critical)
2. **Warn on large methods** (recommended fix)
3. **Generate quality reports** for trending analysis
4. **Block merges** on critical quality issues

## Tools Reference

### Available Scripts
- `duplicate_method_checker.py` - Simple duplicate detection
- `scripts/code_quality_check.py` - Comprehensive quality analysis
- `.pre-commit-config.yaml` - Pre-commit hook configuration

### Exit Codes
- `0` - No issues found
- `1` - Critical issues found (duplicates, syntax errors)
- `2` - Warnings found (large methods, files)

### Configuration
Quality thresholds can be adjusted via command line arguments or environment variables.

## Support

For questions about code quality tools or guidelines:
- Check this documentation first
- Run `python3 scripts/code_quality_check.py --help`
- Create an issue in the repository for tool improvements
