# FEAGI Documentation Standards

## Naming Convention

### Core Principles

1. **Consistency**: All documentation follows the same pattern
2. **Descriptiveness**: Names clearly indicate content
3. **Navigability**: Easy to locate specific information
4. **Modularity**: Supports both system and module documentation

### File Naming Guidelines

#### 1. Basic Format
- Use `kebab-case-lowercase.md` for all documentation files
- Example: `system-architecture.md` instead of `system_architecture.md` or `SystemArchitecture.md`

#### 2. Document Type Prefixes
All documentation should start with a prefix indicating the document type:

| Prefix | Document Type | Example |
|--------|---------------|---------|
| `arch-` | Architecture documents | `arch-system-overview.md` |
| `spec-` | Technical specifications | `spec-shared-memory-protocol.md` |
| `guide-` | Developer/user guides | `guide-coding-standards.md` |
| `adr-` | Architecture Decision Records | `adr-api-refactoring.md` |
| `plan-` | Project planning | `plan-refactoring-strategy.md` |

#### 3. Special Names (Exceptions)
Retain standardized names that developers expect to find:
- `README.md` - Directory overview (always uppercase)
- `CONTRIBUTING.md` - Contribution guidelines (always uppercase)
- `CHANGELOG.md` - Version history (always uppercase)
- `LICENSE.md` - Legal information (always uppercase)

#### 4. Version Indicators (when needed)
For versioned documents, append version after content name:
- `spec-protocol-v1.md`
- `arch-storage-v2.md`

#### 5. Directory-Specific Documentation
For module documentation placed within module directories:
- Use the same naming convention but prefix with `README.md` for the main module documentation
- For additional documents: `arch-module-name-component.md`

## Documentation Structure

### System Documentation (`/docs` folder)
The `/docs` folder should contain only system-level documentation:
- Architecture overviews
- Technical specifications
- Development guidelines
- Project planning
- Architecture Decision Records (ADRs)

### Module Documentation (module folders)
Module-specific documentation should live alongside the code:
- Each module directory should have a `README.md`
- Additional documentation follows the naming convention above
- Implementation details should be documented close to the code

## Content Guidelines

### Each document should:
1. Have a clear purpose stated at the beginning
2. Include a last-updated date
3. Reference related documents where applicable
4. Use consistent Markdown formatting
5. Include diagrams where helpful (stored in a `/docs/assets` folder)
