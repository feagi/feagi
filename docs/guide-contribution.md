# Contributing to FEAGI

Thank you for your interest in contributing to FEAGI (Flexible & Extensible Artificial General Intelligence). This guide will help you understand how to effectively contribute to the project.

## Code of Conduct

All contributors are expected to adhere to our code of conduct, which promotes a respectful, inclusive, and collaborative environment. Key principles:

- Be respectful and inclusive
- Provide constructive feedback
- Focus on the best solution, not personal preferences
- Support fellow contributors
- Be patient with new contributors

## Getting Started

### Setup Development Environment

1. Fork the FEAGI repository
2. Clone your fork: 
   ```bash
   git clone https://github.com/YOUR-USERNAME/feagi.git
   ```
3. Create a virtual environment:
   ```bash
   python -m venv .venv
   source .venv/bin/activate  # On Windows: .venv\Scripts\activate
   ```
4. Install development dependencies:
   ```bash
   pip install -e ".[dev]"
   ```

### Project Structure

FEAGI is organized into several key modules:

- `/feagi/api/` - API interfaces and protocols
- `/feagi/bdu/` - Brain Development Unit for neural architecture
- `/feagi/npu/` - Neural Processing Unit for neural computation
- `/feagi/pns/` - Peripheral Nervous System for I/O
- `/feagi/core/` - Core infrastructure components
- `/feagi_bytes/` - Communication protocol library
- `/feagi_connector/` - Client library for agents
- `/tests/` - Test suite

## Contribution Workflow

### 1. Select an Issue

- Browse open [issues](https://github.com/feagi/feagi/issues) to find something to work on
- Filter by labels like "good first issue" or "help wanted"
- Comment on an issue to express interest before starting work
- If you have a new idea, create an issue first to discuss it

### 2. Create a Branch

```bash
git checkout -b feature/your-feature-name
```

Use prefixes for branch naming:
- `feature/` for new features
- `fix/` for bug fixes
- `docs/` for documentation changes
- `refactor/` for code refactoring

### 3. Development Guidelines

#### Coding Standards

- Follow [PEP 8](https://peps.python.org/pep-0008/) for Python code
- Use [Google style docstrings](https://google.github.io/styleguide/pyguide.html#38-comments-and-docstrings)
- Apply consistent naming conventions as documented in `guide-naming-conventions.md`
- Keep functions small and focused on a single responsibility
- Write self-documenting code with clear variable and function names

#### Testing

- Write tests for new features and bug fixes
- Ensure all tests pass before submitting a pull request:
  ```bash
  python -m pytest tests/
  ```
- Include both unit and integration tests when appropriate
- Aim for comprehensive test coverage

#### Documentation

- Update documentation for any code changes
- Follow the documentation standards in `guide-documentation-standards.md`
- Use appropriate prefixes for documentation files (`arch-`, `guide-`, `spec-`, etc.)
- Include code examples where helpful

### 4. Commit Changes

- Make small, focused commits
- Write clear commit messages using imperative mood:
  ```
  Add feature X to component Y
  Fix bug in Z calculation
  Update documentation for feature A
  ```
- Reference issue numbers in commit messages: `Fix #123: Address memory leak in FCL manager`

### 5. Submit a Pull Request

- Push your branch to your fork:
  ```bash
  git push origin feature/your-feature-name
  ```
- Create a pull request against the main FEAGI repository
- Provide a clear description of the changes and reference related issues
- Complete the pull request template with all required information

### 6. Code Review Process

- Maintainers will review your pull request
- Address any feedback or requested changes
- Ensure CI/CD checks pass
- Your contribution will be merged once approved

## Types of Contributions

### Code Contributions

- Feature development
- Bug fixes
- Performance optimizations
- Test coverage improvements

### Documentation Contributions

- Improving existing documentation
- Creating new guides or tutorials
- Adding code examples
- Fixing typos or clarifying content

### Other Contributions

- Issue reporting
- UX/UI improvements
- Build system enhancements
- Website improvements

## Architecture Decision Records (ADRs)

For significant architectural changes, please create an ADR:

1. Copy the ADR template from `docs/templates/adr-template.md`
2. Name it `adr-your-feature-name.md`
3. Fill in all sections clearly explaining the decision, alternatives considered, and rationale
4. Submit it with your pull request or as a separate PR for discussion

## Communication Channels

- GitHub Issues: For bug reports, feature requests, and general discussion
- Discord: For real-time communication and community support
- Weekly Dev Meetings: Join our open development meetings (see schedule in README)

## Module-Specific Guidelines

### API Module

- Follow REST API best practices
- Ensure backward compatibility
- Update API documentation for any changes

### BDU Module

- Maintain separation between genome and connectome
- Document any changes to cortical area structure
- Include tests for connectome generation

### NPU Module

- Focus on performance optimization
- Document computational approaches
- Test with realistic neural loads

### PNS Module

- Follow sensorimotor protocol specifications
- Test with different I/O scenarios
- Document interface changes

### Core Module

- Minimize dependencies between components
- Focus on stability and error handling
- Address thread safety and concurrency issues

## License

By contributing to FEAGI, you agree that your contributions will be licensed under the project's Apache 2.0 License. 