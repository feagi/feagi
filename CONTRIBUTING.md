## Table of Contents

- [Development Environment Setup](#development-environment-setup)
- [Codebase Structure](#codebase-structure)
- [The Workflow](#the-workflow)
  - [Nightly Checkouts and Pulls](#nightly-checkouts-and-pulls)
- [Coding Standards](#coding-standards)
- [Writing Documentation](#writing-documentation)
- [Testing](#testing)
  - [Unit Testing](#unit-testing)
  - [Contineous Integration](#contineous-integration)
- [Performance Profiling](#performance-profiling)
- [Sharing Your Contributions](#sharing-your-contributions)

## Contributing to FEAGI

We are excited to see your interest in contributing to FEAGI! The FEAGI community values your contributions and we look forward to having you as a long-term member.

## Getting Started

---

## Development Environment Setup

## Codebase Structure

## The Workflow

### Nightly Checkouts and Pulls

## Coding Standards

FEAGI Python code contributions should adhere to the standards described in [PEP 8](https://www.python.org/dev/peps/pep-0008/). All Python code contributions should be documented in accordance with [PEP 257](https://www.python.org/dev/peps/pep-0257/) conventions. Static code analysis tools like [Pylint](https://pypi.org/project/pylint/) and [flake8](https://pypi.org/project/flake8/) provide configurable, automated solutions for checking and enforcing PEP compliance. The `feagi` [repository](https://github.com/feagi/feagi) continuous integration (CI) workflow uses Pylint to analyze Python code contributions. A CI workflow job will fail if Pylint code analysis results in a code quality score below a defined threshold.

## Writing Documentation

## Testing

### Unit Testing

### Continuous Integration

## Performance Profiling

## Sharing Your Contributions

If you haven't done it before (or in a while), here is how to push your changes for review using git/Github:

1. Fork the repo.
2. Clone your fork (if working locally).
3. Create a new branch for your edits, and add and commit them.
4. If working locally, push the branch to your forked repo.
5. Click on create pull request for your branch. Select the actual repo (should be selected by default), not your forked repo, as the base repository.
6. Create the pull request and await review!
