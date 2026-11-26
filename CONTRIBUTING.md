# Contributing to FEAGI

Thanks for helping build FEAGI. This guide covers how the project is organized, how to set up a
development environment, what a pull request must pass, and the design rules every change is
reviewed against.

If you only have a few minutes: open an issue before large changes, branch from `staging`, run
the checks listed for the repository you touched, and keep each pull request to one change.

## Where things live

FEAGI is split across several repositories under [github.com/feagi](https://github.com/feagi).
Open issues and pull requests in the repository that owns the code.

| Repository | What it holds | Language |
| --- | --- | --- |
| [feagi/feagi](https://github.com/feagi/feagi) (this repo) | Python SDK, `feagi` CLI, agent and embodiment connectors, packaging | Python |
| [feagi/feagi-core](https://github.com/feagi/feagi-core) | Neural engine: burst engine, genome, connectome, plasticity, API, Rust agent SDK | Rust |
| [feagi/feagi-rust-py-libs](https://github.com/feagi/feagi-rust-py-libs) | Python bindings to the Rust core | Rust, Python |
| [feagi/brain-visualizer](https://github.com/feagi/brain-visualizer) | 3D brain visualizer | Godot |
| [feagi/feagi-mcp](https://github.com/feagi/feagi-mcp) | MCP server and `feagi` skill for AI assistants | Python |
| [feagi/controllers](https://github.com/feagi/controllers) | Embodiment controllers for robots and simulators | Python |
| [feagi/feagi-java-sdk](https://github.com/feagi/feagi-java-sdk) | Java SDK | Java |

Not sure where something belongs? Ask in [Discussions](https://github.com/feagi/feagi/discussions)
or on [Discord](https://discord.BrainsForRobots.com).

## Ways to contribute

- **Report a bug.** Use the bug report template. Include the FEAGI version (`pip show feagi`),
  your OS and Python version, the genome or embodiment involved, the exact steps, and what you
  expected versus what happened. Logs and a minimal genome make a bug far faster to fix.
- **Propose a feature.** Use the feature request template and describe the problem first, then
  the solution you have in mind. For anything that changes an API, a genome format or the
  connector protocol, wait for a maintainer's reply before writing code.
- **Fix a bug or build a feature.** Comment on the issue so others know it is taken.
- **Improve the docs.** Typos, unclear steps and missing examples are all welcome as small PRs.
- **Share a brain or an embodiment.** Reusable genomes and controllers help everyone; describe
  what the brain does and which embodiment it was tested on.

## Branches and pull requests

- `staging` is the default branch and the target of every pull request. Releases are cut from it.
- Fork the repository, create a branch from `staging`, and name it for the change
  (`fix/serial-reconnect`, `feature/imu-connector`, `docs/quickstart`).
- One logical change per pull request. Split refactors from behavior changes so each can be
  reviewed on its own.
- Write the PR description for a reviewer who has not seen the issue: what changed, why, how you
  tested it, and anything you were unsure about. Link the issue (`Fixes #123`).
- Keep the branch current with `staging` and resolve conflicts on your side.
- A maintainer reviews every pull request. Expect questions; they are about the code, not you.

## Development setup (this repository)

Requires Python 3.10 or later (3.10, 3.11 and 3.12 are supported).

```bash
git clone git@github.com:<your-username>/feagi.git
cd feagi
git checkout staging
python -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install -e ".[dev]"
pre-commit install
```

Optional extras: `.[bv]` installs the Brain Visualizer, `.[video]` adds OpenCV, and
`.[bluetooth]` adds BLE support.

To run FEAGI locally while you work:

```bash
feagi start
feagi bv start
```

### Checks before you open a pull request

```bash
pre-commit run --all-files      # ruff lint and format, plus file hygiene checks
pytest                          # unit tests
```

CI fails on undefined names, import errors and syntax errors, so a clean `ruff check .` matters.
Run tests that need a live FEAGI instance (under `tests/integration/`) when your change touches
the client, connectors or the CLI, and say in the PR whether you ran them.

## Working on feagi-core (Rust)

The Rust toolchain is pinned in `rust-toolchain.toml`; `rustup` picks it up automatically. A pull
request must pass the same checks CI runs:

```bash
cargo fmt --all -- --check
cargo clippy --workspace --lib --tests -- -D warnings
cargo test --workspace --lib
```

CI also runs the integration suites and a `cargo-deny` license check, so a new dependency must
carry a license the project accepts. While iterating, test the crate you changed
(`cargo test -p <crate>`) rather than the whole workspace.

## Code standards

- **Tests come with the change.** A bug fix includes a test that fails without it. A feature
  includes tests for its main path and its edge cases.
- **No silent failures.** Raise or return a clear error with context. Log failed API calls with
  their status and body; do not show raw errors to end users.
- **Public APIs are documented.** Every public function, class and endpoint has a docstring or
  doc comment that says what it does, its inputs, and what it returns or raises.
- **Keep dependencies lean.** Add one only when it earns its place, and say why in the PR.
- **Comments explain why**, not what the code already says.

## Design rules

These rules come from problems the project has already paid for. Reviews hold every change to
them.

### Generic layers stay generic

The core engine, the SDK, simulator controllers (MuJoCo, Gazebo, Webots and others) and shared
tools must work for any robot, genome and experiment.

- Never put a robot's name, its body or joint names, or constants sized for one robot into a
  generic layer.
- Robot-specific values come from the embodiment's configuration or the loaded model (mass,
  joint limits, keyframes by name) at runtime.
- A fix for something one robot exposed must be the general rule that covers every body.
- Defaults are dimensionless or physically general: angles, fractions, seconds.
- Test fixtures and examples use neutral names.

A controller for one specific robot or platform may be as custom as that robot needs; this rule
applies to the layers everyone shares.

### Behavior comes from the brain

When a change is about how a robot behaves, the behavior must come from the genome. Scripted
motions, offline replicas or controller tricks that stand in for the brain will not be merged.
Embodiment settings that fix a transport problem (sensor cutoffs, servo slew limits) are fine;
call them out as such in the PR.

### Every control explains itself

In every UI, each chip, status badge, toggle, icon-only button and abbreviated label has a
tooltip that says what it means and, if it is clickable, what clicking does. A tooltip that only
repeats the label does not count. Before submitting UI work, check every control on the screen
you touched.

### State that survives

Settings that are saved must come back after a save, restart or reload. When you add a setting,
test the round trip by reloading, not by reading back the value you just set.

## Commit messages

- Use the imperative mood and keep the subject under about 72 characters: `Fix serial
  reconnect after device reset`.
- Add a body when the reason is not obvious from the subject.
- Reference issues where relevant (`Refs #123`).

## Security

Do not open a public issue for a security vulnerability. Email **feagi@neuraville.com** with a
description, steps to reproduce and the potential impact. See [SECURITY.md](SECURITY.md) for
details.

## Community

- [Discussions](https://github.com/feagi/feagi/discussions) for questions, ideas and show-and-tell
- [Discord](https://discord.BrainsForRobots.com) for real-time help
- Email: feagi@neuraville.com

Be respectful and assume good intent. Criticize code and ideas, never people.

## License

FEAGI is licensed under the [Apache License 2.0](LICENSE). By submitting a contribution, you agree
that it is licensed under the same terms.
