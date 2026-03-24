<!--
SYNC IMPACT REPORT
==================
Version change  : (none) → 1.0.0
Type of bump    : MAJOR — initial ratification (all placeholders filled)
Modified        : N/A (initial creation)
Added sections  : Core Principles, Technology & Tooling, Development Workflow, Governance
Removed sections: N/A
Templates reviewed:
  ✅ .specify/memory/constitution.md       — this file (created)
  ✅ .specify/templates/plan-template.md   — no changes required; language-agnostic
  ✅ .specify/templates/spec-template.md   — no changes required
  ✅ .specify/templates/tasks-template.md  — no changes required
Deferred TODOs  : None
-->

# path_generation Constitution

## Core Principles

### I. Rust-First

All production source code MUST be written in Rust (stable channel).
No other systems-programming language may be introduced without a formal constitution amendment.
`unsafe` blocks are permitted only when safe alternatives are genuinely unavailable; each
occurrence MUST include an inline comment that explains why safety cannot be upheld by safe
Rust and MUST receive explicit approval during code review.

### II. Safety & Correctness

Rust's ownership model and type system are the primary correctness mechanism and MUST be
used to their full extent.
Library code MUST NOT panic; all fallible operations MUST return `Result<T, E>` or `Option<T>`.
`clippy` warnings are treated as errors in CI (`-D warnings`).
`rustfmt` formatting MUST be applied before every commit; violations block CI.

### III. Test-First (NON-NEGOTIABLE)

TDD is mandatory: tests MUST be written and approved before implementation begins.
The Red-Green-Refactor cycle is strictly enforced.
`cargo test` MUST pass on all commits; un-tested code MUST NOT be merged to the main branch.
Integration tests MUST cover inter-module contracts whenever a public API is introduced or
changed.

### IV. Performance-Aware Design

Path generation operations MUST be analyzed for algorithmic complexity before implementation.
`cargo bench` benchmarks MUST accompany any change to a performance-critical code path.
Unnecessary heap allocations MUST be avoided; prefer stack-allocated types and iterators.
Performance regressions detected by benchmarks MUST be addressed before a PR is merged.

### V. Simplicity & Minimal Dependencies

YAGNI: implement only what is required by the current specification.
Every crate added to `Cargo.toml` MUST be justified in the PR description; prefer standard
library solutions over third-party alternatives wherever practical.
Transitive dependency security audits MUST be performed via `cargo audit` in CI.
Complexity MUST be justified by explicit requirements, not anticipated future needs.

## Technology & Tooling

All development MUST use the following toolchain and conventions:

- **Language**: Rust (stable channel); MSRV MUST be declared via `rust-version` in `Cargo.toml`.
- **Build System**: Cargo; a workspace layout MUST be adopted if the project grows beyond one crate.
- **Formatting**: `rustfmt` with a project-level `rustfmt.toml`; enforced in CI via
  `cargo fmt -- --check`.
- **Linting**: `cargo clippy -- -D warnings`; all warnings are treated as errors in CI.
- **Testing**: `cargo test` for unit and integration tests; `cargo bench` for benchmarks.
- **Security Audit**: `cargo audit` MUST run in CI; known vulnerabilities block merges.
- **Documentation**: All public APIs MUST have `///` doc comments; `cargo doc` MUST build
  without warnings.

## Development Workflow

- All work MUST be done on feature branches; the main branch MUST remain releasable at all times.
- CI MUST enforce: `cargo fmt --check`, `cargo clippy -- -D warnings`, `cargo test`,
  `cargo audit`.
- Every pull request requires at least one code-review approval; changes to public APIs require
  an additional constitution sync check before merge.
- Commit messages MUST follow Conventional Commits (`feat:`, `fix:`, `docs:`, `chore:`, etc.).
- Releases MUST follow Semantic Versioning (SemVer); `CHANGELOG.md` MUST be updated on each
  release.

## Governance

This constitution is the highest-authority document for the `path_generation` project.
All development practices, tooling decisions, and architectural choices MUST comply with it.
Amendments require a pull request targeting `.specify/memory/constitution.md` with a documented
rationale; the version line MUST be bumped according to the semantic versioning rules defined
in the `speckit.constitution` agent (MAJOR for governance/principle removals or redefinitions,
MINOR for new sections or materially expanded guidance, PATCH for clarifications and wording
fixes).
All PRs and code reviews MUST explicitly verify compliance with this constitution.

**Version**: 1.0.0 | **Ratified**: 2026-03-24 | **Last Amended**: 2026-03-24
