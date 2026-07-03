# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Commands

This is a Cargo workspace with two crates (root `Cargo.toml` doubles as both a package manifest for `m2s2-math` and the workspace root). Because the workspace root is a non-virtual manifest, **bare `cargo test`/`build`/`clippy`/`fmt` only operate on `m2s2-math` and silently skip `m2s2-geometry`** — always pass `--workspace` (or `--all` for fmt).

- Build: `cargo build --workspace`
- Test all: `cargo test --workspace`
- Single test: `cargo test --workspace <test_name>` (test names are unique enough not to need `-p`)
- Lint (matches CI exactly): `cargo clippy --workspace --all-targets -- -D warnings`
- Format check: `cargo fmt --check --all`
- Format: `cargo fmt --all`
- Package/verify locally without publishing: `cargo package -p m2s2-math --allow-dirty` (m2s2-geometry's package-verify step will fail locally unless the m2s2-math version it depends on is already live on crates.io — see "Versioning gotcha" below; this is expected, not a bug)

Pre-commit hook mirrors the above (fmt check, clippy, test) and blocks broken commits. Enable once per clone:
```
git config core.hooksPath .githooks
```

## Architecture

Two-crate workspace, one-directional dependency:
- **`m2s2-math`** (root package, `src/`) — allocation-free linear algebra: `Vector<T, const D: usize>`, `Matrix2x2/3x3/4x4`, `Quaternion<T>`. All types are fixed-size `Copy`, zero heap allocation, by design (see README "Allocation strategy" section) — this must not change.
- **`m2s2-geometry`** (`m2s2-geometry/`, workspace member) — geometry primitives and intersection queries built on `m2s2-math`: `Aabb2/3`, `Circle`/`Sphere`, `Ray2/3`, `Plane`, `Triangle3`, `Obb2/3`. Split into its own crate specifically so a renderer that only needs vector/matrix/quaternion math isn't forced to depend on collision code.

### m2s2-math internals

- `Vector<T, const D>` (`src/vector/mod.rs`) is one const-generic type; `Vector2`/`Vector3`/`Vector4` are aliases. Core arithmetic (`Add`/`Sub`/`Mul<T>`/`Div<T>`/`Neg`) is implemented once, generically over `D`.
- Float-only per-dimension ops (`length`, `normalize`, `dot`, `cross`, etc., `src/vector/vector_ops.rs`) are split into separate `Vector2Ops`/`Vector3Ops`/`Vector4Ops` traits, each reimplementing the same handful of methods — **deliberate duplication**, not an oversight, because these traits expose `.x()/.y()/.z()/.w()` accessors that don't generalize over `D`.
- Dimension-generic Float ops that don't need those accessors (`distance`, `lerp`, `reflect`, `project_onto`, `reject_from`, `clamp_length`, `angle_between`) live in `src/vector/vector_algebra.rs` as a single generic `impl<T, const D> Vector<T, D>` block. When adding a new Float op, prefer this file/pattern over the per-dimension traits unless the op genuinely needs `.x()`-style accessors.
- `Matrix2x2/3x3/4x4` (`src/matrix/mod.rs`) come from one `define_matrix_struct!` macro, storing data **row-major** in a flat array (`data[row*cols+col]`). Every operation file (`base_ops.rs`, `identity_ops.rs`, `mat_mul_mat.rs`, `mat_mul_vec.rs`, `linear_ops.rs`) follows the same shape: a `macro_rules!` implementing the op generically, invoked once per matrix size. Follow this pattern for new matrix ops.
- `linear_ops.rs`: `trace`/`determinant` are generic over any numeric `T` (int or float) via cofactor expansion; `inverse` is Float-only via Gauss-Jordan elimination with partial pivoting — chosen deliberately over a hand-derived adjugate formula to avoid formula-transcription bugs (especially risky for 4x4).
- `Quaternion<T>` (`src/quaternion/mod.rs`) is a plain (non-const-generic) struct: Hamilton product, `slerp`/`lerp`, axis-angle/Euler conversions, `to_matrix3x3`/`to_matrix4x4`.
- `Transform2x2`/`Transform3x3`/`Transform4x4` traits (`src/matrix/transform_traits.rs`, impls in `transform_impl.rs`) provide rotation/translation/scale plus perspective/ortho/look-at in all four graphics-API conventions: `_rh_zo` (Vulkan/Metal/D3D12 RH), `_rh_no` (OpenGL), `_lh_zo` (D3D9/11/12 LH), `_lh_no` (rare, included for completeness).

### m2s2-geometry internals

- Every type is `T: num_traits::Float + Copy` — unlike `m2s2-math`'s `Vector`/`Matrix`, geometry types don't support integer element types.
- Cross-type intersection methods live on whichever type's file is the natural "owner"; the other type gets a thin delegating wrapper (e.g. `Aabb2::intersects_circle` has the real closest-point logic, `Circle::intersects_aabb` just calls it). Check both files before assuming a pairwise test doesn't exist yet.
- `sat_obb3` in `obb.rs` is the classical 15-axis OBB-OBB SAT test (Ericson, *Real-Time Collision Detection* §4.4.1). It is **deliberately not unified** with the 2D 4-axis SAT test in the same file, even though both are "SAT" — the 3D case needs `R`/`|R|` rotation-matrix bookkeeping for near-parallel-edge degeneracy that has no 2D analog. Don't try to generalize these into one function.
- Test convention (whole workspace): each file has its own `#[cfg(test)] mod tests` with a locally-defined `approx_eq` helper — there is no shared test-utility crate. Follow this pattern rather than introducing one.

## Allocation & concurrency design rationale (researched precedent)

This came up when the user asked whether a game math library needs a custom allocator and how to think about concurrency. Researched `nalgebra` for comparison (see sources below) rather than answer from assumption:

- **nalgebra's allocation model**: not a custom/arena allocator. It has an `Allocator` trait + `DefaultAllocator` that picks between two storage backends *at compile time*: `ArrayStorage` (plain stack array) when every dimension is known at compile time (e.g. `Vector3<f32>`, `Matrix4<f32>`), or `VecStorage` (a plain `std::Vec<T>`, not `Box<T>`) when a dimension is `Dyn` (runtime-sized, e.g. `DVector<f32>`). The heap path only exists to support nalgebra's general/scientific-computing use case (arbitrary N×M matrices, solvers) — a use case this workspace's `Vector<T, const D>` (D=2/3/4 only) and fixed `Matrix2x2/3x3/4x4` deliberately don't support. For the fixed-size types a game would actually use, nalgebra is stack-only too, same as us.
- **Concurrency**: nothing special needed at the math-primitives layer. Fixed-size value types with no interior mutability are automatically `Send`/`Sync` when `T` is (true for nalgebra's, glam's, and this workspace's types alike). The real concurrency problem (shared mutable world/physics/ECS state, parallel systems) lives at the *engine* layer, same boundary as the allocation split documented above — no math library solves it, an ECS scheduler does.
- **"Is allocation-free not game-ready?"** — the opposite. Bevy evaluated nalgebra and chose **glam** instead specifically because glam is SIMD-optimized, fixed-size-only, and has no dynamic-allocator abstraction to pay for. This workspace's design (const-generic fixed dims, `Copy` stack types, no `Dyn` support) is architecturally closer to glam's philosophy than nalgebra's — that's the right call for a game math library, not a gap.
- **Actual gap worth knowing about**: SIMD. glam's real performance edge over both nalgebra's scalar path and this workspace's current code is `std::simd`/platform-intrinsic use. Legitimate future optimization, but a separate, opt-in concern — not an allocation or concurrency problem, and not something to chase without a profiling reason.

Sources: [nalgebra `DefaultAllocator` docs](https://docs.rs/nalgebra/latest/nalgebra/base/default_allocator/struct.DefaultAllocator.html), [`Allocator` trait docs](https://docs.rs/nalgebra/latest/nalgebra/base/allocator/trait.Allocator.html), [`VecStorage` docs](https://docs.rs/nalgebra/latest/nalgebra/base/struct.VecStorage.html), [Bevy discussion #3231 on switching vector crates](https://github.com/bevyengine/bevy/discussions/3231), [glam/mathbench introduction](https://bitshifter.github.io/2019/07/10/introducing-glam-and-mathbench/).

## Versioning gotcha (context, not a live risk anymore)

`m2s2-math 0.1.0` is already published on crates.io and is **immutable** — that's why `m2s2-math` was manually bumped to `0.2.0` mid-session (for the Vector/Matrix API additions). This class of mistake (shipping API changes without bumping the version) is now caught automatically by release-plz's `semver_check` (see below), so it shouldn't require manual vigilance going forward — but the underlying fact (crates.io versions are immutable, `m2s2-geometry/Cargo.toml`'s `m2s2-math = { path = "..", version = "X" }` must reference a version that actually has what it needs) is still worth understanding.

## Release process (release-plz)

Versioning and publishing are automated via [release-plz](https://release-plz.dev), driven by [Conventional Commits](https://www.conventionalcommits.org/). Both jobs live in `.github/workflows/ci.yml` (not a separate file — kept in the same workflow deliberately so both stay gated on `needs: ci`, consistent with the "publish only runs if CI succeeds" principle established earlier; this deviates from release-plz's own quickstart docs, which default to a standalone workflow with no CI gate):

- **`release-plz-pr`** (`command: release-pr`): on every push to `main` that passes `ci`, opens or updates a "release PR" that bumps `Cargo.toml` versions and regenerates changelogs based on commits since the last release. Nothing is published by this step — it's the review gate.
- **`release-plz-release`** (`command: release`): also runs on every push to `main` that passes `ci`; it's idempotent — it only actually `cargo publish`s a crate whose local `Cargo.toml` version isn't yet live on crates.io. So publishing only happens *after* a release PR has been reviewed and merged (which is what changes the version in `Cargo.toml` on `main` in the first place).
- **`commitlint`** job enforces Conventional Commits format (`<type>(<scope>)!: <description>`) on every push/PR so release-plz's commit-history analysis doesn't silently misfire on an unparseable message. Mirrored locally in `.githooks/commit-msg` for fast feedback before it ever reaches CI (enable via the same `core.hooksPath` step as the pre-commit hook).
- `release-plz.toml` (repo root) turns on `semver_check` (blocks a release if the new API isn't backward-compatible with the version being assigned — catches the exact mistake described above automatically), `changelog_update`, and `git_release_enable`. `m2s2-math` keeps writing to the existing root `CHANGELOG.md` (its default path); `m2s2-geometry` gets its own `m2s2-geometry/CHANGELOG.md` rather than sharing the root file — sharing was possible via `changelog_path`/`changelog_include` but wasn't set up since it couldn't be verified without a live run.
- Requires the `CARGO_REGISTRY_TOKEN` secret (already configured, was used by the old hand-rolled publish job too) and `contents: write`/`pull-requests: write|read` permissions on the relevant jobs.
- **Auth for the two release-plz jobs is a minted GitHub App installation token, not the default `GITHUB_TOKEN`.** The default token cannot open PRs (repo/org-level restriction, independent of the job's `permissions:` block — first hit as a live `403 Forbidden: GitHub Actions is not permitted to create or approve pull requests` on `release-plz-pr`) and, separately, commits/PRs made with the default token can't trigger other workflow runs (so the release PR wouldn't get `ci` checks). Both jobs' `actions/checkout` use `persist-credentials: false`, followed by `actions/create-github-app-token@v2` (`app-id: secrets.APP_ID`, `private-key: secrets.APP_PRIVATE_KEY`) to mint a token, which is what actually gets passed as `GITHUB_TOKEN` to the release-plz action steps. **The App itself must be installed on this repo** (M2S2-Engineering-Group org) with Contents (R/W) and Pull requests (R/W) permissions, and `APP_ID`/`APP_PRIVATE_KEY` added as repo secrets — this is a manual GitHub-side step, not something in the repo files, and wasn't confirmed done as of this writing.

There used to be a separate `publish.yml` with a hand-rolled two-step `cargo publish -p ...` sequence and its own duplicate `ci` job; both were replaced by the above. Don't reintroduce manual version bumping or a second workflow file for this.

## Current status / recent history

Check `git log --oneline -10` and `git status` first when picking this up — this section is a narrative summary and will go stale; git is the source of truth. As of the last update here (2026-07-02), the vector/matrix additions, the `m2s2-geometry` crate split, and the CI merge were already committed (`33820b8 adding v1 of geometry lib`, `1d2cbb8 fix publish yaml`, `26557e4 combine publish in successful ci flow`). The release-plz/commitlint additions described below (point 9) were **not yet committed** as of this writing — check `git status` for `.github/workflows/ci.yml`, `release-plz.toml`, `.githooks/commit-msg`.

What happened, in order:
1. Verified the pre-existing `m2s2-math` library (vectors, 2x2/3x3/4x4 matrices, quaternions, multi-convention transforms) was solid — 83 passing tests, clean build.
2. Added Vector utilities (`distance`, `lerp`, `reflect`, `project_onto`, `reject_from`, `clamp_length`, `angle_between`, 2D `cross`) and Matrix `trace`/`determinant`/`inverse`.
3. Built out a full geometry/collision layer at the user's request (AABB, sphere/circle, ray, plane, triangle, OBB with full SAT collision) — originally under `m2s2-math/src/geometry/`, then split into a new sibling crate `m2s2-geometry` mid-session once it became clear the renderer consuming this library shouldn't need to depend on collision code. `src/geometry/` was deleted from `m2s2-math` after the split.
4. Fixed real bugs the workspace conversion introduced: CI wasn't testing `m2s2-geometry` at all (missing `--workspace`), and `cargo publish` couldn't work unqualified against a two-member workspace.
5. Added the `.githooks/pre-commit` hook and README "Development"/"Allocation strategy" sections.
6. Caught and fixed a real problem before publishing: `m2s2-math 0.1.0` was already live on crates.io, so the new methods added in this session couldn't ship under that version number — bumped to `0.2.0`.
7. Researched nalgebra's allocation model and Bevy's glam-over-nalgebra rationale (see "Allocation & concurrency design rationale" above) to validate this workspace's allocation-free, fixed-size design.
8. Merged `publish.yml` into `ci.yml` (single `publish` job, `needs: ci`, `if` restricted to push-to-main) so publish is gated on the real CI run instead of a duplicated private copy of the same checks — `publish.yml` no longer exists. (Committed by the user directly, not this assistant.)
9. Replaced the hand-rolled `publish` job with release-plz (`release-plz-pr` + `release-plz-release` jobs, `release-plz.toml`) for Conventional-Commits-driven semver automation, plus a `commitlint` CI job and matching `.githooks/commit-msg` hook to enforce the commit format it depends on. See "Release process (release-plz)" above for the full flow.
10. First live run of `release-plz-pr` failed: `403 Forbidden — GitHub Actions is not permitted to create or approve pull requests` (a repo/org-level setting, not a workflow bug). User already runs a GitHub App for this exact purpose on other repos, so switched both release-plz jobs to a minted App installation token (`actions/create-github-app-token@v2`, `secrets.APP_ID`/`secrets.APP_PRIVATE_KEY`) instead of enabling the blanket "Allow Actions to create/approve PRs" repo setting. **Not yet verified live** — the App still needs to be installed on this repo with Contents+Pull-requests R/W, and the two secrets added, before the next push to `main`.

**Deliberately not built** (flagged during planning, not silently skipped): `Obb2`/`Obb3` vs `Sphere`/`Circle` intersection; `Triangle3::normal()`/`centroid()`/`area()` helpers; `cargo-llvm-cov`/`cargo-tarpaulin` coverage tooling. Pick any of these up if asked, otherwise leave them alone.

**Immediate next step, if resuming**: on GitHub's side (not in this repo's files) — confirm the GitHub App is installed on `M2S2-Engineering-Group/m2s2-rust-math` with Contents (R/W) and Pull requests (R/W), and that `APP_ID`/`APP_PRIVATE_KEY` are set as repo secrets. Then commit the release-plz/commitlint/App-token work (points 9-10) if still uncommitted, `git config core.hooksPath .githooks` if not already set, and push to `main` — that push is what actually exercises `commitlint`/`release-plz-pr`/`release-plz-release` for the first time. Watch that Actions run; if `release-plz-pr` still 403s, the App isn't installed/permissioned correctly yet.
