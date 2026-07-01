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

## Versioning gotcha (read before publishing anything)

`m2s2-math 0.1.0` is already published on crates.io and is **immutable**. `m2s2-math` is currently at `0.2.0` in `Cargo.toml` (bumped for the API surface added in the session that split off `m2s2-geometry` — Vector distance/lerp/reflect/etc., Matrix trace/determinant/inverse) but **0.2.0 has not been published yet**. Before publishing again: any further public-API change to `m2s2-math` needs another version bump — you cannot republish an existing version with different content. `m2s2-geometry/Cargo.toml`'s `m2s2-math = { path = "..", version = "X" }` must be kept in sync with whatever version `m2s2-math` actually is.

`.github/workflows/publish.yml` publishes `m2s2-math` first, then `m2s2-geometry` with a retry loop (crates.io index-propagation lag) — this ordering is required; don't reorder or parallelize it.

## Current status / recent history

As of the last session (2026-07-01), the working tree has substantial **uncommitted** changes — nothing from the work described below has been committed to git yet. If you're picking this up on a different machine or after a disconnect, check `git status`/`git diff` first; if the changes aren't there, they only exist wherever the previous session's working tree was.

What happened, in order:
1. Verified the pre-existing `m2s2-math` library (vectors, 2x2/3x3/4x4 matrices, quaternions, multi-convention transforms) was solid — 83 passing tests, clean build.
2. Added Vector utilities (`distance`, `lerp`, `reflect`, `project_onto`, `reject_from`, `clamp_length`, `angle_between`, 2D `cross`) and Matrix `trace`/`determinant`/`inverse`.
3. Built out a full geometry/collision layer at the user's request (AABB, sphere/circle, ray, plane, triangle, OBB with full SAT collision) — originally under `m2s2-math/src/geometry/`, then split into a new sibling crate `m2s2-geometry` mid-session once it became clear the renderer consuming this library shouldn't need to depend on collision code. `src/geometry/` was deleted from `m2s2-math` after the split.
4. Fixed real bugs the workspace conversion introduced: CI wasn't testing `m2s2-geometry` at all (missing `--workspace`), and `cargo publish` couldn't work unqualified against a two-member workspace.
5. Added the `.githooks/pre-commit` hook and README "Development"/"Allocation strategy" sections.
6. Caught and fixed a real problem before publishing: `m2s2-math 0.1.0` was already live on crates.io, so the new methods added in this session couldn't ship under that version number — bumped to `0.2.0`.

**Deliberately not built** (flagged during planning, not silently skipped): `Obb2`/`Obb3` vs `Sphere`/`Circle` intersection; `Triangle3::normal()`/`centroid()`/`area()` helpers; `cargo-llvm-cov`/`cargo-tarpaulin` coverage tooling. Pick any of these up if asked, otherwise leave them alone.

**Immediate next step, if resuming**: get the current working tree committed (and pushed, if there's a remote) — see the uncommitted-changes note above. After that, an actual `crates.io` publish of `m2s2-math 0.2.0` + `m2s2-geometry 0.1.0` is pending (the CI `publish` job runs automatically on push to `main`).
