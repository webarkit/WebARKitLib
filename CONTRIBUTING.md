# Contributing to WebARKitLib

Thanks for helping improve WebARKit! This is the C/C++ source of the tracker; most
changes are consumed by the [webarkit-testing](https://github.com/webarkit/webarkit-testing)
superproject as a submodule.

## Workflow

- **Branch from `dev`** and open your PR **against `dev`** (not `master`).
- Reference the related issue in the PR description.
- **Sign your commits** (`git commit -S …`).

### Merging a branch-sync PR

Ordinary PRs can be squashed. A PR that **syncs one long-lived branch into another**
— typically `master` back into `dev` after a release — must be merged with
**"Create a merge commit"**, never squash or rebase.

Squashing collapses the incoming commits into a single new commit, and rebasing
replays them under new SHAs. Either way `dev` ends up *containing* the code without
git recognising the two branches as related, so the next sync offers the same commits
again and usually conflicts.

Only a merge commit makes the incoming commits genuine ancestors, which is what makes
the following sync a no-op.

This is not hypothetical: `dev` drifted 14 commits behind `master` and had to be
repaired in #68, and consumers could not build against `dev` in the meantime.

## Commit messages — Conventional Commits

All commit messages **must** follow
[Conventional Commits](https://www.conventionalcommits.org/):

```
<type>(<optional scope>): <short description>
```

Common types: `feat`, `fix`, `docs`, `refactor`, `test`, `chore`, `perf`. Examples:

```
feat(tracker): detect features on a downsampled frame
fix(#55): populate pose3d so getPoseMatrixCV() isn't zero
docs: document the pose pipeline
refactor(#50): remove dead computePose/invertPose
```

## What you may change

- Active WebAR work lives under **`WebARKit/`** (the OCVT planar tracker).
- **Do not** modify the vendored **old ARToolKit5** sources under `lib/SRC/**` or
  `include/AR/**` — those are upstream utilities used by
  [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT) /
  [ARnft](https://github.com/webarkit/ARnft) and are kept untouched.

## Cross-repo changes (the PR pair)

> **Applies to `WebARKit/` (this folder) ↔ the `webarkit-testing` repo.** This flow is
> for changes to the **`WebARKit/`** OCVT tracker, which the **webarkit-testing**
> superproject compiles to WASM. Changes to the vendored `lib/SRC/**` / `include/AR/**`
> (old ARToolKit5) do **not** use this flow — they reach the web through
> [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT), a different consumer.

A change to `WebARKit/` that affects the WASM build is consumed by webarkit-testing via
the submodule. Coordinate both repos:

1. Branch from `dev` in **both** WebARKitLib and webarkit-testing.
2. Make the C++ change here; in webarkit-testing bump the submodule pointer and
   rebuild (`npm run build-docker` → `build-es6`).
3. Open a **PR pair** (WebARKitLib → `dev`, webarkit-testing → `dev`); cross-link them.
4. After the WebARKitLib PR merges, re-point the submodule to the merged `dev` tip.

## Building & tests

- The library is normally built **from webarkit-testing** (WASM) — see its README.
  `WebARKit/CMakeLists.txt` can also build it as a static lib for **WASM** or **native
  Linux** (selected via `EMSCRIPTEN_COMP`), pulling OpenCV from
  [webarkit/opencv-em](https://github.com/webarkit/opencv-em).
- **Unit tests** (GoogleTest) live in `tests/` and run in CI on every PR:

  ```bash
  cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
  ```

  If you change a config constant or the version, update the matching assertions in
  `tests/webarkit_test.cc`.

## License

By contributing, you agree that your contributions are licensed under **LGPL-3.0**.
