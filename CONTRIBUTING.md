# Contributing to WebARKitLib

Thanks for helping improve WebARKit! This is the C/C++ source of the tracker; most
changes are consumed by the [webarkit-testing](https://github.com/webarkit/webarkit-testing)
superproject as a submodule.

## Workflow

- **Branch from `dev`** and open your PR **against `dev`** (not `main`).
- Reference the related issue in the PR description.
- **Sign your commits** (`git commit -S …`).

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

A change that affects the WASM build is consumed by webarkit-testing via the
submodule. Coordinate both repos:

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
