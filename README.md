![github releases](https://flat.badgen.net/github/release/webarkit/WebARKitLib)
![github stars](https://flat.badgen.net/github/stars/webarkit/WebARKitLib)
![github forks](https://flat.badgen.net/github/forks/webarkit/WebARKitLib)[![Test](https://github.com/webarkit/WebARKitLib/actions/workflows/test.yml/badge.svg)](https://github.com/webarkit/WebARKitLib/actions/workflows/test.yml)

# WebARKitLib

The C/C++ source of **WebARKit** — a browser-side Augmented Reality tracker. The
active code (`WebARKit/`) is an emscripten-friendly port of the **OCVT** planar
image-tracking design from [ArtoolkitX](https://github.com/artoolkitx/artoolkitx),
built on OpenCV. It is normally compiled to WebAssembly and consumed from the
[webarkit-testing](https://github.com/webarkit/webarkit-testing) superproject.

> **Two lineages live here, keep them straight:**
> - **`WebARKit/`** — the WebAR OCVT planar tracker (the design reference is *ArtoolkitX*). This is the code under active development.
> - **`lib/SRC/**`, `include/AR/**`** — vendored **old ARToolKit5** utility sources/headers, kept only because [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT) needs them (and [ARnft](https://github.com/webarkit/ARnft) in turn consumes jsartoolkitNFT as an npm module). Treated as untouched upstream. (Updated to Eigen 3.4.0 / emsdk 3.1.26.)

## Code layout (`WebARKit/`)

| Component | Role |
|---|---|
| `WebARKitManager` | top-level façade — owns the tracker, drives init/process, exposes the output (homography, pose, GL-view, projection) |
| `WebARKitTrackers/WebARKitOpticalTracking/WebARKitTracker` | the OCVT tracker core: feature detect/match → homography → optical-flow + template-match tracking |
| `…/TrackingPointSelector` + `…/TrackedPoint` | bin-based selection of template tracking points (ArtoolkitX-derived) |
| `…/WebARKitHomographyInfo` + `WebARKitUtils.h` | RANSAC homography estimation/validation (`getHomographyInliers`) |
| `…/WebARKitConfig` | tuning constants (feature counts, match ratios, pyramid/template sizes, version) |
| `…/TrackerVisualization` | optional debug-overlay scaffolding (inert until wired) |
| `WebARKitCamera` | camera intrinsics from a diagonal-FOV estimate |
| `WebARKitGL` | CV→GL matrix conversion (`arglCameraViewRHf`) + GL projection |
| `WebARKitPattern` | the reference marker (`WebARKitPattern`) and per-frame pose state (`WebARKitPatternTrackingInfo`) |
| `WebARKitLog` | logging |

## Pose pipeline (one glance)

```
solvePnP (cameraPoseFromPoints)            // OpenCV-convention camera pose
  -> getTrackablePose / updateTrackable    // CV->GL handedness fix, D*R*D  (see #42)
  -> arglCameraViewRHf                      // right-handed GL modelview
  -> matrixGL_RH                            // what the examples attach content to
```

Pose getters: **`getPoseMatrixCV()`** (raw 4×4 OpenCV pose) and **`getPoseMatrixGL()`**
(the GL/right-handed pose); projection via `getCameraProjectionMatrix()`. The full
derivation is in
[`docs/design-projection-and-pose-artoolkitx-alignment.md`](https://github.com/webarkit/webarkit-testing/blob/dev/docs/design-projection-and-pose-artoolkitx-alignment.md)
(webarkit-testing).

## Building

This repo isn't built as a single unit — each consumer compiles the **subset it
needs**:

- **`WebARKit/` (the OCVT tracker)** is built by the
  [webarkit-testing](https://github.com/webarkit/webarkit-testing) superproject, which
  compiles it to WebAssembly with emscripten (emsdk **3.1.26**) inside Docker and wires
  it to JS through `emscripten/WebARKitJS.{cpp,h}` + `bindings.cpp` (which live in
  webarkit-testing). webarkit-testing currently drives that compile with the Node script
  `tools/makem.js` (`npm run build-docker` → `build-es6`); migrating to the CMake config
  (`WebARKit/CMakeLists.txt`) is planned.
- **The vendored `lib/SRC/**` + `include/AR/**` (old ARToolKit5)** is built by
  [jsartoolkitNFT](https://github.com/webarkit/jsartoolkitNFT) as part of its build —
  not by webarkit-testing. ([ARnft](https://github.com/webarkit/ARnft) consumes
  jsartoolkitNFT as an npm module and doesn't build the emscripten code itself.)

A standalone CMake config also exists for the unit tests (see below).

## Tests

C++ unit tests (GoogleTest) live in [`tests/`](tests/) (`webarkit_test.cc`,
`CMakeLists.txt`, `pinball.jpg`) and run in CI via
[`.github/workflows/test.yml`](https://github.com/webarkit/WebARKitLib/actions/workflows/test.yml).
Build them standalone with CMake:

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
```

## Contributing

- Branch from `dev`; PR back to `dev`; sign your commits and reference the issue.
- Library changes are usually paired with a build/bump in webarkit-testing — see that
  repo's "cross-repo PR pair" flow.
- Don't modify the vendored `lib/SRC/**` / `include/AR/**` (old ARToolKit5) as part of
  WebAR work.

## License

LGPL-3.0 (see `LICENSE.txt`).
