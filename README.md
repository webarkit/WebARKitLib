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

`WebARKit/CMakeLists.txt` can **already build WebARKitLib as a static library**
today — for both **WASM** (emscripten) and **native Linux**, selected via the
`EMSCRIPTEN_COMP` flag. It depends on a prebuilt **OpenCV** from
[webarkit/opencv-em](https://github.com/webarkit/opencv-em) (the emscripten
`opencv-js-…-emcc` build for WASM, or the native `opencv-…` build for Linux),
fetched automatically via CMake `FetchContent`. The "planned" part above is
webarkit-testing switching its WASM build over to this CMake config (away from
`tools/makem.js`). The same config also drives the unit tests (see below).

### NFT helpers (`WebARKit/WebARKitTrackers/WebARKitNFT`)

The web-adapted NFT helpers formerly in jsartoolkitNFT live here (#75), together
with the NFT tracking core that jsartoolkitNFT's bindings are thin adapters over:

| Header (`<WebARKitTrackers/WebARKitNFT/...>`) | Provides |
|---|---|
| `ARToolKitNFTCore.h` | `ARToolKitNFTCore` — the NFT tracking core: camera, markers, frames, KPM detection policy, AR2 tracking (C++, no Embind) |
| `NFTTrackingConfig.h` | `NFTTrackingConfig` and its presets `singleThreadPreset()` / `threadedPreset()` — the core's variant settings |
| `NFTDetector.h` | `NFTDetector` — interface of a KPM detection pass (`start()` / `collect()`) |
| `SyncKpmDetector.h` | `SyncKpmDetector` — KPM detection on the calling thread |
| `ThreadedKpmDetector.h` | `ThreadedKpmDetector` — KPM detection on a worker thread (only with `WEBARKIT_NFT_THREADS`) |
| `KpmRefDataSetCopy.h` | `kpmCopyRefDataSet()` — deep copy of a KPM reference data set |
| `trackingMod.h` | `ar2TrackingMod()`, `ar2CreateHandleMod()`, `ar2DeleteHandleMod()` — single-threaded AR2 tracking |
| `markerDecompress.h` | `decompressMarkers()` — unpacks a `.zft` into `.iset`/`.fset`/`.fset3` (returns `-1` on error) |
| `trackingSub.h` | `trackingInit*()` — KPM detection on a worker thread (pthreads) |
| `NFTMarkerState.h` | per-marker tracking state (C++ only) |

They are built by the `WebARKitNFT` static library, which also compiles the
ARToolKit5 sources it needs (AR, ARICP, AR2, KPM, ARUtil) and does **not** need
OpenCV. CMake options in `WebARKit/CMakeLists.txt`:

- `WEBARKIT_BUILD_OPTICAL` (ON) — the OpenCV `WebARKitLib` target
- `WEBARKIT_BUILD_NFT` (OFF) — the `WebARKitNFT` target (needs libjpeg and zlib)
- `WEBARKIT_NFT_THREADS` (OFF) — adds `trackingSub` and `ThreadedKpmDetector`
  (pthreads), and defines `WEBARKIT_NFT_THREADS` as a **PUBLIC** compile definition,
  so everything that links `WebARKitNFT` sees it (the core then supports the
  threaded detector)

`WebARKit/WebARKitLog.cpp` (the native logger the core uses) is compiled into both
the `WebARKitLib` and the `WebARKitNFT` libraries. That is fine for static archives,
where the linker takes the first copy; linking both with `--whole-archive`, or as
shared libraries, would define its symbols twice.

#### Using `ARToolKitNFTCore` natively

- Call `loadCamera()`, `setup()`, `setupAR2()`, then `addNFTMarkers()`; then, per
  frame, `setVideoFrame()` and `detectNFTMarker()`, and read `markerState(i)`.
- `setup()` returns the controller id even when the camera could not be applied (as
  the JS bindings do): check `cameraParamLT() != nullptr` afterwards.
- `setCamera()` frees the KPM handle and the detector with the old camera; call
  `setupAR2()` after it. After a second `setupAR2()` the markers already loaded are
  not detected until the next `addNFTMarkers()`, which hands the new KPM handle every
  loaded marker; `addNFTMarkers({})` does that without adding markers and, like a
  failure, returns an empty vector.
- `teardown()` frees the handles, the markers and the frame buffers, but
  `getNFTData()` keeps returning the old markers' data until markers are loaded again.
- Threads: drive each core from one thread (the threaded detector's worker is the
  only other one). `loadCamera()` and `setup()` use a process-wide camera registry
  and id counters that are not synchronised, so cores on different threads must not
  call them concurrently.

```bash
emcmake cmake -S WebARKit -B build-nft -DWEBARKIT_BUILD_OPTICAL=OFF -DWEBARKIT_BUILD_NFT=ON -DWEBARKIT_NFT_THREADS=ON
cmake --build build-nft
```

## Tests

C++ unit tests (GoogleTest) live in [`tests/`](tests/) (`webarkit_test.cc`, `webarkit_nft_test.cc`,
`webarkit_nft_limit_test.cc`, `webarkit_nft_core_test.cc` — the NFT tracking core, with the
markers and frames in `tests/data` — `CMakeLists.txt`, `pinball.jpg`) and run in CI via
[`.github/workflows/test.yml`](https://github.com/webarkit/WebARKitLib/actions/workflows/test.yml).
Build them standalone with CMake:

```bash
cmake -S tests -B tests/build && cmake --build tests/build && ctest --test-dir tests/build
```

## Contributing

Please read [`./CONTRIBUTING.md`](./CONTRIBUTING.md) for the full guidelines. In short:

- Branch from `dev`; PR back to `dev`; sign your commits and reference the issue.
- **Commit messages must follow [Conventional Commits](https://www.conventionalcommits.org/)**
  (e.g. `feat:`, `fix:`, `docs:`, `refactor:`, `test:`, `chore:`).
- Library changes are usually paired with a build/bump in webarkit-testing — see that
  repo's "cross-repo PR pair" flow.
- Don't modify the vendored `lib/SRC/**` / `include/AR/**` (old ARToolKit5) as part of
  WebAR work.

## License

LGPL-3.0 (see `LICENSE.txt`).
