# WebARKitLib tests — notes for coding agents

Same scope as `../AGENTS.md`: only when working in a WebARKitLib checkout, not through a
superproject's submodule. Commands run from the WebARKitLib root.

- GoogleTest, native Linux only (on Windows use WSL). CI runs exactly what `.github/workflows/test.yml` lists.
- Build: `cmake -S tests -B tests/build -DEMSCRIPTEN_COMP=0 && cmake --build tests/build -j`, then run the five binaries from `tests/build`.
- The build directory must sit directly inside `tests/`: `webarkit_test` opens `../pinball.jpg`. `webarkit_nft_core_test` and `webarkit_param_test` copy what they need from `tests/data` next to their binaries.
- `webarkit_nft_core_test` takes about 3 minutes. While iterating, use `--gtest_filter='Suite.*'`; run everything before committing.
- This CMakeLists forces `WEBARKIT_NFT_THREADS=ON`; threaded-only tests sit under `#ifdef WEBARKIT_NFT_THREADS`. To check a build without threads, build `WebARKit/` on its own with `-DWEBARKIT_BUILD_OPTICAL=OFF -DWEBARKIT_BUILD_NFT=ON -DWEBARKIT_NFT_THREADS=OFF`.
- To assert what the code logs, use `webarkitLogSetLogger()` with `gCapturedLog`, and wrap level changes in `LogLevelGuard`.
- A configurable fake `NFTDetector` (`FakeExternalDetector`) already exists in `webarkit_nft_core_test.cc`: reuse it rather than writing another.
- If you change a config constant or the version, update the matching assertions in `webarkit_test.cc`.
- CI is pinned to `ubuntu-24.04` because GCC 15 rejects the vendored `ioapi.c` (#118).
