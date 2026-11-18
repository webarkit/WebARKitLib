# WebARKitLib — notes for coding agents

**Scope.** These rules apply when you work in a checkout of webarkit/WebARKitLib itself.
WebARKitLib is also a git submodule of other projects (WebAR-Canvas, jsartoolkitNFT, …).
If you are reading this through one of them, do not edit files here: follow that
project's own rules. A change to WebARKitLib goes upstream, in a WebARKitLib clone with
its own PR; the superproject then bumps the submodule pointer. Paths below are relative
to the WebARKitLib root.

CONTRIBUTING.md has the full rules. The essentials:

## Workflow
- Branch from `dev` and open PRs against `dev`, never `master`. Branch names: `feat/…`, `fix/…`, `chore/…`, `docs/…`.
- Commits follow Conventional Commits (`feat(nft): …`) and are GPG-signed through git's own config. Never pass `--author`, `-c user.*` or `--no-gpg-sign`.
- In PR bodies, reference issues as `For #N`, without closing keywords: the maintainer closes them.
- To pick up fixes from `dev`, merge `dev` into the feature branch. No rebase or force-push on pushed branches.
- Do not modify the vendored ARToolKit5 sources (`lib/SRC/**`, `include/AR/**`). Work around their problems in CMake.
- Public headers are consumed by the superprojects: keep changes backward compatible, and when adding a capability a consumer will test for, define a feature macro (e.g. `WEBARKIT_NFT_HAS_EXTERNAL_DETECTOR`).
- Tests: see `tests/AGENTS.md`.

## Code
- `ARParam` is an anonymous typedef, so it can't be forward-declared: include `<AR/ar.h>`, never `<AR/param.h>` first (circular include).
- `ARToolKitNFTCore.cpp` and the KPM detectors build with `-fno-rtti -fno-exceptions`.
- The `NFTDetector` contract (lifetime, `waitIdle()`, re-entrancy of `markersAdded()`) is documented in `NFTDetector.h` and `NFTTrackingConfig.h`: update it whenever the behaviour changes.
