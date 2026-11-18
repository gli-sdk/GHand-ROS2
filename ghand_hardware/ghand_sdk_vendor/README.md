# ghand_sdk_vendor

This package installs a copy of the SDK-generated Linux x86_64 distribution,
including `libghand.so`, public headers, CMake exports, product JSON and license
notices. No SDK source checkout is needed for ROS builds or runtime.

The hardware adapter finds this package and links only its installed SDK copy.
`SDK_MANIFEST.json` records the version, source revision and artifact hashes.
The original SDK license and third-party notices are under `sdk/`.

For an SDK update, generate an SDK installation with the required architecture,
replace `sdk/` with that complete installation (preserving library symlinks),
update the manifest, then rebuild this package and the hardware adapter.
Do not copy individual headers from a different SDK version.
