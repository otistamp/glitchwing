#!/usr/bin/env bash
# Build/run the Android app with the right toolchain env.
#   scripts/build-android.sh [build|run]   (default: build)
# Passes the 16 KB page-size linker flag via RUSTFLAGS (cargo-apk overrides
# .cargo/config.toml, so it must be set in the environment here).
set -euo pipefail

export ANDROID_HOME="${ANDROID_HOME:-/opt/homebrew/share/android-commandlinetools}"
export ANDROID_NDK_ROOT="${ANDROID_NDK_ROOT:-$ANDROID_HOME/ndk/27.2.12479018}"
export ANDROID_NDK_HOME="$ANDROID_NDK_ROOT"
export PATH="$(brew --prefix rustup)/bin:$HOME/.cargo/bin:$ANDROID_HOME/platform-tools:$PATH"
export RUSTFLAGS="${RUSTFLAGS:-} -C link-arg=-Wl,-z,max-page-size=16384"

cmd="${1:-build}"
shift || true
exec cargo apk "$cmd" -p skyraptor-android --target aarch64-linux-android "$@"
