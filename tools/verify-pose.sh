#!/usr/bin/env bash
# Off-device verification of the pose/mount path. Compiles the SHIPPING HeadPose + MountCal
# (no Android imports in either) together with tools/PoseHarness.java and runs the checks.
#
#   ./tools/verify-pose.sh
#
# Requires JDK 17 (Homebrew openjdk@17) — the same JDK AGP needs for the APK build.
set -euo pipefail
cd "$(dirname "$0")/.."

JAVA_HOME="${JAVA_HOME:-$(brew --prefix openjdk@17 2>/dev/null || echo /opt/homebrew/opt/openjdk@17)}"
export JAVA_HOME
export PATH="$JAVA_HOME/bin:$PATH"

SRC=app/src/main/java/com/xman/xrealbeam
OUT=build/pose-harness
rm -rf "$OUT"
mkdir -p "$OUT"

javac -d "$OUT" \
  "$SRC/HeadPose.java" \
  "$SRC/MountCal.java" \
  "$SRC/Geometry.java" \
  tools/PoseHarness.java

java -cp "$OUT" com.xman.xrealbeam.PoseHarness
