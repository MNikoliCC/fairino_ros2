#!/usr/bin/env bash
set -Eeuo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
: "${RUNNER_TEMP:?Run this through GitHub Actions}"
: "${GITHUB_RUN_ID:?}"
: "${GITHUB_RUN_ATTEMPT:?}"
export CI_ARTIFACTS="${RUNNER_TEMP}/fairino-${GITHUB_RUN_ID}-${GITHUB_RUN_ATTEMPT}/artifacts"
mkdir -p "$CI_ARTIFACTS"
exec > >(tee "$CI_ARTIFACTS/runner.log") 2>&1
mode="${CI_MODE:-diagnostics}"
case "$mode" in diagnostics|mock|simmachine) ;; *) exit 2 ;; esac
export CI_MODE="$mode" CI_UID="$(id -u)" CI_GID="$(id -g)"
export CI_ROBOPLAN_VERSION="$(cat roboplan-ros/ROBOPLAN_VERSION)"
project="fairino-ci-${GITHUB_RUN_ID}-${GITHUB_RUN_ATTEMPT}"
compose() { docker compose -p "$project" -f ci/compose.yaml "$@"; }
cleanup() {
  result=$?
  trap - EXIT
  set +e
  if [[ "$mode" != diagnostics ]]; then
    compose --profile simmachine logs --no-color > "$CI_ARTIFACTS/containers.log" 2>&1
    compose --profile simmachine down --timeout 15 > "$CI_ARTIFACTS/cleanup.log" 2>&1
    cleanup_result=$?
    if ((result == 0 && cleanup_result != 0)); then result=$cleanup_result; fi
  fi
  printf 'Exit status: %s\n' "$result" > "$CI_ARTIFACTS/result.txt"
  if [[ -n "${GITHUB_STEP_SUMMARY:-}" ]]; then
    printf 'Mode: %s\n\nCommit: %s\n\nExit status: %s\n' \
      "$mode" "${GITHUB_SHA:-unknown}" "$result" >> "$GITHUB_STEP_SUMMARY"
  fi
  exit "$result"
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
printf 'Commit: %s\nMode: %s\n' "${GITHUB_SHA:-unknown}" "$mode"
uname -srmo
id
df -h "$RUNNER_TEMP"
docker version
docker compose version
docker info --format 'Architecture={{.Architecture}} CPUs={{.NCPU}} Memory={{.MemTotal}}'
[[ "$CI_UID" != 0 ]] || { echo 'Use a non-root runner account.'; exit 1; }
[[ "$mode" != diagnostics ]] || exit 0
compose config --quiet
if [[ "$mode" == simmachine ]]; then
  : "${SIMMACHINE_IMAGE:?Set the FAIRINO_CI_SIMMACHINE_IMAGE repository variable}"
  docker image inspect "$SIMMACHINE_IMAGE" --format '{{.Id}}' > "$CI_ARTIFACTS/simmachine-image.txt"
fi
compose build ros2
if [[ "$mode" == simmachine ]]; then
  compose --profile simmachine up -d simmachine
fi
# The existing entrypoint builds this checkout before running smoke.sh.
# A fresh container filesystem avoids stale build/install outputs from other PRs.
compose run --rm --no-deps ros2
