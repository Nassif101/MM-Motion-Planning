#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
unity_project="${repo_root}/motion-planning-sim"
install_editor=false
build_container=false

for arg in "$@"; do
  case "${arg}" in
    --install-editor) install_editor=true ;;
    --build-container) build_container=true ;;
    *) echo "Unknown option: ${arg}" >&2; exit 2 ;;
  esac
done

command -v git >/dev/null || { echo "git is required" >&2; exit 1; }
command -v docker >/dev/null || { echo "Docker Desktop is required" >&2; exit 1; }
command -v unity >/dev/null || { echo "Unity CLI is required" >&2; exit 1; }

unity_paths="$(type -a -p unity | awk '!seen[$0]++')"
if [ "$(printf '%s\n' "${unity_paths}" | wc -l | tr -d ' ')" -gt 1 ]; then
  echo "Warning: multiple Unity CLI binaries are on PATH; using $(command -v unity):" >&2
  printf '%s\n' "${unity_paths}" | sed 's/^/  /' >&2
fi
echo "Unity CLI: $(command -v unity) ($(unity --version))"

git -C "${repo_root}" submodule update --init --recursive
docker compose -f "${repo_root}/.devcontainer/docker-compose.yml" config --quiet

host_arch="$(uname -m)"
case "${host_arch}" in
  arm64|aarch64) unity_arch=arm64 ;;
  x86_64|amd64) unity_arch=x86_64 ;;
  *) echo "Unsupported host architecture: ${host_arch}" >&2; exit 1 ;;
esac

if ${install_editor}; then
  unity install 6000.5.2f1 --architecture "${unity_arch}" --yes --accept-eula
  unity pipeline install --project-path "${unity_project}"
fi

if ${build_container}; then
  docker compose -f "${repo_root}/.devcontainer/docker-compose.yml" build
fi

unity projects info "${unity_project}" --format json
echo "Host setup checks passed for ${host_arch}."
