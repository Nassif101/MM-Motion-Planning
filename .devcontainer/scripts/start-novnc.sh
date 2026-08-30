#!/usr/bin/env bash
set -euo pipefail

display="${NOVNC_DISPLAY:-:99}"
if [[ "${display}" != :* ]]; then
  display=":${display}"
fi

screen="${NOVNC_SCREEN:-1920x1080x24}"
novnc_port="${NOVNC_PORT:-6080}"
vnc_port="${NOVNC_VNC_PORT:-5901}"
vnc_host="127.0.0.1"
window_manager="${NOVNC_WM:-openbox}"
user_name="${USER:-$(id -un)}"
runtime_dir="${XDG_RUNTIME_DIR:-}"
if [[ -z "${runtime_dir}" || ! -d "${runtime_dir}" || ! -w "${runtime_dir}" ]]; then
  runtime_dir="/tmp/novnc-${user_name}"
fi
log_dir="${runtime_dir}/logs"

mkdir -p "${runtime_dir}" "${log_dir}"
chmod 700 "${runtime_dir}"

xvfb_pid_file="${runtime_dir}/xvfb.pid"
wm_pid_file="${runtime_dir}/wm.pid"
x11vnc_pid_file="${runtime_dir}/x11vnc.pid"
novnc_pid_file="${runtime_dir}/novnc.pid"
x_socket_dir="/tmp/.X11-unix"
x_lock_file="/tmp/.X${display#:}-lock"

if command -v novnc_proxy >/dev/null 2>&1; then
  novnc_command=(novnc_proxy)
elif [[ -x /usr/share/novnc/utils/novnc_proxy ]]; then
  novnc_command=(/usr/share/novnc/utils/novnc_proxy)
elif [[ -x /usr/share/novnc/utils/launch.sh ]]; then
  novnc_command=(/usr/share/novnc/utils/launch.sh)
else
  echo "Could not find a noVNC launcher. Expected novnc_proxy or /usr/share/novnc/utils/launch.sh." >&2
  exit 1
fi

start_if_needed() {
  local pid_file="$1"
  shift

  if [[ -f "${pid_file}" ]]; then
    local pid
    pid="$(<"${pid_file}")"
    if kill -0 "${pid}" 2>/dev/null; then
      return
    fi
    rm -f "${pid_file}"
  fi

  nohup "$@" >>"${log_dir}/$(basename "${pid_file}" .pid).log" 2>&1 &
  echo "$!" >"${pid_file}"
}

wait_for_tcp_listener() {
  local host="$1"
  local port="$2"
  local attempts="${3:-40}"

  for _ in $(seq 1 "${attempts}"); do
    if bash -lc ">/dev/tcp/${host}/${port}" >/dev/null 2>&1; then
      return 0
    fi
    sleep 0.25
  done

  return 1
}

ensure_x_socket_dir() {
  if [[ ! -d "${x_socket_dir}" ]]; then
    sudo mkdir -p "${x_socket_dir}"
  fi

  sudo chown root:root "${x_socket_dir}"
  sudo chmod 1777 "${x_socket_dir}"
}

cleanup_stale_display_lock() {
  if [[ -S "${x_socket_dir}/X${display#:}" ]] && ! xdpyinfo -display "${display}" >/dev/null 2>&1; then
    sudo rm -f "${x_socket_dir}/X${display#:}"
  fi

  if [[ -f "${x_lock_file}" ]] && ! xdpyinfo -display "${display}" >/dev/null 2>&1; then
    sudo rm -f "${x_lock_file}"
  fi
}

ensure_x_socket_dir
cleanup_stale_display_lock

if ! xdpyinfo -display "${display}" >/dev/null 2>&1; then
  start_if_needed "${xvfb_pid_file}" \
    Xvfb "${display}" -screen 0 "${screen}" -ac -nolisten tcp +extension GLX +extension RANDR -noreset
fi

for _ in {1..20}; do
  if xdpyinfo -display "${display}" >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

if ! xdpyinfo -display "${display}" >/dev/null 2>&1; then
  echo "Failed to start Xvfb on ${display}." >&2
  if [[ -f "${log_dir}/xvfb.log" ]]; then
    tail -n 20 "${log_dir}/xvfb.log" >&2
  fi
  exit 1
fi

export DISPLAY="${display}"
unset WAYLAND_DISPLAY
unset WAYLAND_SOCKET
export XDG_SESSION_TYPE="x11"

start_if_needed "${wm_pid_file}" \
  bash -lc "DISPLAY='${display}' exec ${window_manager}"

if ! wait_for_tcp_listener "${vnc_host}" "${vnc_port}" 1; then
  rm -f "${x11vnc_pid_file}"
  start_if_needed "${x11vnc_pid_file}" \
    env -u WAYLAND_DISPLAY -u WAYLAND_SOCKET XDG_SESSION_TYPE=x11 \
      x11vnc -display "${display}" -rfbport "${vnc_port}" -localhost -forever -shared -nopw -xkb
fi

if ! wait_for_tcp_listener "${vnc_host}" "${vnc_port}"; then
  echo "x11vnc did not open ${vnc_host}:${vnc_port}." >&2
  if [[ -f "${log_dir}/x11vnc.log" ]]; then
    tail -n 40 "${log_dir}/x11vnc.log" >&2
  fi
  exit 1
fi

start_if_needed "${novnc_pid_file}" \
  "${novnc_command[@]}" --vnc "${vnc_host}:${vnc_port}" --listen "${novnc_port}"

cat <<EOF
noVNC session started.
DISPLAY=${display}
Browser URL: http://localhost:${novnc_port}/vnc.html?autoconnect=1&resize=scale
VNC backend: ${vnc_host}:${vnc_port}
Logs: ${log_dir}
Launch GUI apps in this shell, for example: DISPLAY=${display} rviz2
EOF
