#!/usr/bin/env bash
set -euo pipefail

workspace="${RANGER_WS:-/home/zbx/ranger_ws}"
port="${OFFICE_RPG_PORT:-8502}"
llm_env_file="${RANGER_LLM_ENV_FILE:-/home/zbx/.config/ranger-office-rpg/llm.env}"
office_rpg_inherited_http_proxy="${HTTPS_PROXY:-${HTTP_PROXY:-}}"
set +u
source /opt/ros/humble/setup.bash
if [[ -f "${workspace}/install/setup.bash" ]]; then
  source "${workspace}/install/setup.bash"
fi
if [[ -f "${llm_env_file}" ]]; then
  # Private, repository-external configuration. Never print the API key.
  source "${llm_env_file}"
fi
set -u

# Clash Verge exposes a mixed HTTP/SOCKS port.  The OpenAI SDK in this ROS
# environment supports its HTTP proxy endpoint, while an inherited
# ALL_PROXY=socks://... is rejected before any API request is sent.  Clear only
# the ambiguous catch-all proxy for this demo process; HTTP(S)_PROXY remains in
# place and the parent shell / ChatGPT / Codex environments are untouched.
unset ALL_PROXY all_proxy || true
if [[ -z "${HTTPS_PROXY:-}" && -n "${office_rpg_inherited_http_proxy}" ]]; then
  export HTTPS_PROXY="${office_rpg_inherited_http_proxy}"
  export https_proxy="${office_rpg_inherited_http_proxy}"
fi
if [[ -z "${HTTP_PROXY:-}" && -n "${office_rpg_inherited_http_proxy}" ]]; then
  export HTTP_PROXY="${office_rpg_inherited_http_proxy}"
  export http_proxy="${office_rpg_inherited_http_proxy}"
fi

if [[ -n "${DEEPSEEK_API_KEY:-}" ]]; then
  demo_provider="deepseek"
  echo "DeepSeek API: configured (${LLM_MODEL:-deepseek-v4-flash})"
else
  demo_provider="offline"
  echo "DeepSeek API: not configured; use Offline (demo will not crash)"
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "FAILED: ROS 2 environment is unavailable" >&2
  exit 1
fi
if ss -lnt 2>/dev/null | rg -q ":${port}[[:space:]]"; then
  echo "FAILED: port ${port} is already in use" >&2
  exit 1
fi
if ps -eo cmd | rg -q '[o]ffice_rpg_demo.launch|[s]treamlit run .*/web_ui/app.py'; then
  echo "FAILED: residual EGA-OfficeNav process detected; stop it before launch" >&2
  exit 1
fi
if ! ros2 pkg prefix rangermini_doorway_sim >/dev/null 2>&1; then
  echo "Build result missing; building rangermini_doorway_sim..."
  (cd "${workspace}" && colcon build --packages-select rangermini_doorway_sim)
  set +u
  source "${workspace}/install/setup.bash"
  set -u
fi

# All demo participants run on this machine. A process-local Fast DDS fallback
# avoids stale host-specific CycloneDDS interface files without editing them.
if [[ "${OFFICE_RPG_LOCAL_DDS:-true}" == "true" ]]; then
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  unset CYCLONEDDS_URI || true
fi
export ROS_LOG_DIR="${ROS_LOG_DIR:-/tmp/office_rpg_ros_logs}"

children=()
cleanup() {
  trap - INT TERM EXIT
  for pid in "${children[@]}"; do
    kill -INT "${pid}" 2>/dev/null || true
  done
  wait 2>/dev/null || true
}
trap cleanup INT TERM EXIT

ros2 launch rangermini_doorway_sim office_rpg_demo.launch.py \
  launch_web_ui:=false use_pointcloud_to_laserscan:=false \
  llm_provider:="${demo_provider}" "$@" &
children+=("$!")

for _ in $(seq 1 40); do
  if ros2 service type /office_rpg/stop >/dev/null 2>&1; then
    break
  fi
  sleep 0.25
done

streamlit run "${workspace}/src/rangermini_doorway_sim/web_ui/app.py" \
  --server.port "${port}" --server.headless true &
children+=("$!")
echo "Frontend: http://localhost:${port}"
echo "Provider: ${LLM_MODEL:-deepseek-v4-flash} / offline"
echo "Demo task: ready"
wait
