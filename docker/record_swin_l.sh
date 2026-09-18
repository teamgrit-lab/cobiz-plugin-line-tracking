#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF'
Record topics from the already-running debugging-swin-l container into MCAP.

Usage:
  ./docker/record_swin_l.sh [options] [topic ...]

The container is not started automatically. Start it separately with:
  docker compose --profile debug up -d debugging-swin-l

Options:
  -o, --output-dir DIR  Bag directory inside the container (default: /rosbags)
  -n, --name NAME       Bag name (default: swin_l_<UTC timestamp>)
  -h, --help            Show this help

When no topics are supplied, the predefined recording topic list is used.
Extra topics can be supplied through SWIN_L_RECORD_EXTRA_TOPICS as a
space-separated list.
EOF
}

if (($# == 1)) && [[ "$1" == "-h" || "$1" == "--help" ]]; then
  usage
  exit 0
fi

if ! command -v docker >/dev/null 2>&1; then
  echo "[record-swin-l] docker is not available on the host" >&2
  exit 1
fi

script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
project_dir="$(cd -- "${script_dir}/.." && pwd)"
cd "${project_dir}"

service="${SWIN_L_RECORD_SERVICE:-debugging-swin-l}"
compose=(docker compose --profile debug)

if ! "${compose[@]}" ps --status running --services | grep -Fxq "${service}"; then
  echo "[record-swin-l] service '${service}' is not running" >&2
  echo "[record-swin-l] start it first: docker compose --profile debug up -d ${service}" >&2
  exit 1
fi

exec "${compose[@]}" exec -T \
  -e "SWIN_L_RECORD_EXTRA_TOPICS=${SWIN_L_RECORD_EXTRA_TOPICS:-}" \
  "${service}" bash -s -- "$@" <<'CONTAINER_SCRIPT'
set -euo pipefail

usage() {
  cat <<'EOF'
Record the debugging-swin-l ROS 2 topics into an MCAP bag.

Usage:
  record-swin-l [options] [topic ...]

Options:
  -o, --output-dir DIR  Bag directory inside the container (default: /rosbags)
  -n, --name NAME       Bag name (default: swin_l_<UTC timestamp>)
  -h, --help            Show this help
EOF
}

ros_distro="${ROS_DISTRO:-humble}"
output_dir="/rosbags"
bag_name="swin_l_$(date -u +%Y%m%dT%H%M%SZ)"
declare -a record_topics=()

while (($# > 0)); do
  case "$1" in
    -o|--output-dir)
      if (($# < 2)); then
        echo "[record-swin-l] --output-dir requires a value" >&2
        exit 2
      fi
      output_dir="$2"
      shift 2
      ;;
    -n|--name)
      if (($# < 2)); then
        echo "[record-swin-l] --name requires a value" >&2
        exit 2
      fi
      bag_name="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    --)
      shift
      record_topics+=("$@")
      break
      ;;
    -*)
      echo "[record-swin-l] unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
    *)
      record_topics+=("$1")
      shift
      ;;
  esac
done

if [[ -z "${bag_name}" || "${bag_name}" == */* ]]; then
  echo "[record-swin-l] bag name must be non-empty and must not contain '/'" >&2
  exit 2
fi

ros_setup="/opt/ros/${ros_distro}/setup.bash"
if [[ ! -r "${ros_setup}" ]]; then
  echo "[record-swin-l] ROS setup file is not readable: ${ros_setup}" >&2
  exit 1
fi
source "${ros_setup}"

teamgrit_dds_env="${TEAMGRIT_DDS_ENV:-/opt/ros/teamgrit/dds/teamgrit_dds_env.sh}"
if [[ ! -r "${teamgrit_dds_env}" ]]; then
  echo "[record-swin-l] TeamGRIT DDS environment is not readable: ${teamgrit_dds_env}" >&2
  exit 1
fi
source "${teamgrit_dds_env}"

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[record-swin-l] ros2 is not available after sourcing ROS ${ros_distro}" >&2
  exit 1
fi

if ((${#record_topics[@]} == 0)); then
  record_topics=(
    /tf
    /tf_static
    /rosout
    /a2/front_camera/res_720p/image_raw
    /a2/front_camera/res_720p/camera_info
    /line_tracking/swin_l/local_path
    /line_tracking/swin_l/safety_stop
    /line_tracking/swin_l/metrics
    /a2_control
  )

  declare -a extra_topics=()
  read -r -a extra_topics <<< "${SWIN_L_RECORD_EXTRA_TOPICS:-}"
  record_topics+=("${extra_topics[@]}")
fi

mkdir -p "${output_dir}"
bag_path="${output_dir%/}/${bag_name}"

echo "[record-swin-l] recording ${#record_topics[@]} topic(s) to ${bag_path}"
printf '  %s\n' "${record_topics[@]}"

exec ros2 bag record \
  --storage mcap \
  --output "${bag_path}" \
  "${record_topics[@]}"
CONTAINER_SCRIPT
