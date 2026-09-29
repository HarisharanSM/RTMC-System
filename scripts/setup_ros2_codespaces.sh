#!/usr/bin/env bash
# Install the headless ROS 2 Jazzy toolchain in an Ubuntu 24.04 Codespace.
set -Eeuo pipefail

die() { printf 'setup_ros2_codespaces: %s\n' "$*" >&2; exit 1; }
repo=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)
apt_install() { sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends "$@"; }

[[ -r /etc/os-release ]] || die 'missing /etc/os-release'
# shellcheck disable=SC1091
source /etc/os-release
[[ ${ID:-} == ubuntu && ${VERSION_ID:-} == 24.04 && ${VERSION_CODENAME:-} == noble ]] ||
  die 'requires Ubuntu 24.04 (noble)'
arch=$(dpkg --print-architecture)
[[ $arch == amd64 || $arch == arm64 ]] || die "unsupported architecture: $arch (expected amd64 or arm64)"
command -v sudo >/dev/null || die 'sudo is required'
sudo -n true 2>/dev/null || die 'passwordless sudo is required for Codespaces postCreateCommand'
command -v apt-get >/dev/null || die 'apt-get is required'

export DEBIAN_FRONTEND=noninteractive
sudo apt-get update
apt_install \
  ca-certificates curl python3 python3-pip python3-venv python3-setuptools \
  build-essential cmake pkg-config git locales software-properties-common

# Ubuntu Universe supplies ROS development dependencies. This command is
# idempotent when Universe is already enabled in the base image.
sudo add-apt-repository -y universe
sudo apt-get update

if ! locale -a | grep -qi '^en_US\.utf8$'; then
  sudo locale-gen en_US en_US.UTF-8
fi
sudo update-locale LANG=en_US.UTF-8
export LANG=en_US.UTF-8

if ! dpkg-query -W -f='${Status}' ros2-apt-source 2>/dev/null | grep -qx 'install ok installed'; then
  command -v python3 >/dev/null || die 'Python 3 is required to read the ROS apt-source release metadata'
  release=$(python3 - <<'PY'
import json
import re
import urllib.request

request = urllib.request.Request(
    "https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest",
    headers={"Accept": "application/vnd.github+json", "User-Agent": "rtmc-codespaces-setup"},
)
with urllib.request.urlopen(request, timeout=20) as response:
    version = json.load(response)["tag_name"]
if not re.fullmatch(r"[0-9]+(?:\.[0-9]+)*", version):
    raise SystemExit(f"Unexpected ros-apt-source version: {version!r}")
print(version)
PY
  )
  package=$(mktemp --suffix=.deb)
  trap 'rm -f "$package"' EXIT
  curl --fail --location --show-error --silent --retry 3 \
    --output "$package" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${release}/ros2-apt-source_${release}.noble_all.deb"
  sudo dpkg -i "$package"
  rm -f "$package"
  trap - EXIT
fi

sudo apt-get update
apt_install \
  ros-jazzy-ros-base \
  ros-jazzy-rclpy \
  ros-jazzy-rcl-interfaces \
  ros-jazzy-sensor-msgs \
  ros-jazzy-diagnostic-msgs \
  ros-jazzy-std-msgs \
  ros-jazzy-builtin-interfaces \
  ros-jazzy-ament-cmake \
  ros-jazzy-ament-cmake-python \
  ros-jazzy-launch-ros \
  ros-jazzy-rosidl-default-generators \
  ros-jazzy-rosidl-default-runtime \
  ros-jazzy-rosbag2 \
  ros-jazzy-rosbag2-storage-default-plugins \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pytest

[[ -r /opt/ros/jazzy/setup.bash ]] || die 'Jazzy installation did not produce /opt/ros/jazzy/setup.bash'
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update --rosdistro jazzy
rosdep install --from-paths "$repo/ros2/src" --ignore-src --rosdistro jazzy -y
printf 'ROS 2 Jazzy is installed. Source /opt/ros/jazzy/setup.bash in each Bash terminal.\n'
