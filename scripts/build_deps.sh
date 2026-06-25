#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -s)" != "Linux" ]]; then
  echo "build_deps.sh supports Linux only" >&2
  exit 2
fi

if [[ "${EUID}" -eq 0 ]]; then
  echo "Run this script as a normal user, not with sudo." >&2
  exit 2
fi

for command_name in cmake git; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "Missing ${command_name}; first run sudo ./scripts/install_build_deps_ubuntu.sh" >&2
    exit 2
  fi
done

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
deps_root="${OPENPI_CONTROL_DEPS_DIR:-${repo_root}/.deps}"
pinocchio_source="${deps_root}/pinocchio"
pinocchio_prefix="${pinocchio_source}/install"
pinocchio_ref="${PINOCCHIO_GIT_REF:-v3.4.0}"
cppzmq_source="${deps_root}/cppzmq"
cppzmq_prefix="${cppzmq_source}/install"
cppzmq_ref="${CPPZMQ_GIT_REF:-v4.9.0}"
build_jobs="${BUILD_JOBS:-2}"

if ! [[ "${build_jobs}" =~ ^[1-9][0-9]*$ ]]; then
  echo "BUILD_JOBS must be a positive integer; got ${build_jobs@Q}" >&2
  exit 2
fi

mkdir -p "${deps_root}"

if [[ ! -d "${cppzmq_source}/.git" ]]; then
  git clone --branch "${cppzmq_ref}" --depth 1 \
    https://github.com/zeromq/cppzmq.git "${cppzmq_source}"
fi

install -d "${cppzmq_prefix}/include"
install -m 0644 "${cppzmq_source}/zmq.hpp" "${cppzmq_prefix}/include/zmq.hpp"
install -m 0644 "${cppzmq_source}/zmq_addon.hpp" "${cppzmq_prefix}/include/zmq_addon.hpp"

if [[ ! -d "${pinocchio_source}/.git" ]]; then
  git clone --branch "${pinocchio_ref}" --depth 1 \
    https://github.com/stack-of-tasks/pinocchio.git "${pinocchio_source}"
fi

if [[ ! -f "${pinocchio_source}/cmake/base.cmake" ]]; then
  git -C "${pinocchio_source}" submodule update --init --depth 1 cmake \
    || git -C "${pinocchio_source}" submodule update --init cmake
fi

cmake -S "${pinocchio_source}" -B "${pinocchio_source}/build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${pinocchio_prefix}" \
  -DBUILD_BENCHMARK=OFF \
  -DBUILD_EXAMPLES=OFF \
  -DBUILD_PYTHON_INTERFACE=OFF \
  -DBUILD_TESTING=OFF \
  -DENABLE_TEMPLATE_INSTANTIATION=OFF
cmake --build "${pinocchio_source}/build" --parallel "${build_jobs}"
cmake --install "${pinocchio_source}/build"

if [[ ! -f "${pinocchio_prefix}/lib/pkgconfig/pinocchio.pc" ]]; then
  echo "Pinocchio installation is missing pinocchio.pc: ${pinocchio_prefix}" >&2
  exit 1
fi
if [[ ! -f "${cppzmq_prefix}/include/zmq.hpp" ]]; then
  echo "cppzmq installation is missing zmq.hpp: ${cppzmq_prefix}" >&2
  exit 1
fi

cat <<EOF

cppzmq ${cppzmq_ref} is installed at:
  ${cppzmq_prefix}

Pinocchio ${pinocchio_ref} is installed at:
  ${pinocchio_prefix}

CMake discovers these prefixes automatically. Build a wheel from this directory with:
  uv build --wheel
EOF
