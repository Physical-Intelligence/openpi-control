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

for command_name in autoreconf cmake git make; do
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
# Trossen iNerve controller SDK (prebuilt static lib + headers). Pinned so the
# driver <-> controller firmware compatibility stays reproducible: the
# controller rejects a driver whose major.minor doesn't match its firmware.
trossen_arm_source="${deps_root}/trossen_arm"
trossen_arm_prefix="${trossen_arm_source}/install"
trossen_arm_ref="${TROSSEN_ARM_GIT_REF:-v1.10.0}"
libfranka_source="${deps_root}/libfranka"
libfranka_prefix="${libfranka_source}/install"
libfranka_ref="${LIBFRANKA_GIT_REF:-0.21.3}"
libmodbus_source="${deps_root}/libmodbus"
libmodbus_prefix="${libmodbus_source}/install"
libmodbus_ref="${LIBMODBUS_GIT_REF:-v3.1.12}"
build_jobs="${BUILD_JOBS:-2}"

# A cached clone built from a different pinned ref must not be reused
# silently: fail fast with instructions instead.
verify_cached_ref() {
  local source_dir="$1"
  local expected_ref="$2"
  [[ -d "${source_dir}/.git" ]] || return 0
  local actual_ref
  actual_ref="$(git -C "${source_dir}" describe --tags --exact-match 2>/dev/null || echo unknown)"
  if [[ "${actual_ref}" != "${expected_ref}" ]]; then
    echo "Cached ${source_dir} is at ref '${actual_ref}' but '${expected_ref}' is pinned." >&2
    echo "Remove it and re-run:  rm -rf ${source_dir}" >&2
    exit 2
  fi
}

verify_cached_ref "${pinocchio_source}" "${pinocchio_ref}"
verify_cached_ref "${cppzmq_source}" "${cppzmq_ref}"
verify_cached_ref "${trossen_arm_source}" "${trossen_arm_ref}"
verify_cached_ref "${libfranka_source}" "${libfranka_ref}"
verify_cached_ref "${libmodbus_source}" "${libmodbus_ref}"

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

# Per-dep skip: verify_cached_ref above already guarantees that any cached
# clone is at the pinned ref (a ref bump against an old cache fails fast), so
# an existing install artifact means this exact pinned version is installed.
if [[ -f "${pinocchio_prefix}/lib/pkgconfig/pinocchio.pc" ]]; then
  echo "Pinocchio ${pinocchio_ref} already installed at ${pinocchio_prefix}; skipping."
else
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
fi

if [[ -f "${libfranka_prefix}/lib/libfranka.a" ]]; then
  echo "libfranka ${libfranka_ref} already installed at ${libfranka_prefix}; skipping."
else
  if [[ ! -d "${libfranka_source}/.git" ]]; then
    git clone --branch "${libfranka_ref}" --depth 1 --recurse-submodules --shallow-submodules \
      https://github.com/frankarobotics/libfranka.git "${libfranka_source}"
  fi

  libfranka_patch="${repo_root}/scripts/patches/libfranka-0.21.3-static.patch"
  if git -C "${libfranka_source}" apply --check "${libfranka_patch}"; then
    git -C "${libfranka_source}" apply "${libfranka_patch}"
  elif ! git -C "${libfranka_source}" apply --reverse --check "${libfranka_patch}"; then
    echo "The pinned libfranka source does not accept ${libfranka_patch}." >&2
    exit 1
  fi

  cmake -S "${libfranka_source}" -B "${libfranka_source}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${libfranka_prefix}" \
    -DCMAKE_PREFIX_PATH="${pinocchio_prefix}" \
    -DBUILD_EXAMPLES=OFF \
    -DBUILD_TESTS=OFF \
    -DGENERATE_PYLIBFRANKA=OFF
  cmake --build "${libfranka_source}/build" --parallel "${build_jobs}"
  cmake --install "${libfranka_source}/build"
fi

if [[ -f "${libmodbus_prefix}/lib/libmodbus.a" ]]; then
  echo "libmodbus ${libmodbus_ref} already installed at ${libmodbus_prefix}; skipping."
else
  if [[ ! -d "${libmodbus_source}/.git" ]]; then
    git clone --branch "${libmodbus_ref}" --depth 1 \
      https://github.com/stephane/libmodbus.git "${libmodbus_source}"
  fi

  (
    cd "${libmodbus_source}"
    ./autogen.sh
    ./configure --prefix="${libmodbus_prefix}" --enable-static --disable-shared
    make -j"${build_jobs}"
    make install
  )
fi

if [[ -f "${trossen_arm_prefix}/lib/libtrossen_arm.a" ]]; then
  echo "libtrossen_arm ${trossen_arm_ref} already installed at ${trossen_arm_prefix}; skipping."
else
  if [[ ! -d "${trossen_arm_source}/.git" ]]; then
    git clone --branch "${trossen_arm_ref}" --depth 1 \
      https://github.com/TrossenRobotics/trossen_arm.git "${trossen_arm_source}"
  fi

  # The repo ships a prebuilt static library + headers per OS/arch; "building"
  # is just a CMake install into the prefix.
  cmake -S "${trossen_arm_source}" -B "${trossen_arm_source}/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="${trossen_arm_prefix}"
  cmake --build "${trossen_arm_source}/build" --parallel "${build_jobs}"
  cmake --install "${trossen_arm_source}/build"
fi

if [[ ! -f "${pinocchio_prefix}/lib/pkgconfig/pinocchio.pc" ]]; then
  echo "Pinocchio installation is missing pinocchio.pc: ${pinocchio_prefix}" >&2
  exit 1
fi
if [[ ! -f "${cppzmq_prefix}/include/zmq.hpp" ]]; then
  echo "cppzmq installation is missing zmq.hpp: ${cppzmq_prefix}" >&2
  exit 1
fi
if [[ ! -f "${trossen_arm_prefix}/lib/libtrossen_arm.a" ]]; then
  echo "libtrossen_arm installation is missing libtrossen_arm.a: ${trossen_arm_prefix}" >&2
  exit 1
fi
if [[ ! -f "${libfranka_prefix}/lib/libfranka.a" ]]; then
  echo "libfranka installation is missing libfranka.a: ${libfranka_prefix}" >&2
  exit 1
fi
if [[ ! -f "${libmodbus_prefix}/lib/libmodbus.a" ]]; then
  echo "libmodbus installation is missing libmodbus.a: ${libmodbus_prefix}" >&2
  exit 1
fi

cat <<EOF

cppzmq ${cppzmq_ref} is installed at:
  ${cppzmq_prefix}

Pinocchio ${pinocchio_ref} is installed at:
  ${pinocchio_prefix}

libtrossen_arm ${trossen_arm_ref} is installed at:
  ${trossen_arm_prefix}

libfranka ${libfranka_ref} is installed at:
  ${libfranka_prefix}

libmodbus ${libmodbus_ref} is installed at:
  ${libmodbus_prefix}

CMake discovers these prefixes automatically. Build a wheel from this directory with:
  uv build --wheel
EOF
