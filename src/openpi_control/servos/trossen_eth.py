"""Trossen iNerve Ethernet controller helpers: discovery, zeroing, and IP provisioning.

Central home for every Python-side interaction with the ``trossen_arm`` SDK
(same pinned release as the C++ ``libtrossen_arm`` linked by the native node,
so driver <-> controller firmware compatibility stays aligned):

- ``discover()``            read-only /24 TCP sweep for controllers
- ``reachable()``           single-host reachability gate (native connect)
- ``open_session()``        configured driver session (idled + cleaned up)
- ``set_zero_whole_arm()``  persist the current pose as zero (one EEPROM write)
- ``read_ip_settings()`` / ``provision_manual_ip()``  IP provisioning
- ``local_subnet_prefixes()`` / ``candidate_ips()`` / ``ip_in_use()``
                            host-side helpers for the provisioning flow
- ``wired_up_interfaces()`` / ``free_host_ip()`` / ``add_persistent_address()``
  / ``add_temporary_address()`` / ``remove_temporary_address()``
                            host NIC setup for the factory controller subnet
"""

from __future__ import annotations

import contextlib
import ipaddress
import logging
import pathlib
import shutil
import subprocess
import threading
import time
from collections.abc import Iterator
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass

import trossen_arm

PORT_TYPE = "ethernet"

# All joint offsets are persisted in ONE EEPROM write; per-servo zero commands
# would wear the controller flash once per joint (see set_zero_whole_arm).
WHOLE_ARM_ZERO = True

# Trossen model string accepted in model configs (must match the C++
# pi_trossen_shim.cpp TROSSEN_MODEL_WXAI_V0 constant).
TROSSEN_MODEL_WXAI_V0 = "wxai_v0"

# Vendor factory default: every controller ships at this address, so two
# factory-fresh arms on one LAN collide until they are provisioned.
FACTORY_DEFAULT_IP = "192.168.1.2"

# Default provisioning target per arm slot: the host octet on the controller's
# /24 subnet. When a base octet is already in use on the LAN the walk
# continues at base + k * PROVISION_HOST_OCTET_STEP (see candidate_ips), so
# every slot keeps a disjoint, predictable candidate set (e.g. follower_left:
# .11 / .111 / .211).
PROVISION_SLOT_HOST_OCTETS = {
    "follower_left": 11,
    "follower_right": 12,
    "leader_left": 21,
    "leader_right": 22,
}
PROVISION_HOST_OCTET_STEP = 100

# Host-side address on the controller subnet (the wizard can add it via
# NetworkManager when missing). The walk starts high to stay clear of typical
# LAN allocations and skips every arm provisioning candidate octet.
HOST_OCTET_START = 200
_ARM_CANDIDATE_OCTETS = frozenset(
    base + step
    for base in PROVISION_SLOT_HOST_OCTETS.values()
    for step in range(0, 255, PROVISION_HOST_OCTET_STEP)
    if base + step < 255
)

# Duplicate-address probes for a provisioning candidate. ping catches any live
# IP stack; arping -D (duplicate address detection) additionally catches hosts
# that drop ICMP. One second is enough for a LAN device to defend its address.
_PING_TIMEOUT_S = 1
_ARPING_TIMEOUT_S = 2

# Discovery probes every host of a /24 subnet with a short per-host TCP
# timeout. The vendor discover() probes serially, so the host range is split
# into chunks probed on parallel threads (the SDK releases the GIL during the
# socket wait). Worker count is capped at 16: measured on hardware in
# robot-test, 32 concurrent probes flood the link and the controller's reply
# gets dropped past the per-host timeout (missed discovery).
DISCOVERY_TIMEOUT_S = 0.05
DISCOVERY_CHUNK_HOSTS = 16
DISCOVERY_MAX_WORKERS = 16

# TCP handshake budget for an actual configure(): generous enough for a
# booting controller, small enough to fail fast on a wrong IP.
CONFIGURE_TIMEOUT_S = 10.0

# Single-host reachability gate: a controller that is momentarily busy (the
# config port is single-client) can miss one probe, so a few attempts with a
# longer per-host timeout are used.
REACHABILITY_TIMEOUT_S = 0.5
REACHABILITY_ATTEMPTS = 3
REACHABILITY_RETRY_DELAY_S = 1.0

# iNerve controllers use a PJRC Ethernet MAC. This lets the wizard distinguish
# a controller whose network stack answers but whose Trossen SDK service does
# not from a controller that is absent from the network entirely.
CONTROLLER_MAC_PREFIXES = ("04:e9:e5",)
_PING_REACHABILITY_TIMEOUT_S = 1
POWER_CYCLE_OFF_S = 10
CONTROLLER_BOOT_WAIT_S = 20

# After a provisioning reboot the controller needs time to come back before
# the re-discovery verification can succeed.
PROVISION_REBOOT_WAIT_S = 10.0

# Guards discover(): concurrent sweeps double the worker count, flood the
# link, and can make the controller's single-client config port stop
# answering real connects for a while.
_DISCOVER_SWEEP_LOCK = threading.Lock()


@dataclass(frozen=True)
class DiscoveredController:
    """One controller that answered a discovery probe."""

    address: str
    model: str
    firmware_version: str
    error_state: str


@dataclass(frozen=True)
class IpSettings:
    """Controller EEPROM network settings (read via a configure session)."""

    manual_ip: str
    ip_method: str
    firmware_version: str


def discover(subnets: list[str]) -> list[DiscoveredController]:
    """Scan IPv4 /24 ``subnets`` (prefixes like ``"192.168.1"``) for controllers.

    Read-only: probes TCP connects only, never changes host or controller
    network state.
    """
    with _DISCOVER_SWEEP_LOCK:
        return _discover_locked(subnets)


def reachable(ip: str) -> bool:
    """True when the controller at ``ip`` answers a discovery probe."""
    subnet, host = _split_host(ip)
    for attempt in range(REACHABILITY_ATTEMPTS):
        if attempt > 0:
            time.sleep(REACHABILITY_RETRY_DELAY_S)
        with _DISCOVER_SWEEP_LOCK:
            found = trossen_arm.TrossenArmDriver.discover(
                subnet=subnet, ip_start=host, ip_end=host, timeout=REACHABILITY_TIMEOUT_S
            )
        if found:
            return True
    return False


def ping_reachable_controller_ips(subnets: list[str]) -> list[str]:
    """Return controller-like hosts that answer ping on the scanned subnets.

    A discovery sweep populates the local ARP/neighbor table even when the
    controller's TCP service is unavailable. The known controller MAC prefix
    filters out unrelated LAN hosts before the explicit ping check.
    """
    result = subprocess.run(
        ["ip", "neigh", "show"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        logging.getLogger(__name__).warning("Could not inspect the IP neighbor table: %s", detail)
        return []

    candidates: list[str] = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if len(parts) < 5 or "lladdr" not in parts:
            continue
        ip = parts[0]
        try:
            address = ipaddress.IPv4Address(ip)
        except ipaddress.AddressValueError:
            continue
        prefix = ".".join(str(address).split(".")[:3])
        mac = parts[parts.index("lladdr") + 1].lower()
        if prefix not in subnets or not mac.startswith(CONTROLLER_MAC_PREFIXES):
            continue
        ping = subprocess.run(
            ["ping", "-c", "1", "-W", str(_PING_REACHABILITY_TIMEOUT_S), str(address)],
            capture_output=True,
            text=True,
            check=False,
        )
        if ping.returncode == 0:
            candidates.append(str(address))
    return sorted(set(candidates), key=ipaddress.IPv4Address)


@contextlib.contextmanager
def open_session(ip: str, *, clear_error: bool = False) -> Iterator[trossen_arm.TrossenArmDriver]:
    """Configured driver session for maintenance work; idled and cleaned up on exit.

    The follower end-effector mass set only matters for the controller's own
    gravity compensation while idle, which is correct for both roles at
    standstill (the maintenance path never drives the arm).
    """
    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        str(ip),
        clear_error,
        CONFIGURE_TIMEOUT_S,
    )
    try:
        yield driver
    finally:
        try:
            driver.set_all_modes(trossen_arm.Mode.idle)
        finally:
            driver.cleanup()


def set_zero_whole_arm(driver: trossen_arm.TrossenArmDriver) -> str | None:
    """Persist the current pose as zero via position_offset (one EEPROM write).

    Trossen semantics (docs "position_offset"): position_motor = position +
    position_offset, i.e. the offset is subtracted from motor feedback. To
    make the current pose read 0: new_offset = old_offset + current_position.

    Returns None on success or an error message (arm_zero driver contract).
    """
    positions = driver.get_all_positions()
    characteristics = driver.get_joint_characteristics()
    if len(characteristics) != len(positions):
        return (
            f"joint characteristics size mismatch: {len(characteristics)} characteristics "
            f"vs {len(positions)} positions"
        )
    for characteristic, position in zip(characteristics, positions, strict=True):
        characteristic.position_offset = float(characteristic.position_offset) + float(position)
    driver.set_joint_characteristics(characteristics)
    logging.getLogger(__name__).info(
        "Trossen zero set at current pose (new position_offsets: %s)",
        [round(float(c.position_offset), 4) for c in characteristics],
    )
    return None


def read_ip_settings(ip: str) -> IpSettings:
    """Read the controller's EEPROM network settings (no changes)."""
    with open_session(ip) as driver:
        return IpSettings(
            manual_ip=str(driver.get_manual_ip()),
            ip_method=str(driver.get_ip_method()),
            firmware_version=str(driver.get_controller_version()),
        )


def provision_manual_ip(current_ip: str, new_ip: str) -> None:
    """Persist ``new_ip`` as the controller's manual IP and reboot it.

    The controller writes the address to EEPROM and only applies it after the
    reboot; the caller must re-discover the controller at ``new_ip`` to verify
    (see PROVISION_REBOOT_WAIT_S). Raises on any SDK failure.
    """
    ipaddress.IPv4Address(new_ip)  # Fast-fail before touching the controller.
    driver = trossen_arm.TrossenArmDriver()
    driver.configure(
        trossen_arm.Model.wxai_v0,
        trossen_arm.StandardEndEffector.wxai_v0_follower,
        str(current_ip),
        False,
        CONFIGURE_TIMEOUT_S,
    )
    driver.set_manual_ip(str(new_ip))
    driver.set_ip_method(trossen_arm.IPMethod.manual)
    # cleanup(True) reboots the controller so the EEPROM settings take effect.
    driver.cleanup(True)
    logging.getLogger(__name__).info(
        "Trossen controller IP provisioned: %s -> %s (EEPROM, controller rebooting)",
        current_ip,
        new_ip,
    )


def local_subnet_prefixes() -> list[str]:
    """The /24 prefixes (``"a.b.c"``) of all global IPv4 addresses of this host.

    Read-only (``ip -4 addr show``); loopback and link-local are excluded.
    These are the subnets a discovery sweep can actually reach.
    """
    result = subprocess.run(
        ["ip", "-4", "-o", "addr", "show", "scope", "global"],
        capture_output=True,
        text=True,
        check=True,
    )
    prefixes: list[str] = []
    for line in result.stdout.splitlines():
        parts = line.split()
        if "inet" not in parts:
            continue
        address = parts[parts.index("inet") + 1].split("/")[0]
        prefix = ".".join(address.split(".")[:3])
        if prefix.startswith(("127.", "169.254.")) or prefix in prefixes:
            continue
        prefixes.append(prefix)
    return prefixes


def candidate_ips(subnet: str, slot: str) -> list[str]:
    """Provisioning candidate addresses for ``slot`` on the ``"a.b.c"`` subnet.

    The walk starts at the slot's reserved host octet and steps by
    PROVISION_HOST_OCTET_STEP, keeping each slot's candidates disjoint from
    every other slot's. When all candidates are in use the caller must ask
    for a manual address.
    """
    base = PROVISION_SLOT_HOST_OCTETS[slot]
    return [f"{subnet}.{octet}" for octet in range(base, 255, PROVISION_HOST_OCTET_STEP)]


def ip_in_use(ip: str, interface: str | None = None) -> bool:
    """Best-effort duplicate-address check for a provisioning candidate.

    True when anything on the LAN answers a ping or an ``arping -D``
    duplicate-address probe. arping needs the outgoing ``interface`` and is
    skipped when unavailable (not installed, or no raw-socket privilege) —
    ping alone still catches every normal IP stack.
    """
    ping = subprocess.run(
        ["ping", "-c", "1", "-W", str(_PING_TIMEOUT_S), str(ip)],
        capture_output=True,
        text=True,
        check=False,
    )
    if ping.returncode == 0:
        return True
    if interface is None or shutil.which("arping") is None:
        return False
    # arping -D exits 0 when the address is free, non-zero when a reply came
    # back (duplicate) — but also on errors (e.g. missing raw-socket
    # privilege), so only a reply actually printed counts as "in use".
    arping = subprocess.run(
        ["arping", "-D", "-c", "2", "-w", str(_ARPING_TIMEOUT_S), "-I", str(interface), str(ip)],
        capture_output=True,
        text=True,
        check=False,
    )
    return arping.returncode != 0 and "reply from" in arping.stdout.lower()


def wired_up_interfaces(sys_class_net: pathlib.Path = pathlib.Path("/sys/class/net")) -> list[str]:
    """Wired Ethernet interfaces whose link is up, sorted by name.

    Wireless interfaces are excluded (adding addresses there can disturb the
    Wi-Fi LAN), as are loopback and non-Ethernet devices.
    """
    interfaces: list[str] = []
    if not sys_class_net.is_dir():
        return interfaces
    for iface in sys_class_net.iterdir():
        if (iface / "wireless").exists():
            continue
        try:
            # ARPHRD_ETHER == 1; excludes loopback (772), CAN (280), ...
            if (iface / "type").read_text().strip() != "1":
                continue
            if (iface / "operstate").read_text().strip() != "up":
                continue
        except OSError:
            continue  # interface disappeared mid-scan
        interfaces.append(iface.name)
    return sorted(interfaces)


def free_host_ip(subnet: str, interface: str) -> str | None:
    """A conflict-free host address on the ``"a.b.c"`` /24 for this machine.

    Walks from HOST_OCTET_START upward, skipping every arm provisioning
    candidate octet and every address something on the LAN already answers
    for (ping + arping duplicate detection via ip_in_use). None when the
    whole range is in use.
    """
    for octet in range(HOST_OCTET_START, 255):
        if octet in _ARM_CANDIDATE_OCTETS:
            continue
        candidate = f"{subnet}.{octet}"
        if not ip_in_use(candidate, interface):
            return candidate
    return None


def add_temporary_address(interface: str, cidr: str) -> None:
    """Add ``cidr`` to ``interface`` until reboot/removal (``sudo ip addr add``).

    Used to probe which NIC reaches the arm controllers before anything is
    persisted; pair with remove_temporary_address. Raises RuntimeError on
    failure.
    """
    result = subprocess.run(
        ["sudo", "ip", "addr", "add", str(cidr), "dev", str(interface)],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        raise RuntimeError(f"adding temporary {cidr} to {interface} failed: {detail}")


def remove_temporary_address(interface: str, cidr: str) -> None:
    """Remove an address added by add_temporary_address (best effort, logged)."""
    result = subprocess.run(
        ["sudo", "ip", "addr", "del", str(cidr), "dev", str(interface)],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        detail = result.stderr.strip() or result.stdout.strip()
        logging.getLogger(__name__).warning(
            "Removing temporary %s from %s failed: %s", cidr, interface, detail
        )


def add_persistent_address(interface: str, cidr: str) -> None:
    """Add ``cidr`` (e.g. "192.168.1.200/24") to ``interface`` via NetworkManager.

    The address is ADDED next to the connection's existing addresses (DHCP
    stays untouched) and persists across reboots. Raises RuntimeError when the
    interface has no active NetworkManager connection or nmcli fails — the
    caller falls back to printing manual setup instructions.
    """
    if shutil.which("nmcli") is None:
        raise RuntimeError(
            "nmcli not found — configure the address manually "
            "(see packages/openpi-runtime/docs/trossen/README.md)"
        )
    show = subprocess.run(
        ["nmcli", "-g", "GENERAL.CONNECTION", "device", "show", str(interface)],
        capture_output=True,
        text=True,
        check=False,
    )
    connection = show.stdout.strip()
    if show.returncode != 0 or not connection or connection == "--":
        raise RuntimeError(
            f"interface {interface} has no active NetworkManager connection — "
            "configure the address manually (see packages/openpi-runtime/docs/trossen/README.md)"
        )
    for command in (
        ["sudo", "nmcli", "connection", "modify", connection, "+ipv4.addresses", str(cidr)],
        ["sudo", "nmcli", "connection", "up", connection],
    ):
        result = subprocess.run(command, capture_output=True, text=True, check=False)
        if result.returncode != 0:
            detail = result.stderr.strip() or result.stdout.strip()
            raise RuntimeError(f"{' '.join(command)} failed: {detail}")
    logging.getLogger(__name__).info(
        "Added %s to NetworkManager connection %r (interface %s)", cidr, connection, interface
    )


def interface_for_subnet(subnet: str) -> str | None:
    """The host interface owning an address on the ``"a.b.c"`` /24, if any."""
    result = subprocess.run(
        ["ip", "-4", "-o", "addr", "show", "scope", "global"],
        capture_output=True,
        text=True,
        check=True,
    )
    for line in result.stdout.splitlines():
        parts = line.split()
        if "inet" not in parts:
            continue
        address = parts[parts.index("inet") + 1].split("/")[0]
        if ".".join(address.split(".")[:3]) == subnet:
            return parts[1]
    return None


def _discover_locked(subnets: list[str]) -> list[DiscoveredController]:
    def _probe_chunk(subnet: str, ip_start: int, ip_end: int) -> list[DiscoveredController]:
        return [
            DiscoveredController(
                address=str(found.ip),
                model=str(trossen_arm.MODEL_NAME[found.model]),
                firmware_version=str(found.firmware_version),
                error_state=str(trossen_arm.ERROR_INFORMATION[found.error_state]),
            )
            for found in trossen_arm.TrossenArmDriver.discover(
                subnet=subnet,
                ip_start=ip_start,
                ip_end=ip_end,
                timeout=DISCOVERY_TIMEOUT_S,
            )
        ]

    chunks = [
        (subnet, start, min(start + DISCOVERY_CHUNK_HOSTS - 1, 254))
        for subnet in subnets
        for start in range(1, 255, DISCOVERY_CHUNK_HOSTS)
    ]
    results: list[DiscoveredController] = []
    if chunks:
        with ThreadPoolExecutor(max_workers=DISCOVERY_MAX_WORKERS) as pool:
            for chunk_result in pool.map(lambda args: _probe_chunk(*args), chunks):
                results.extend(chunk_result)
    if results:
        logging.getLogger(__name__).info(
            "Discovered %d Trossen controller(s): %s", len(results), [r.address for r in results]
        )
    return results


def _split_host(ip: str) -> tuple[str, int]:
    """Split ``"a.b.c.d"`` into the ``"a.b.c"`` prefix and the host octet."""
    ipaddress.IPv4Address(ip)
    octets = str(ip).split(".")
    return ".".join(octets[:3]), int(octets[3])
