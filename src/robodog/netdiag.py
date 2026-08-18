"""Connection diagnostics for the Wi-Fi transport.

"Timed out" is a useless message when the real cause is that the PC is on the
wrong network. The robot's access point puts it on a link-local 192.168.4.0/24
of its own, so the decisive question is: which local address would the OS use to
reach it? If that address is not in the robot's subnet, we are routing somewhere
else entirely — usually the home Wi-Fi, or a VPN tunnel that captured the route.
"""

from __future__ import annotations

import socket
import subprocess
from collections.abc import Callable

Runner = Callable[[list[str]], str]


def host_address(host: str) -> str:
    """Strip scheme and port from a host string."""
    if "://" in host:
        host = host.split("://", 1)[1]
    host = host.split("/", 1)[0]
    if host.count(":") == 1:  # host:port, not IPv6
        host = host.split(":", 1)[0]
    return host


def route_source_ip(host: str, port: int = 80) -> str | None:
    """Local address the OS would use to reach `host`, without sending anything.

    Connecting a UDP socket only sets the destination and picks a route.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.connect((host_address(host), port))
            source: str = probe.getsockname()[0]
            return source
    except OSError:
        return None


def same_ipv4_subnet(a: str, b: str, prefix: int = 24) -> bool:
    """True if both addresses share the first `prefix` bits."""
    try:
        octets_a = [int(part) for part in a.split(".")]
        octets_b = [int(part) for part in b.split(".")]
    except ValueError:
        return False
    if len(octets_a) != 4 or len(octets_b) != 4:
        return False
    packed_a = int.from_bytes(bytes(octets_a), "big")
    packed_b = int.from_bytes(bytes(octets_b), "big")
    mask = (0xFFFFFFFF << (32 - prefix)) & 0xFFFFFFFF
    return packed_a & mask == packed_b & mask


def _run_netsh(command: list[str]) -> str:
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )
    return completed.stdout


def current_ssid(run: Runner | None = None) -> str | None:
    """Best-effort current Wi-Fi SSID on Windows; None elsewhere or on failure."""
    runner = run if run is not None else _run_netsh
    try:
        output = runner(["netsh", "wlan", "show", "interfaces"])
    except (OSError, subprocess.SubprocessError):
        return None
    for line in output.splitlines():
        stripped = line.strip()
        # Skip 'AP BSSID'; match the plain 'SSID' line in any UI language.
        if stripped.startswith("SSID") and ":" in stripped:
            value = stripped.split(":", 1)[1].strip()
            if value:
                return value
    return None


def diagnose_unreachable(
    host: str,
    ap_ssid: str,
    ap_password: str,
    *,
    source_ip: str | None = None,
    ssid: str | None = None,
    probe: bool = True,
) -> list[str]:
    """Human-readable hints for why `host` could not be reached."""
    if probe:
        source_ip = route_source_ip(host) if source_ip is None else source_ip
        ssid = current_ssid() if ssid is None else ssid

    target = host_address(host)
    hints: list[str] = []

    if source_ip is None:
        hints.append(f"No route to {target} at all -- no network interface can reach it.")
    elif same_ipv4_subnet(source_ip, target):
        hints.append(
            f"Your PC is on the right subnet ({source_ip}), so the network side looks "
            f"fine. The robot may still be booting, or its web server is not up yet -- "
            f"try http://{target}/ in a browser."
        )
        return hints
    else:
        hints.append(
            f"Your PC would reach {target} via {source_ip}, which is NOT in the robot's "
            f"subnet -- so it is not connected to the robot."
        )

    if ssid:
        hints.append(f"Current Wi-Fi network: {ssid!r}.")
    hints.append(f"Join the robot's access point: SSID {ap_ssid!r}, password {ap_password!r}.")
    hints.append(
        "Windows will warn about 'no internet' on that network -- that is expected, the "
        "robot is not a router."
    )
    if source_ip is not None and _looks_like_vpn(source_ip):
        hints.append(
            f"A VPN appears to be capturing traffic (source {source_ip}). Disconnect it, "
            "or enable its 'allow LAN connections' option -- otherwise it will keep "
            "blocking the robot even once you are on its access point."
        )
    else:
        hints.append(
            "If a VPN is running, disconnect it or allow LAN connections; VPN kill "
            "switches block link-local traffic like this."
        )
    return hints


def _looks_like_vpn(source_ip: str) -> bool:
    """Heuristic: a CGNAT/tunnel-typical source for a link-local destination."""
    return source_ip.startswith("10.") or source_ip.startswith("100.")
