"""Connection diagnostics: subnet reasoning, SSID parsing, hint quality."""

from __future__ import annotations

import pytest

from robodog.netdiag import (
    current_ssid,
    diagnose_unreachable,
    host_address,
    route_source_ip,
    same_ipv4_subnet,
)

AP_SSID = "WAVESHARE Robot"
AP_PASSWORD = "1234567890"
ROBOT = "192.168.4.1"


@pytest.mark.parametrize(
    ("given", "expected"),
    [
        ("192.168.4.1", "192.168.4.1"),
        ("192.168.4.1:80", "192.168.4.1"),
        ("http://192.168.4.1", "192.168.4.1"),
        ("http://192.168.4.1:8080/", "192.168.4.1"),
        ("robot.local", "robot.local"),
    ],
)
def test_host_address_strips_scheme_and_port(given: str, expected: str) -> None:
    assert host_address(given) == expected


@pytest.mark.parametrize(
    ("a", "b", "expected"),
    [
        ("192.168.4.2", "192.168.4.1", True),
        ("192.168.4.255", "192.168.4.1", True),
        ("192.168.178.98", "192.168.4.1", False),
        ("10.2.0.2", "192.168.4.1", False),
        ("garbage", "192.168.4.1", False),
        ("1.2.3", "192.168.4.1", False),
    ],
)
def test_same_ipv4_subnet(a: str, b: str, expected: bool) -> None:
    assert same_ipv4_subnet(a, b) is expected


def test_route_source_ip_returns_a_local_address() -> None:
    # Routing to a public address must yield some local source on any machine.
    assert route_source_ip("192.0.2.1") is not None


def test_route_source_ip_handles_garbage() -> None:
    assert route_source_ip("not a host at all") is None


# --- SSID parsing (netsh output, any UI language) -----------------------------


GERMAN_NETSH = """
Es ist 1 Schnittstelle auf dem System vorhanden:

    Name                   : WLAN
    Status                  : Verbunden
    SSID                   : Alfie
    AP BSSID               : 2c:3a:fd:bc:b4:ee
    Signal              : 82%
"""

ENGLISH_NETSH = """
There is 1 interface on the system:

    Name                   : Wi-Fi
    State                  : connected
    SSID                   : WAVESHARE Robot
    BSSID                  : aa:bb:cc:dd:ee:ff
"""


def test_ssid_is_parsed_from_german_output() -> None:
    assert current_ssid(lambda _cmd: GERMAN_NETSH) == "Alfie"


def test_ssid_is_parsed_from_english_output() -> None:
    assert current_ssid(lambda _cmd: ENGLISH_NETSH) == "WAVESHARE Robot"


def test_ssid_is_none_when_not_connected() -> None:
    assert current_ssid(lambda _cmd: "There is 1 interface\n    State : disconnected\n") is None


def test_ssid_survives_a_missing_netsh() -> None:
    def explode(_cmd: list[str]) -> str:
        raise OSError("netsh not found")

    assert current_ssid(explode) is None


# --- hints --------------------------------------------------------------------


def hints_for(source_ip: str | None, ssid: str | None = None) -> str:
    return "\n".join(
        diagnose_unreachable(
            ROBOT, AP_SSID, AP_PASSWORD, source_ip=source_ip, ssid=ssid, probe=False
        )
    )


def test_wrong_network_hint_names_the_access_point() -> None:
    text = hints_for("192.168.178.98", "Alfie")
    assert "not connected to the robot" in text.lower()
    assert AP_SSID in text
    assert AP_PASSWORD in text
    assert "'Alfie'" in text


def test_wrong_network_hint_explains_the_no_internet_warning() -> None:
    assert "no internet" in hints_for("192.168.178.98").lower()


def test_vpn_source_address_is_called_out() -> None:
    text = hints_for("10.2.0.2", "Alfie")
    assert "VPN appears to be capturing traffic" in text
    assert "allow LAN" in text


def test_right_subnet_points_at_the_robot_instead_of_the_network() -> None:
    text = hints_for("192.168.4.2")
    assert "right subnet" in text
    assert "still be booting" in text
    # No point telling someone to join a network they are already on.
    assert AP_PASSWORD not in text


def test_no_route_at_all_is_reported() -> None:
    assert "No route to 192.168.4.1" in hints_for(None)


def test_vpn_caveat_is_mentioned_even_without_a_tunnel_source() -> None:
    assert "kill" in hints_for("192.168.178.98").lower()


def test_connect_error_includes_the_diagnosis() -> None:
    from robodog.backends.http import HttpBackend
    from robodog.errors import BackendError

    backend = HttpBackend("127.0.0.1:9", timeout=0.3)
    with pytest.raises(BackendError) as excinfo:
        backend.connect()
    message = str(excinfo.value)
    assert "cannot reach robot" in message
    # And it tells the operator what to actually do about it.
    assert AP_SSID in message or "right subnet" in message
