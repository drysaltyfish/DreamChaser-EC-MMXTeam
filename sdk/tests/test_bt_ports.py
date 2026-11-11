"""蓝牙串口发现模块的纯逻辑测试。"""

from __future__ import annotations

import serial.tools.list_ports as list_ports

import bt_ports


class FakePort:
    def __init__(self, device: str, description: str, hwid: str) -> None:
        self.device = device
        self.description = description
        self.hwid = hwid


def test_bluetooth_port_detection() -> None:
    assert bt_ports.is_bluetooth_port(
        "COM5",
        "Standard Serial over Bluetooth link (COM5)",
        r"BTHENUM\{00001101-0000-1000-8000-00805F9B34FB}_LOCALMFG&0000",
    )
    assert bt_ports.is_bluetooth_port("COM6", "蓝牙链接上的标准串行 (COM6)", "")
    assert bt_ports.is_bluetooth_port("/dev/rfcomm0", "n/a", "")
    assert not bt_ports.is_bluetooth_port("COM7", "USB-SERIAL CH340 (COM7)", "USB VID:PID=1A86:7523")


def test_port_label_uses_remote_name_and_device() -> None:
    assert (
        bt_ports.format_port_label(
            "COM5", "Standard Serial over Bluetooth link (COM5)", "HC-05"
        )
        == "HC-05 (COM5)"
    )
    assert (
        bt_ports.format_port_label("COM6", "Standard Serial over Bluetooth link (COM6)")
        == "Standard Serial over Bluetooth link (COM6)"
    )
    assert bt_ports.format_port_label("/dev/rfcomm0", "n/a") == "n/a (/dev/rfcomm0)"


def test_device_sorting_is_numeric() -> None:
    devices = ["COM10", "COM2", "COM1"]
    assert sorted(devices, key=bt_ports.device_sort_key) == ["COM1", "COM2", "COM10"]


def test_resolve_device_accepts_label_and_raw_name() -> None:
    ports = [bt_ports.BluetoothPort("COM5", "HC-05 (COM5)", "Bluetooth", "HC-05")]
    assert bt_ports.resolve_device("HC-05 (COM5)", ports) == "COM5"
    assert bt_ports.resolve_device("COM5", ports) == "COM5"
    assert bt_ports.resolve_device("COM7", ports) == "COM7"
    assert bt_ports.resolve_device("", ports) == ""


def test_discover_filters_non_bluetooth_ports(monkeypatch) -> None:
    ports = [
        FakePort("COM5", "Standard Serial over Bluetooth link (COM5)", "BTHENUM\\foo"),
        FakePort("COM7", "USB-SERIAL CH340 (COM7)", "USB VID:PID=1A86:7523"),
    ]
    monkeypatch.setattr(list_ports, "comports", lambda: ports)
    monkeypatch.setattr(
        bt_ports, "_windows_port_details", lambda: {"COM5": ("HC-05", "00:11:22:33:44:55")}
    )

    found = bt_ports.discover_bluetooth_ports()

    assert [port.device for port in found] == ["COM5"]
    assert found[0].label == "HC-05 (COM5)"
    assert found[0].remote_name == "HC-05"
    assert found[0].address == "00:11:22:33:44:55"


def test_match_port_for_device_uses_address_then_name() -> None:
    ports = [
        bt_ports.BluetoothPort(
            "COM5",
            "HC-05 (COM5)",
            "Bluetooth",
            "HC-05",
            "F0:72:D1:80:FB:EB",
        )
    ]
    assert bt_ports.match_port_for_device("f0:72:d1:80:fb:eb", "", ports) is ports[0]
    assert bt_ports.match_port_for_device("", "hc-05", ports) is ports[0]
    assert bt_ports.match_port_for_device("11:22:33:44:55:66", "HC-06", ports) is None


def test_wait_for_device_port_falls_back_to_new_port(monkeypatch) -> None:
    ports = [
        bt_ports.BluetoothPort("COM5", "HC-05 (COM5)", "Bluetooth", "HC-05"),
        bt_ports.BluetoothPort("COM7", "HC-06 (COM7)", "Bluetooth", "HC-06"),
    ]
    monkeypatch.setattr(bt_ports, "discover_bluetooth_ports", lambda: ports)

    found = bt_ports.wait_for_device_port(
        "11:22:33:44:55:66", "HC-06", previous_devices=["COM5"], timeout=0
    )
    assert found is ports[1]
