"""经典蓝牙发现模块的纯逻辑测试（不访问硬件）。"""

from __future__ import annotations

import bt_discovery


def test_address_text_is_canonical() -> None:
    device = bt_discovery.BluetoothDevice(0xF072D180FBEB, "原神？启动！")
    assert device.address_text == "F0:72:D1:80:FB:EB"


def test_label_marks_pairing_state() -> None:
    paired = bt_discovery.BluetoothDevice(0x001122334455, "HC-05", authenticated=True)
    fresh = bt_discovery.BluetoothDevice(0x001122334455, "")
    assert bt_discovery.format_device_label(paired) == "HC-05 [00:11:22:33:44:55] (已配对)"
    assert bt_discovery.format_device_label(fresh) == "00:11:22:33:44:55 (未配对)"
    assert bt_discovery.format_device_label(fresh, "HC-05 (COM5)") == "HC-05 (COM5)"


def test_sort_puts_connected_first() -> None:
    devices = [
        bt_discovery.BluetoothDevice(1, "Z", authenticated=False),
        bt_discovery.BluetoothDevice(2, "A", connected=True),
    ]
    ordered = sorted(devices, key=bt_discovery.device_sort_key)
    assert [device.name for device in ordered] == ["A", "Z"]
