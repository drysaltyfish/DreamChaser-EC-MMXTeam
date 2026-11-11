"""蓝牙 SPP 串口发现。

Windows 会把已配对的经典蓝牙串口（例如 HC-05）暴露成虚拟 COM 口。
本模块只负责找到这些口，并尽量把远程设备名（HC-05）附加到显示标签上；
真正的协议通讯仍由 fusai_sdk.SerialTransport 完成。

没有蓝牙串口时返回空列表，调用方应提示用户先配对设备。
"""

from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from typing import Any, Dict, Iterable, List, Optional

COM_PATTERN = re.compile(r"\((COM\d+)\)", re.IGNORECASE)
RAW_COM_PATTERN = re.compile(r"^COM\d+$", re.IGNORECASE)

# 这些名字来自 Windows 的设备树，只是蓝牙链路的容器，不是用户认识的设备名。
_GENERIC_NAMES = (
    "bluetooth",
    "蓝牙",
    "standard serial over",
    "serial over bluetooth",
    "bluetooth link",
    "蓝牙链接",
    "rfcomm",
    "communications port",
    "通信端口",
)


@dataclass(frozen=True)
class BluetoothPort:
    """一个已配对的蓝牙串口。"""

    device: str
    label: str
    description: str
    remote_name: str = ""


def is_bluetooth_port(device: str, description: str, hwid: str) -> bool:
    """判断 pyserial 报告的端口是否来自蓝牙 SPP。"""

    text = " ".join((device, description, hwid)).lower()
    return any(
        token in text
        for token in ("bluetooth", "bthenum", "bthmodem", "rfcomm", "蓝牙")
    )


def format_port_label(device: str, description: str, remote_name: str = "") -> str:
    """生成下拉框里的显示文本，例如 HC-05 (COM5)。"""

    base = (remote_name or description or device).strip()
    base = re.sub(r"\s*\(COM\d+\)\s*$", "", base, flags=re.IGNORECASE)
    if not base or base.upper() == device.upper():
        return device
    return "%s (%s)" % (base, device)


def device_sort_key(device: str) -> Any:
    """让 COM2 排在 COM10 前面；非 Windows 名字按字典序。"""

    match = re.search(r"(\d+)$", device)
    if match:
        return (0, int(match.group(1)), device.lower())
    return (1, 0, device.lower())


def _meaningful_remote_name(names: Iterable[str]) -> str:
    for name in names:
        candidate = (name or "").strip()
        if not candidate:
            continue
        lowered = candidate.lower()
        if any(token in lowered for token in _GENERIC_NAMES):
            continue
        return candidate
    return ""


def _windows_remote_names() -> Dict[str, str]:
    """通过 PnP 设备树查询每个蓝牙 COM 口对应的远程设备名。

    查询失败（非 Windows、没有 PowerShell、权限不足）时返回空字典，
    上层会退回到 pyserial 的描述文本。
    """

    if sys.platform != "win32":
        return {}

    powershell = shutil.which("powershell") or shutil.which("pwsh")
    if not powershell:
        return {}

    script = r"""
$ErrorActionPreference = 'SilentlyContinue'
[Console]::OutputEncoding = [System.Text.Encoding]::UTF8
$result = @()
Get-PnpDevice -Class Ports | Where-Object { $_.FriendlyName -match 'Bluetooth|蓝牙' } | ForEach-Object {
    $portName = $_.FriendlyName
    $match = [regex]::Match($portName, '\((COM\d+)\)')
    if (-not $match.Success) { return }
    $com = $match.Groups[1].Value
    $names = @()
    $id = $_.InstanceId
    for ($level = 0; $level -lt 4 -and $id; $level++) {
        $device = Get-PnpDevice -InstanceId $id
        if ($device -and $device.FriendlyName) { $names += $device.FriendlyName }
        $parent = (Get-PnpDeviceProperty -InstanceId $id -KeyName 'DEVPKEY_Device_Parent').Data
        if (-not $parent) { break }
        $id = $parent
    }
    $result += [pscustomobject]@{ com = $com; names = $names }
}
$result | ConvertTo-Json -Compress -Depth 4
"""
    creationflags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
    try:
        completed = subprocess.run(
            [powershell, "-NoProfile", "-NonInteractive", "-Command", script],
            capture_output=True,
            text=True,
            encoding="utf-8",
            errors="replace",
            timeout=4.0,
            creationflags=creationflags,
            check=False,
        )
    except (OSError, subprocess.SubprocessError):
        return {}

    raw = (completed.stdout or "").strip().lstrip("\ufeff")
    if not raw:
        return {}
    try:
        payload = json.loads(raw)
    except json.JSONDecodeError:
        return {}

    entries = payload if isinstance(payload, list) else [payload]
    names_by_com: Dict[str, str] = {}
    for entry in entries:
        if not isinstance(entry, dict):
            continue
        com = str(entry.get("com") or "").strip().upper()
        names = entry.get("names") or []
        if not com or not isinstance(names, list):
            continue
        remote = _meaningful_remote_name(str(name) for name in names)
        if remote:
            names_by_com[com] = remote
    return names_by_com


def discover_bluetooth_ports() -> List[BluetoothPort]:
    """返回当前系统上可用的蓝牙串口，按 COM 编号排序。"""

    from serial.tools import list_ports

    ports = [
        port
        for port in list_ports.comports()
        if is_bluetooth_port(port.device, port.description or "", port.hwid or "")
    ]
    remote_names = _windows_remote_names() if ports else {}

    result: List[BluetoothPort] = []
    for port in ports:
        description = (port.description or "").strip()
        remote = remote_names.get(port.device.upper(), "")
        result.append(
            BluetoothPort(
                device=port.device,
                label=format_port_label(port.device, description, remote),
                description=description,
                remote_name=remote,
            )
        )
    result.sort(key=lambda item: device_sort_key(item.device))
    return result


def resolve_device(selection: str, ports: Optional[List[BluetoothPort]] = None) -> str:
    """把下拉框文本（标签或裸 COM 名）还原成串口设备名。"""

    value = (selection or "").strip()
    if not value:
        return ""
    for port in ports or []:
        if value == port.label or value.upper() == port.device.upper():
            return port.device
    match = COM_PATTERN.search(value)
    if match:
        return match.group(1).upper()
    if RAW_COM_PATTERN.match(value):
        return value.upper()
    return value
