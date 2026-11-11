"""Windows 经典蓝牙发现与主动配对。

HC-05 是经典蓝牙 SPP 从机：未连接时处于可被发现状态，配对后 Windows 会
为它创建虚拟串口。本模块用 ctypes 调用 bthprops.cpl，扫描附近设备并发送
PIN 完成配对；不依赖 pybluez 等需要编译的扩展。

只支持 Windows 经典蓝牙。BLE 设备不能提供 SPP 串口，不在这里处理。
"""

from __future__ import annotations

import ctypes
import sys
from contextlib import contextmanager
from ctypes import wintypes
from dataclasses import dataclass
from typing import Iterator, List, Optional

ERROR_SUCCESS = 0
ERROR_ALREADY_EXISTS = 183
ERROR_NO_MORE_ITEMS = 259
ERROR_NOT_FOUND = 1168
BLUETOOTH_SERVICE_ENABLE = 0x00000001

# {00001101-0000-1000-8000-00805F9B34FB}
_SPP_GUID_DATA = (0x00001101, 0x0000, 0x1000, (0x80, 0x00, 0x00, 0x80, 0x5F, 0x9B, 0x34, 0xFB))


class BluetoothUnavailableError(RuntimeError):
    """当前系统没有可用的经典蓝牙栈。"""


class BluetoothError(RuntimeError):
    """蓝牙 API 调用失败。"""


class BluetoothPairingError(BluetoothError):
    """配对请求被拒绝或失败。"""

    def __init__(self, code: int, message: str) -> None:
        super().__init__(message)
        self.code = code


@dataclass(frozen=True)
class BluetoothDevice:
    """扫描到的一个经典蓝牙设备。"""

    address: int
    name: str
    connected: bool = False
    remembered: bool = False
    authenticated: bool = False
    class_of_device: int = 0

    @property
    def address_text(self) -> str:
        raw = "%012X" % (self.address & 0xFFFFFFFFFFFF)
        return ":".join(raw[index : index + 2] for index in range(0, 12, 2))

    @property
    def display_name(self) -> str:
        return self.name.strip() or self.address_text

    @property
    def paired(self) -> bool:
        return self.authenticated or self.remembered


class _SYSTEMTIME(ctypes.Structure):
    _fields_ = [
        ("wYear", wintypes.WORD),
        ("wMonth", wintypes.WORD),
        ("wDayOfWeek", wintypes.WORD),
        ("wDay", wintypes.WORD),
        ("wHour", wintypes.WORD),
        ("wMinute", wintypes.WORD),
        ("wSecond", wintypes.WORD),
        ("wMilliseconds", wintypes.WORD),
    ]


class _BLUETOOTH_ADDRESS(ctypes.Union):
    _fields_ = [
        ("ullLong", ctypes.c_ulonglong),
        ("rgBytes", ctypes.c_ubyte * 6),
    ]


class _BLUETOOTH_DEVICE_INFO(ctypes.Structure):
    _fields_ = [
        ("dwSize", wintypes.DWORD),
        ("Address", _BLUETOOTH_ADDRESS),
        ("ulClassofDevice", wintypes.DWORD),
        ("fConnected", wintypes.BOOL),
        ("fRemembered", wintypes.BOOL),
        ("fAuthenticated", wintypes.BOOL),
        ("stLastSeen", _SYSTEMTIME),
        ("stLastUsed", _SYSTEMTIME),
        ("szName", wintypes.WCHAR * 248),
    ]


class _BLUETOOTH_DEVICE_SEARCH_PARAMS(ctypes.Structure):
    _fields_ = [
        ("dwSize", wintypes.DWORD),
        ("fReturnAuthenticated", wintypes.BOOL),
        ("fReturnRemembered", wintypes.BOOL),
        ("fReturnUnknown", wintypes.BOOL),
        ("fReturnConnected", wintypes.BOOL),
        ("fIssueInquiry", wintypes.BOOL),
        ("cTimeoutMultiplier", ctypes.c_ubyte),
        ("hRadio", wintypes.HANDLE),
    ]


class _BLUETOOTH_FIND_RADIO_PARAMS(ctypes.Structure):
    _fields_ = [("dwSize", wintypes.DWORD)]


class _GUID(ctypes.Structure):
    _fields_ = [
        ("Data1", wintypes.DWORD),
        ("Data2", wintypes.WORD),
        ("Data3", wintypes.WORD),
        ("Data4", ctypes.c_ubyte * 8),
    ]


_SPP_GUID = _GUID(
    _SPP_GUID_DATA[0],
    _SPP_GUID_DATA[1],
    _SPP_GUID_DATA[2],
    (ctypes.c_ubyte * 8)(*_SPP_GUID_DATA[3]),
)


def _format_error(code: int) -> str:
    """把 Win32 错误码或 HRESULT 变成可读文本。"""

    if code == ERROR_SUCCESS:
        return "成功"
    win32 = code & 0xFFFF if code > 0xFFFF else code
    try:
        text = ctypes.FormatError(win32).strip()
    except (OSError, ValueError):
        text = ""
    if text:
        return "0x%08X %s" % (code & 0xFFFFFFFF, text)
    return "0x%08X" % (code & 0xFFFFFFFF)


def _load_library() -> ctypes.WinDLL:
    if sys.platform != "win32":
        raise BluetoothUnavailableError("主动扫描/配对只在 Windows 上可用")
    try:
        lib = ctypes.WinDLL("bthprops.cpl", use_last_error=True)
    except OSError as exc:
        raise BluetoothUnavailableError("无法加载 Windows 蓝牙 API: %s" % exc) from exc
    return lib


def _bind(lib: ctypes.WinDLL) -> None:
    lib.BluetoothFindFirstDevice.argtypes = [
        ctypes.POINTER(_BLUETOOTH_DEVICE_SEARCH_PARAMS),
        ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
    ]
    lib.BluetoothFindFirstDevice.restype = wintypes.HANDLE
    lib.BluetoothFindNextDevice.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
    ]
    lib.BluetoothFindNextDevice.restype = wintypes.BOOL
    lib.BluetoothFindDeviceClose.argtypes = [wintypes.HANDLE]
    lib.BluetoothFindDeviceClose.restype = wintypes.BOOL

    lib.BluetoothFindFirstRadio.argtypes = [
        ctypes.POINTER(_BLUETOOTH_FIND_RADIO_PARAMS),
        ctypes.POINTER(wintypes.HANDLE),
    ]
    lib.BluetoothFindFirstRadio.restype = wintypes.HANDLE
    lib.BluetoothFindRadioClose.argtypes = [wintypes.HANDLE]
    lib.BluetoothFindRadioClose.restype = wintypes.BOOL

    lib.BluetoothAuthenticateDevice.argtypes = [
        wintypes.HWND,
        wintypes.HANDLE,
        ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
        wintypes.LPWSTR,
        wintypes.DWORD,
    ]
    lib.BluetoothAuthenticateDevice.restype = wintypes.DWORD

    lib.BluetoothGetDeviceInfo.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
    ]
    lib.BluetoothGetDeviceInfo.restype = wintypes.DWORD

    lib.BluetoothSetServiceState.argtypes = [
        wintypes.HANDLE,
        ctypes.POINTER(_BLUETOOTH_DEVICE_INFO),
        ctypes.POINTER(_GUID),
        wintypes.DWORD,
    ]
    lib.BluetoothSetServiceState.restype = wintypes.DWORD


@contextmanager
def _radio_handle(lib: ctypes.WinDLL) -> Iterator[Optional[int]]:
    """打开第一个本地蓝牙电台；没有电台时返回 None。"""

    params = _BLUETOOTH_FIND_RADIO_PARAMS(ctypes.sizeof(_BLUETOOTH_FIND_RADIO_PARAMS))
    radio = wintypes.HANDLE()
    find = lib.BluetoothFindFirstRadio(ctypes.byref(params), ctypes.byref(radio))
    if not find:
        yield None
        return
    try:
        yield radio.value
    finally:
        lib.BluetoothFindRadioClose(find)


def _empty_device_info() -> _BLUETOOTH_DEVICE_INFO:
    info = _BLUETOOTH_DEVICE_INFO()
    info.dwSize = ctypes.sizeof(_BLUETOOTH_DEVICE_INFO)
    return info


def _device_from_info(info: _BLUETOOTH_DEVICE_INFO) -> BluetoothDevice:
    return BluetoothDevice(
        address=int(info.Address.ullLong),
        name=str(info.szName or "").strip(),
        connected=bool(info.fConnected),
        remembered=bool(info.fRemembered),
        authenticated=bool(info.fAuthenticated),
        class_of_device=int(info.ulClassofDevice),
    )


def scan_nearby_devices(timeout_multiplier: int = 6) -> List[BluetoothDevice]:
    """发起一次经典蓝牙 inquiry，返回附近设备。

    timeout_multiplier 的单位约为 1.28 秒；默认 6 约 8 秒。Windows 会同时
    返回已配对/已记住的设备，因此即使 HC-05 当前被别的设备连着，也还能看到。
    """

    if not 1 <= timeout_multiplier <= 48:
        raise ValueError("timeout_multiplier 必须在 1..48 之间")
    lib = _load_library()
    _bind(lib)

    params = _BLUETOOTH_DEVICE_SEARCH_PARAMS()
    params.dwSize = ctypes.sizeof(_BLUETOOTH_DEVICE_SEARCH_PARAMS)
    params.fReturnAuthenticated = True
    params.fReturnRemembered = True
    params.fReturnUnknown = True
    params.fReturnConnected = True
    params.fIssueInquiry = True
    params.cTimeoutMultiplier = timeout_multiplier
    params.hRadio = None

    info = _empty_device_info()
    handle = lib.BluetoothFindFirstDevice(ctypes.byref(params), ctypes.byref(info))
    if not handle:
        error = ctypes.get_last_error()
        if error in (ERROR_NO_MORE_ITEMS, ERROR_NOT_FOUND):
            return []
        raise BluetoothError("扫描蓝牙失败: %s" % _format_error(error))

    found = {}
    try:
        while True:
            device = _device_from_info(info)
            if device.address:
                previous = found.get(device.address)
                if previous is None or (not previous.name and device.name):
                    found[device.address] = device
            info = _empty_device_info()
            if not lib.BluetoothFindNextDevice(handle, ctypes.byref(info)):
                break
    finally:
        lib.BluetoothFindDeviceClose(handle)

    return sorted(
        found.values(),
        key=lambda item: (not item.connected, not item.paired, item.display_name.casefold()),
    )


def _info_for_device(device: BluetoothDevice) -> _BLUETOOTH_DEVICE_INFO:
    info = _empty_device_info()
    info.Address.ullLong = device.address & 0xFFFFFFFFFFFF
    info.szName = device.name[:247]
    return info


def pair_device(device: BluetoothDevice, pin: str = "1234") -> None:
    """用 PIN 主动配对，并尝试启用 SPP 服务以生成虚拟串口。

    Windows 的配对对话框可能短暂出现在桌面上；成功返回后串口不一定立刻
    出现，调用方应轮询 discover_bluetooth_ports()。
    """

    pin = pin or ""
    if not pin:
        raise ValueError("PIN 不能为空")
    if len(pin) > 16:
        raise ValueError("PIN 最长 16 位")

    lib = _load_library()
    _bind(lib)
    info = _info_for_device(device)
    with _radio_handle(lib) as radio:
        if radio is None:
            raise BluetoothUnavailableError("没有可用的蓝牙电台")
        code = lib.BluetoothAuthenticateDevice(
            None,
            radio,
            ctypes.byref(info),
            ctypes.c_wchar_p(pin),
            len(pin),
        )
    if code not in (ERROR_SUCCESS, ERROR_ALREADY_EXISTS):
        raise BluetoothPairingError(
            int(code),
            "配对 %s 失败: %s" % (device.display_name, _format_error(int(code))),
        )

    # 配对成功后再启用 SPP；某些 Windows 版本会在这里创建 COM 口。
    # 失败不当作配对失败，调用方还可以在系统设置里手动添加端口。
    info.fAuthenticated = True
    info.fRemembered = True
    with _radio_handle(lib) as radio:
        if radio is not None:
            # 用系统里的真实记录（包含刚配对的标志）启用 SPP，兼容性更好。
            actual = _empty_device_info()
            actual.Address.ullLong = info.Address.ullLong
            if lib.BluetoothGetDeviceInfo(radio, ctypes.byref(actual)) == ERROR_SUCCESS:
                info = actual
                info.fAuthenticated = True
                info.fRemembered = True
            lib.BluetoothSetServiceState(
                radio,
                ctypes.byref(info),
                ctypes.byref(_SPP_GUID),
                BLUETOOTH_SERVICE_ENABLE,
            )


def format_device_label(device: BluetoothDevice, port_label: str = "") -> str:
    """下拉框里的显示文本；已生成串口时优先用 HC-05 (COM5)。"""

    if port_label:
        return port_label
    if device.connected:
        status = "已连接"
    elif device.authenticated:
        status = "已配对"
    elif device.remembered:
        status = "已记住"
    else:
        status = "未配对"
    if device.name.strip():
        return "%s [%s] (%s)" % (device.display_name, device.address_text, status)
    return "%s (%s)" % (device.address_text, status)


def device_sort_key(device: BluetoothDevice) -> tuple:
    return (not device.connected, not device.paired, device.display_name.casefold())
