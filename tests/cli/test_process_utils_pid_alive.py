"""Tests for the cross-platform pid_alive probe (never kills on Windows)."""

from __future__ import annotations

import os
import sys

import pytest

from feagi.cli import process_utils
from feagi.cli.bv_process import BVProcessManager
from feagi.cli.feagi_process import FeagiProcessManager
from feagi.cli.process_utils import _pid_alive_windows, pid_alive


class FakeKernel32:
    def __init__(self, handle=1234, exit_code=259, last_error=0, exit_ok=True):
        self.handle = handle
        self.exit_code = exit_code
        self.last_error = last_error
        self.exit_ok = exit_ok
        self.closed = []

    def get_last_error(self):
        return self.last_error

    def OpenProcess(self, _access, _inherit, _pid):  # noqa: N802
        return self.handle

    def GetExitCodeProcess(self, _handle, code_ref):  # noqa: N802
        if not self.exit_ok:
            return 0
        code_ref._obj.value = self.exit_code
        return 1

    def CloseHandle(self, handle):  # noqa: N802
        self.closed.append(handle)
        return 1


@pytest.mark.parametrize("bad", [None, "12", True, 0, -1])
def test_invalid_inputs(bad):
    assert pid_alive(bad) is False


def test_current_process_alive():
    assert pid_alive(os.getpid()) is True


def test_windows_still_active():
    k = FakeKernel32(exit_code=259)
    assert _pid_alive_windows(42, k) is True
    assert k.closed == [1234]


def test_windows_exited():
    k = FakeKernel32(exit_code=0)
    assert _pid_alive_windows(42, k) is False
    assert k.closed == [1234]


def test_windows_exit_code_call_fails():
    k = FakeKernel32(exit_ok=False)
    assert _pid_alive_windows(42, k) is False
    assert k.closed == [1234]


def test_windows_access_denied_means_alive():
    assert _pid_alive_windows(42, FakeKernel32(handle=0, last_error=5)) is True


def test_windows_other_error_means_gone():
    assert _pid_alive_windows(42, FakeKernel32(handle=0, last_error=87)) is False


def test_windows_never_calls_os_kill(monkeypatch):
    def boom(*_args, **_kwargs):
        raise AssertionError("os.kill must not be called on Windows")

    monkeypatch.setattr(sys, "platform", "win32")
    monkeypatch.setattr(os, "kill", boom)
    monkeypatch.setattr(process_utils, "_pid_alive_windows", lambda _pid: True)
    assert pid_alive(4242) is True
    assert FeagiProcessManager._is_process_running(4242) is True
    assert BVProcessManager._is_process_running(4242) is True
