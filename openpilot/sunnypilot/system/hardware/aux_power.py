"""
Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other contributors.

This file is part of sunnypilot and is licensed under the MIT License.
See the LICENSE.md file in the root directory for more details.
"""
import subprocess

from openpilot.common.params import Params
from openpilot.common.hardware.usb import CHESTNUT_FW_VERSION, CHESTNUT_ROM_USB_IDS, CHESTNUT_USB_IDS
from openpilot.system.hardware.chestnut.flash import VBUS_PATH


class AuxPower:
  def __init__(self):
    self.params = Params()
    self.vbus_on: bool | None = None

  def _set_vbus(self, on: bool) -> None:
    if on == self.vbus_on:
      return
    subprocess.run(["sudo", "tee", VBUS_PATH], input=b"1" if on else b"0", stdout=subprocess.DEVNULL, check=False)
    self.vbus_on = on

  def _powersave(self) -> bool:
    try:
      return self.params.get_bool("AuxPowerSave")
    except Exception:
      return False

  def update(self, offroad: bool, usb_state: list[dict]) -> None:
    mismatch = any((d["vendorId"], d["productId"]) in CHESTNUT_USB_IDS + CHESTNUT_ROM_USB_IDS and
                   d["product"] != f"custom {CHESTNUT_FW_VERSION}-CLEAN" for d in usb_state)
    self._set_vbus((not offroad or mismatch) or not self._powersave())


aux_power = AuxPower()
