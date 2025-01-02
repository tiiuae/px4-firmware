from abc import ABC, abstractmethod
import time
#Acroname lib
import brainstem
from brainstem.result import Result


class Device(ABC):
    def __init__(self, name):
        self.name = name

    @abstractmethod
    def reboot(self, port):
        pass

device_registry = {}

# Decorator for automated registartion
def register_device(device_type):
    def decorator(cls):
        device_registry[device_type] = cls
        return cls
    return decorator

@register_device("")
class DummyDevice(Device):
    def reboot(self, port):
        pass

@register_device("Acroname")
class AcronameDevice(Device):

    def turn_on(self, port):
        hub = brainstem.stem.USBHub3p()
        result = hub.discoverAndConnect(brainstem.link.Spec.USB)
        if result != Result.NO_ERROR:
            print(f"Unable to discover device {self.name}")
            return

        hub.usb.setPortEnable(port)
        hub.disconnect()

    def turn_off(self, port):
        hub = brainstem.stem.USBHub3p()
        result = hub.discoverAndConnect(brainstem.link.Spec.USB)
        if result != Result.NO_ERROR:
            print(f"Unable to discover device {self.name}")
            return

        hub.usb.setPortDisable(port)
        hub.disconnect()

    def reboot(self, port):
        print(f"Rebooting though {self.name} ...")
        self.turn_off(port)
        time.sleep(0.1)
        self.turn_on(port)

@register_device("Raspi")
class RaspiDevice(Device):
    def reboot(self, port):
        print(f"Rebooting though {self.name} ...")

class DeviceManager:
    def __init__(self):
        self.device = None

    def set_device(self, device_type):
        if device_type not in device_registry:
            self.device = None
            raise ValueError(f"Unknown device type: {device_type}")
        else:
             self.device = device_registry[device_type](device_type)

    def reboot_device(self, port):
        if self.device:
            self.device.reboot(port)
        else:
            print("No device for rebooting is set!")
