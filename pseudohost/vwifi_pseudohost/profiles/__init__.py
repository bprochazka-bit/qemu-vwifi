#
# vwifi-pseudohost — device profiles
#
# SPDX-License-Identifier: GPL-2.0-or-later
#
# Each profile is a PseudoHost subclass describing one kind of device.
# They are deliberately thin: a persona is mostly a few fingerprint
# knobs (OS TTL, MAC OUI, hostname style) plus the list of services that
# device would run.  That thinness is the point — adding a new device
# type should be a dozen lines, not a new stack.
#
# The registry at the bottom lets the CLI pick a profile by name.
#
from .workstation import LinuxWorkstation, WindowsWorkstation
from .printer import NetworkPrinter
from .nas import NAS
from ..host import PseudoHost


class GenericHost(PseudoHost):
    """A plain host with no services beyond ICMP — the default."""
    persona = "generic"
    hostname = "pseudohost"


PROFILES = {
    "generic": GenericHost,
    "linux": LinuxWorkstation,
    "windows": WindowsWorkstation,
    "printer": NetworkPrinter,
    "nas": NAS,
}


def get_profile(name):
    return PROFILES.get(name)


def profile_names():
    return sorted(PROFILES)
