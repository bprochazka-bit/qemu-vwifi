#!/usr/bin/env python3
"""
Debian qcow2 Image Builder
===========================
Builds customizable Debian qcow2 images using debootstrap + chroot.

Requirements (host packages):
    apt install qemu-utils debootstrap parted kpartx e2fsprogs
    Kernel module: nbd (for --chroot mode; loaded automatically)

Usage:
    sudo python3 build_debian_qcow2.py                         # Interactive (saves recipe)
    sudo python3 build_debian_qcow2.py --recipe web-server.json # Build from recipe
    sudo python3 build_debian_qcow2.py --recipe web-server.json --output override.qcow2
    sudo python3 build_debian_qcow2.py --chroot myimage.qcow2  # Chroot into existing image
    sudo python3 build_debian_qcow2.py --show-recipe web.json   # Print recipe without building

Recipe JSON format:
    {
        "_meta": { "version": 1, "created": "...", "description": "..." },
        "image":   { "output": "...", "disk_size": "8G", "fs_type": "ext4" },
        "debian":  { "suite": "bookworm", "arch": "amd64", "mirror": "..." },
        "system":  { "hostname": "...", "root_password": "...", ... },
        "user":    { "username": "", "password": "" },
        "packages": { "base": [...], "extra": [...] },
        "options":  { "enable_serial": true, "enable_ssh": true, "permit_root_ssh": true }
    }
"""

import argparse
import json
import os
import sys
import subprocess
import shutil
import tempfile
import time
import signal
import atexit
import readline  # noqa: F401 — enables input() line editing
from datetime import datetime, timezone

# ---------------------------------------------------------------------------
# Constants & schema version
# ---------------------------------------------------------------------------
RECIPE_VERSION = 1

SUITE_CHOICES = ["bullseye", "bookworm", "trixie", "sid"]
ARCH_CHOICES  = ["amd64", "arm64", "i386"]
FS_CHOICES    = ["ext4", "xfs", "btrfs"]

DEFAULT_BASE_PACKAGES = [
    "openssh-server", "sudo", "curl", "wget", "vim", "net-tools",
    "ca-certificates", "locales", "systemd-sysv",
    "linux-image-amd64", "grub-pc",
]

DEFAULT_RECIPE = {
    "_meta": {
        "version": RECIPE_VERSION,
        "created": "",
        "description": "",
    },
    "image": {
        "output":    "debian-custom.qcow2",
        "disk_size": "8G",
        "fs_type":   "ext4",
    },
    "debian": {
        "suite":  "bookworm",
        "arch":   "amd64",
        "mirror": "http://deb.debian.org/debian",
    },
    "system": {
        "hostname":      "debian",
        "root_password": "changeme",
        "locale":        "en_US.UTF-8",
        "timezone":      "UTC",
        "dns":           "1.1.1.1",
    },
    "user": {
        "username": "",
        "password": "",
    },
    "packages": {
        "base":  list(DEFAULT_BASE_PACKAGES),
        "extra": [],
    },
    "options": {
        "enable_serial":    True,
        "enable_ssh":       True,
        "permit_root_ssh":  True,
    },
}

# ---------------------------------------------------------------------------
# Globals for cleanup tracking
# ---------------------------------------------------------------------------
_cleanup_stack = []  # list of (description, callable) for teardown


def _run(cmd, check=True, capture=False, **kwargs):
    """Run a shell command with logging."""
    if isinstance(cmd, str):
        print(f"  >> {cmd}")
        return subprocess.run(cmd, shell=True, check=check,
                              capture_output=capture, text=True, **kwargs)
    else:
        print(f"  >> {' '.join(cmd)}")
        return subprocess.run(cmd, check=check,
                              capture_output=capture, text=True, **kwargs)


def cleanup():
    """Unwind the cleanup stack (unmount, detach, etc.)."""
    print("\n[*] Cleaning up...")
    for desc, fn in reversed(_cleanup_stack):
        try:
            print(f"  -> {desc}")
            fn()
        except Exception as e:
            print(f"  !! {desc} failed: {e}")
    _cleanup_stack.clear()


def _signal_handler(sig, frame):
    cleanup()
    sys.exit(1)


signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)
atexit.register(cleanup)

# ---------------------------------------------------------------------------
# Recipe I/O
# ---------------------------------------------------------------------------

def recipe_to_flat(recipe):
    """Convert structured recipe dict to flat params dict for build functions."""
    return {
        "output":          recipe["image"]["output"],
        "disk_size":       recipe["image"]["disk_size"],
        "fs_type":         recipe["image"]["fs_type"],
        "suite":           recipe["debian"]["suite"],
        "arch":            recipe["debian"]["arch"],
        "mirror":          recipe["debian"]["mirror"],
        "hostname":        recipe["system"]["hostname"],
        "root_password":   recipe["system"]["root_password"],
        "locale":          recipe["system"]["locale"],
        "timezone":        recipe["system"]["timezone"],
        "dns":             recipe["system"]["dns"],
        "username":        recipe["user"]["username"],
        "user_password":   recipe["user"]["password"],
        "packages":        ",".join(recipe["packages"]["base"]),
        "extra_packages":  ",".join(recipe["packages"]["extra"]),
        "enable_serial":   recipe["options"]["enable_serial"],
        "enable_ssh":      recipe["options"]["enable_ssh"],
        "permit_root_ssh": recipe["options"]["permit_root_ssh"],
    }


def flat_to_recipe(p, description=""):
    """Convert flat params dict back to structured recipe."""
    def split_pkg(s):
        """Split comma-separated package string into sorted list."""
        if not s:
            return []
        return [pkg.strip() for pkg in s.split(",") if pkg.strip()]

    return {
        "_meta": {
            "version":     RECIPE_VERSION,
            "created":     datetime.now(timezone.utc).isoformat(),
            "description": description,
        },
        "image": {
            "output":    p["output"],
            "disk_size": p["disk_size"],
            "fs_type":   p["fs_type"],
        },
        "debian": {
            "suite":  p["suite"],
            "arch":   p["arch"],
            "mirror": p["mirror"],
        },
        "system": {
            "hostname":      p["hostname"],
            "root_password": p["root_password"],
            "locale":        p["locale"],
            "timezone":      p["timezone"],
            "dns":           p["dns"],
        },
        "user": {
            "username": p.get("username", ""),
            "password": p.get("user_password", ""),
        },
        "packages": {
            "base":  split_pkg(p["packages"]),
            "extra": split_pkg(p.get("extra_packages", "")),
        },
        "options": {
            "enable_serial":    p.get("enable_serial", True),
            "enable_ssh":       p.get("enable_ssh", True),
            "permit_root_ssh":  p.get("permit_root_ssh", True),
        },
    }


def load_recipe(path):
    """Load and validate a recipe JSON file."""
    with open(path, "r") as f:
        recipe = json.load(f)

    ver = recipe.get("_meta", {}).get("version", 0)
    if ver != RECIPE_VERSION:
        print(f"[!] Warning: recipe version {ver} (expected {RECIPE_VERSION}). "
              "Attempting to load anyway...")

    # Merge with defaults so missing keys get filled in
    merged = json.loads(json.dumps(DEFAULT_RECIPE))  # deep copy
    for section in ["image", "debian", "system", "user", "packages", "options"]:
        if section in recipe:
            merged[section].update(recipe[section])
    merged["_meta"] = recipe.get("_meta", merged["_meta"])

    return merged


def save_recipe(recipe, path):
    """Save recipe to a JSON file."""
    with open(path, "w") as f:
        json.dump(recipe, f, indent=2)
    print(f"  ✓ Recipe saved: {path}")


def print_recipe(recipe):
    """Pretty-print a recipe with passwords masked."""
    display = json.loads(json.dumps(recipe))
    if display["system"].get("root_password"):
        display["system"]["root_password"] = "***"
    if display["user"].get("password"):
        display["user"]["password"] = "***"
    print(json.dumps(display, indent=2))


# ---------------------------------------------------------------------------
# Interactive prompts
# ---------------------------------------------------------------------------

def prompt(label, default, choices=None):
    """Prompt user for a value with a default."""
    hint = f" [{default}]" if default else ""
    if choices:
        hint += f"  ({'/'.join(choices)})"
    while True:
        val = input(f"  {label}{hint}: ").strip()
        if not val:
            return default
        if choices and val not in choices:
            print(f"    Invalid choice. Pick from: {', '.join(choices)}")
            continue
        return val


def prompt_bool(label, default=True):
    d = "Y/n" if default else "y/N"
    val = input(f"  {label} [{d}]: ").strip().lower()
    if not val:
        return default
    return val in ("y", "yes", "1", "true")


def prompt_packages(label, defaults):
    """Prompt for a package list, showing current list and allowing edits."""
    print(f"  Current {label}:")
    for i, pkg in enumerate(defaults):
        print(f"    {i+1:3d}. {pkg}")
    print()
    print(f"  Enter a comma-separated list to REPLACE, or:")
    print(f"    +pkg1,pkg2   to ADD packages")
    print(f"    -pkg1,pkg2   to REMOVE packages")
    print(f"    (blank)      to keep as-is")

    val = input(f"  {label}: ").strip()
    if not val:
        return list(defaults)

    if val.startswith("+"):
        additions = [p.strip() for p in val[1:].split(",") if p.strip()]
        result = list(defaults) + [p for p in additions if p not in defaults]
        print(f"    Added: {', '.join(additions)}")
        return result

    if val.startswith("-"):
        removals = {p.strip() for p in val[1:].split(",") if p.strip()}
        result = [p for p in defaults if p not in removals]
        removed = removals & set(defaults)
        not_found = removals - set(defaults)
        if removed:
            print(f"    Removed: {', '.join(removed)}")
        if not_found:
            print(f"    Not found (ignored): {', '.join(not_found)}")
        return result

    # Full replacement
    return [p.strip() for p in val.split(",") if p.strip()]


def gather_params():
    """Interactive parameter gathering. Returns (flat_params, recipe)."""
    d = DEFAULT_RECIPE

    print("=" * 60)
    print("  Debian qcow2 Image Builder — Configuration")
    print("=" * 60)

    p = {}

    print("\n── Image settings ──")
    p["output"]    = prompt("Output filename", d["image"]["output"])
    p["disk_size"] = prompt("Disk size", d["image"]["disk_size"])
    p["fs_type"]   = prompt("Filesystem", d["image"]["fs_type"], FS_CHOICES)

    print("\n── Debian settings ──")
    p["suite"]  = prompt("Suite/codename", d["debian"]["suite"], SUITE_CHOICES)
    p["arch"]   = prompt("Architecture", d["debian"]["arch"], ARCH_CHOICES)
    p["mirror"] = prompt("Mirror URL", d["debian"]["mirror"])

    print("\n── System settings ──")
    p["hostname"]      = prompt("Hostname", d["system"]["hostname"])
    p["root_password"] = prompt("Root password", d["system"]["root_password"])
    p["locale"]        = prompt("Locale", d["system"]["locale"])
    p["timezone"]      = prompt("Timezone", d["system"]["timezone"])
    p["dns"]           = prompt("DNS server", d["system"]["dns"])

    print("\n── Options ──")
    p["enable_serial"]   = prompt_bool("Enable serial console?", d["options"]["enable_serial"])
    p["enable_ssh"]      = prompt_bool("Enable SSH server?", d["options"]["enable_ssh"])
    p["permit_root_ssh"] = prompt_bool("Permit root SSH login?", d["options"]["permit_root_ssh"])

    print("\n── User settings (leave blank to skip) ──")
    p["username"] = prompt("Non-root username", d["user"]["username"])
    if p["username"]:
        p["user_password"] = prompt("User password", p["username"])
    else:
        p["user_password"] = ""

    print("\n── Base packages ──")
    base_pkgs = prompt_packages("base packages", d["packages"]["base"])
    p["packages"] = ",".join(base_pkgs)

    print("\n── Extra packages (installed via apt after debootstrap) ──")
    extra_pkgs = prompt_packages("extra packages", d["packages"]["extra"])
    p["extra_packages"] = ",".join(extra_pkgs)

    # Build the recipe
    description = prompt("Recipe description (optional)", "")
    recipe = flat_to_recipe(p, description)

    # Confirmation
    print("\n" + "=" * 60)
    print("  Configuration Summary")
    print("=" * 60)
    print_recipe(recipe)
    print()

    if not prompt_bool("Proceed with build?", True):
        # Still offer to save the recipe even if not building
        if prompt_bool("Save recipe anyway?", True):
            recipe_path = prompt("Recipe filename",
                                 p["output"].rsplit(".", 1)[0] + ".recipe.json")
            save_recipe(recipe, recipe_path)
        print("Aborted.")
        sys.exit(0)

    # Auto-save recipe
    recipe_path = p["output"].rsplit(".", 1)[0] + ".recipe.json"
    recipe_path = prompt("Save recipe as", recipe_path)
    if recipe_path:
        save_recipe(recipe, recipe_path)

    return p, recipe


# ---------------------------------------------------------------------------
# Image building
# ---------------------------------------------------------------------------

def check_prerequisites():
    """Verify required tools are present."""
    required = ["qemu-img", "qemu-nbd", "debootstrap", "parted", "kpartx", "mkfs.ext4", "chroot"]
    missing = [t for t in required if not shutil.which(t)]
    if missing:
        print(f"[!] Missing tools: {', '.join(missing)}")
        print("    Install with: apt install qemu-utils debootstrap parted kpartx e2fsprogs")
        sys.exit(1)
    if os.geteuid() != 0:
        print("[!] This script must be run as root (for mount/chroot).")
        sys.exit(1)


def create_raw_image(path, size):
    """Create a raw disk image, partition it, and return the loop device path."""
    raw_path = path + ".raw"
    print(f"\n[1/7] Creating raw image ({size})...")
    _run(f"qemu-img create -f raw {raw_path} {size}")

    print("[2/7] Partitioning...")
    _run(f"parted -s {raw_path} mklabel msdos")
    _run(f"parted -s {raw_path} mkpart primary ext4 1MiB 100%")
    _run(f"parted -s {raw_path} set 1 boot on")

    print("[2/7] Mapping partitions...")
    result = _run(f"kpartx -av {raw_path}", capture=True)
    _cleanup_stack.append(("detach kpartx", lambda: _run(f"kpartx -dv {raw_path}", check=False)))

    # Parse loop device — e.g. "add map loop0p1 ..."
    loop_part = None
    for line in result.stdout.splitlines():
        if "add map" in line:
            loop_part = "/dev/mapper/" + line.split()[2]
            break

    if not loop_part:
        print("[!] Failed to find mapped partition. kpartx output:")
        print(result.stdout)
        cleanup()
        sys.exit(1)

    return raw_path, loop_part


def format_and_mount(loop_part, fs_type, mountpoint):
    """Format the partition and mount it."""
    print(f"[3/7] Formatting as {fs_type}...")
    _run(f"mkfs.{fs_type} -q {loop_part}")

    print(f"[3/7] Mounting at {mountpoint}...")
    os.makedirs(mountpoint, exist_ok=True)
    _run(f"mount {loop_part} {mountpoint}")
    _cleanup_stack.append(("unmount rootfs", lambda: _run(f"umount -lf {mountpoint}", check=False)))


def run_debootstrap(mountpoint, suite, arch, mirror, packages):
    """Run debootstrap to install base system."""
    print(f"[4/7] Running debootstrap ({suite}/{arch})... (this takes a while)")
    pkg_str = packages.replace(" ", "")
    _run(f"debootstrap --arch={arch} --include={pkg_str} {suite} {mountpoint} {mirror}")


def mount_pseudo_fs(mountpoint):
    """Mount proc/sys/dev inside the chroot."""
    for fs in ["proc", "sys", "dev", "dev/pts"]:
        target = os.path.join(mountpoint, fs)
        os.makedirs(target, exist_ok=True)
        if fs == "proc":
            _run(f"mount -t proc proc {target}")
        elif fs == "sys":
            _run(f"mount -t sysfs sys {target}")
        elif fs == "dev":
            _run(f"mount --bind /dev {target}")
        elif fs == "dev/pts":
            _run(f"mount --bind /dev/pts {target}")
        _cleanup_stack.append((f"unmount {fs}", lambda t=target: _run(f"umount -lf {t}", check=False)))


def configure_chroot(mountpoint, params):
    """Configure the system inside the chroot."""
    print("[5/7] Configuring system in chroot...")

    mount_pseudo_fs(mountpoint)

    def chroot_run(cmd):
        _run(f"chroot {mountpoint} /bin/bash -c '{cmd}'")

    # ── fstab ──
    with open(f"{mountpoint}/etc/fstab", "w") as f:
        f.write(f"/dev/sda1  /  {params['fs_type']}  defaults,relatime  0  1\n")

    # ── hostname ──
    with open(f"{mountpoint}/etc/hostname", "w") as f:
        f.write(params["hostname"] + "\n")

    with open(f"{mountpoint}/etc/hosts", "w") as f:
        f.write(f"127.0.0.1  localhost\n127.0.1.1  {params['hostname']}\n")

    # ── DNS ──
    with open(f"{mountpoint}/etc/resolv.conf", "w") as f:
        f.write(f"nameserver {params['dns']}\n")

    # ── Locale ──
    locale = params["locale"]
    with open(f"{mountpoint}/etc/locale.gen", "a") as f:
        f.write(f"{locale} UTF-8\n")
    chroot_run("locale-gen")
    with open(f"{mountpoint}/etc/default/locale", "w") as f:
        f.write(f'LANG="{locale}"\n')

    # ── Timezone ──
    chroot_run(f"ln -sf /usr/share/zoneinfo/{params['timezone']} /etc/localtime")
    with open(f"{mountpoint}/etc/timezone", "w") as f:
        f.write(params["timezone"] + "\n")

    # ── Networking (basic DHCP via ifupdown) ──
    netcfg_dir = f"{mountpoint}/etc/network"
    os.makedirs(netcfg_dir, exist_ok=True)
    with open(f"{netcfg_dir}/interfaces", "w") as f:
        f.write("auto lo\niface lo inet loopback\n\n")
        f.write("auto ens3\niface ens3 inet dhcp\n")

    # ── Root password ──
    chroot_run(f"echo 'root:{params['root_password']}' | chpasswd")

    # ── Optional non-root user ──
    if params.get("username"):
        user = params["username"]
        chroot_run(f"useradd -m -s /bin/bash -G sudo {user}")
        chroot_run(f"echo '{user}:{params['user_password']}' | chpasswd")

    # ── Serial console ──
    if params.get("enable_serial"):
        chroot_run("systemctl enable serial-getty@ttyS0.service || true")

    # ── SSH ──
    if params.get("enable_ssh", True):
        chroot_run("systemctl enable ssh.service || true")
        sshd_cfg = f"{mountpoint}/etc/ssh/sshd_config"
        if os.path.exists(sshd_cfg) and params.get("permit_root_ssh", True):
            with open(sshd_cfg, "a") as f:
                f.write("\nPermitRootLogin yes\n")

    # ── Extra packages ──
    extra = params.get("extra_packages", "").strip()
    if extra:
        pkg_list = extra.replace(",", " ")
        chroot_run(f"apt-get update && apt-get install -y {pkg_list}")

    # ── GRUB defaults ──
    print("[5/7] Configuring bootloader (GRUB)...")
    grub_default = f"{mountpoint}/etc/default/grub"
    if os.path.exists(grub_default):
        with open(grub_default, "r") as f:
            content = f.read()
        if params.get("enable_serial"):
            content = content.replace(
                'GRUB_CMDLINE_LINUX_DEFAULT="quiet"',
                'GRUB_CMDLINE_LINUX_DEFAULT="quiet console=tty0 console=ttyS0,115200n8"'
            )
            content = content.replace(
                'GRUB_CMDLINE_LINUX=""',
                'GRUB_CMDLINE_LINUX="console=tty0 console=ttyS0,115200n8"'
            )
            if "GRUB_TERMINAL" not in content:
                content += '\nGRUB_TERMINAL="console serial"\n'
                content += 'GRUB_SERIAL_COMMAND="serial --speed=115200 --unit=0"\n'
        with open(grub_default, "w") as f:
            f.write(content)


def install_grub(mountpoint, raw_path, loop_part):
    """Install GRUB to the raw disk image MBR."""
    print("[6/7] Installing GRUB to MBR...")
    basename = os.path.basename(loop_part)  # e.g. loop0p1
    loop_dev = "/dev/" + basename.rstrip("0123456789").rstrip("p")  # /dev/loop0

    dev_loop = f"{mountpoint}/{loop_dev}"
    os.makedirs(os.path.dirname(dev_loop), exist_ok=True)

    def chroot_run(cmd):
        _run(f"chroot {mountpoint} /bin/bash -c '{cmd}'")

    chroot_run(f"grub-install --target=i386-pc --boot-directory=/boot {loop_dev} --force")
    chroot_run("update-grub")


def convert_to_qcow2(raw_path, qcow2_path):
    """Convert raw image to qcow2."""
    print(f"[7/7] Converting to qcow2: {qcow2_path}")
    _run(f"qemu-img convert -f raw -O qcow2 -c {raw_path} {qcow2_path}")
    size = os.path.getsize(qcow2_path) / (1024 * 1024)
    print(f"  Final image: {qcow2_path} ({size:.1f} MB)")


def build_image(params):
    """Main build pipeline."""
    check_prerequisites()

    output = params["output"]
    mountpoint = tempfile.mkdtemp(prefix="deb-build-")
    _cleanup_stack.append(("remove tmpdir", lambda: shutil.rmtree(mountpoint, ignore_errors=True)))

    raw_path, loop_part = create_raw_image(output, params["disk_size"])
    format_and_mount(loop_part, params["fs_type"], mountpoint)
    run_debootstrap(mountpoint, params["suite"], params["arch"],
                    params["mirror"], params["packages"])
    configure_chroot(mountpoint, params)
    install_grub(mountpoint, raw_path, loop_part)

    # Unmount everything
    print("\n[*] Unmounting...")
    mounts_to_remove = [item for item in _cleanup_stack if "unmount" in item[0]]
    for desc, fn in reversed(mounts_to_remove):
        try:
            fn()
            _cleanup_stack.remove((desc, fn))
        except Exception:
            pass

    convert_to_qcow2(raw_path, output)

    # Remove raw image
    os.remove(raw_path)

    # Remaining cleanup
    cleanup()

    print("\n" + "=" * 60)
    print(f"  ✓ Image ready: {output}")
    print(f"\n  Boot with:")
    if params.get("enable_serial"):
        print(f"    qemu-system-x86_64 -hda {output} -m 1024 -nographic")
    print(f"    qemu-system-x86_64 -hda {output} -m 1024 -enable-kvm")
    print(f"\n  Customize later:")
    print(f"    sudo python3 {sys.argv[0]} --chroot {output}")
    print("=" * 60)


# ---------------------------------------------------------------------------
# NBD helpers
# ---------------------------------------------------------------------------

def _find_free_nbd(max_devs=16):
    """Find an unused /dev/nbdN device, loading the module if needed."""
    # Ensure nbd module is loaded
    _run("modprobe nbd max_part=16", check=False)

    for i in range(max_devs):
        dev = f"/dev/nbd{i}"
        if not os.path.exists(dev):
            continue
        # A device is "free" if it has no size (not connected)
        size_path = f"/sys/block/nbd{i}/size"
        try:
            with open(size_path, "r") as f:
                size = int(f.read().strip())
            if size == 0:
                return dev
        except (FileNotFoundError, ValueError):
            # If sysfs entry is missing, assume available
            return dev

    return None


def _nbd_connect(qcow2_path):
    """Connect a qcow2 to an NBD device. Returns (nbd_dev, partition_dev)."""
    nbd_dev = _find_free_nbd()
    if not nbd_dev:
        print("[!] No free /dev/nbd* devices. Is the nbd module loaded?")
        print("    Try: modprobe nbd max_part=16")
        sys.exit(1)

    print(f"[*] Connecting {qcow2_path} → {nbd_dev}")
    _run(f"qemu-nbd --connect={nbd_dev} {qcow2_path}")
    _cleanup_stack.append((f"disconnect {nbd_dev}",
                           lambda d=nbd_dev: _run(f"qemu-nbd --disconnect {d}", check=False)))

    # Wait for partition device to appear
    _run(f"partprobe {nbd_dev}", check=False)
    _run("udevadm settle --timeout=5", check=False)

    # Try partition 1 first (most common), fall back to whole device
    part_dev = f"{nbd_dev}p1"
    if not os.path.exists(part_dev):
        # Some kernels are slow; give it a moment
        time.sleep(1)
        _run(f"partprobe {nbd_dev}", check=False)

    if not os.path.exists(part_dev):
        print(f"  [!] Partition {part_dev} not found, using whole device {nbd_dev}")
        part_dev = nbd_dev

    return nbd_dev, part_dev


# ---------------------------------------------------------------------------
# Chroot into existing image
# ---------------------------------------------------------------------------

def chroot_into_image(qcow2_path):
    """Mount an existing qcow2 via qemu-nbd and drop into a chroot shell."""
    check_prerequisites()

    if not os.path.exists(qcow2_path):
        print(f"[!] Image not found: {qcow2_path}")
        sys.exit(1)

    mountpoint = tempfile.mkdtemp(prefix="deb-chroot-")
    _cleanup_stack.append(("remove tmpdir", lambda: shutil.rmtree(mountpoint, ignore_errors=True)))

    # Connect qcow2 directly via NBD — no conversion needed
    nbd_dev, part_dev = _nbd_connect(os.path.abspath(qcow2_path))

    # Mount the partition
    print(f"[*] Mounting {part_dev} → {mountpoint}")
    _run(f"mount {part_dev} {mountpoint}")
    _cleanup_stack.append(("unmount rootfs", lambda: _run(f"umount -lf {mountpoint}", check=False)))

    # Pseudo-filesystems
    mount_pseudo_fs(mountpoint)

    # Copy resolv.conf for network access
    shutil.copy2("/etc/resolv.conf", f"{mountpoint}/etc/resolv.conf")

    print("\n" + "=" * 60)
    print("  Entering chroot — install packages, make changes, etc.")
    print("  Type 'exit' when done.")
    print(f"  Changes write directly to {qcow2_path} (via qemu-nbd).")
    print("=" * 60 + "\n")

    # Drop into interactive shell
    subprocess.run(f"chroot {mountpoint} /bin/bash", shell=True)

    # Clean unmount — changes are already written to qcow2 via nbd
    print("\n[*] Unmounting and disconnecting...")
    cleanup()

    print(f"\n  ✓ Changes saved to {qcow2_path}")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main():
    parser = argparse.ArgumentParser(
        description="Build or customize Debian qcow2 images",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""Examples:
  Interactive build (auto-saves recipe):
    sudo python3 %(prog)s

  Build from saved recipe:
    sudo python3 %(prog)s --recipe web-server.recipe.json

  Override output filename from recipe:
    sudo python3 %(prog)s --recipe web-server.recipe.json --output node2.qcow2

  Chroot into existing image:
    sudo python3 %(prog)s --chroot myimage.qcow2

  Inspect a recipe file:
    python3 %(prog)s --show-recipe web-server.recipe.json
"""
    )
    parser.add_argument("--recipe", metavar="FILE",
                        help="Build from a JSON recipe file (non-interactive)")
    parser.add_argument("--show-recipe", metavar="FILE",
                        help="Print a recipe file and exit (no root required)")
    parser.add_argument("--chroot", metavar="IMAGE",
                        help="Chroot into an existing qcow2 for customization")
    parser.add_argument("--output", metavar="FILE",
                        help="Override the output filename from recipe")
    parser.add_argument("--save-recipe", metavar="FILE",
                        help="When using --recipe, also save a copy of the "
                             "(potentially overridden) recipe to this path")

    args = parser.parse_args()

    # ── Show recipe ──
    if args.show_recipe:
        recipe = load_recipe(args.show_recipe)
        print_recipe(recipe)
        sys.exit(0)

    # ── Chroot mode ──
    if args.chroot:
        chroot_into_image(args.chroot)
        return

    # ── Recipe mode (non-interactive) ──
    if args.recipe:
        if not os.path.exists(args.recipe):
            print(f"[!] Recipe not found: {args.recipe}")
            sys.exit(1)

        print(f"[*] Loading recipe: {args.recipe}")
        recipe = load_recipe(args.recipe)
        params = recipe_to_flat(recipe)

        # Apply CLI overrides
        if args.output:
            params["output"] = args.output
            recipe["image"]["output"] = args.output

        print("\n  Building from recipe:")
        print_recipe(recipe)
        print()

        # Optionally save the (possibly overridden) recipe
        if args.save_recipe:
            recipe["_meta"]["created"] = datetime.now(timezone.utc).isoformat()
            save_recipe(recipe, args.save_recipe)

        build_image(params)
        return

    # ── Interactive mode ──
    params, recipe = gather_params()
    build_image(params)


if __name__ == "__main__":
    main()
