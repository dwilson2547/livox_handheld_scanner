# Linksys WUSB6300 V2 remained unavailable because Ubuntu was using the wrong Realtek driver family

**Date:** 2026-06-20  
**Component:** `ros2-livox-scanner` USB Wi-Fi stack — `rtl8812au` / `88XXau`, `rtl88x2bu`, `NetworkManager`  
**Severity:** Medium — the node had no usable Wi-Fi, which blocked package installs and remote setup until Ethernet or manual intervention was used

---

## Observed symptom

On the Ubuntu 22.04 scanner node, the Linksys USB Wi-Fi adapter was physically present in `lsusb`
as `13b1:0045`, but no usable wireless interface came up with the stock system. After installing
NetworkManager and an initial Realtek driver, the adapter only reached an **unavailable** state and
would not scan or connect.

Attempts to bring the interface up failed with:

```text
RTNETLINK answers: Operation not permitted
```

NetworkManager also reported:

```text
Couldn't initialize supplicant interface: ... wpa_supplicant couldn't grab this interface.
```

The node ultimately connected only after replacing the incorrect driver path and activating the
adapter with the correct `rtl88x2bu` DKMS module. Final connected state:

```text
GENERAL.STATE:100 (connected)
IP4.ADDRESS[1]:192.168.0.110/24
IP4.GATEWAY:192.168.0.1
```

---

## Root cause

### The adapter was misidentified as an `rtl8812au` device

The Linksys adapter on this node uses USB ID `13b1:0045`. The initial troubleshooting path assumed
it belonged to the `rtl8812au` family and installed both Ubuntu's `rtl8812au-dkms` package and a
newer `88XXau` out-of-tree variant. That assumption was wrong for this hardware revision.

The old `rtl8812au` source only recognized the original WUSB6300 ID:

```c
{USB_DEVICE(0x13B1, 0x003F), .driver_info = RTL8812}, /* Linksys - WUSB6300 */
```

By contrast, the maintained `88x2bu` driver explicitly supports the actual device on the node:

```c
{USB_DEVICE_AND_INTERFACE_INFO(0x13B1, 0x0045, 0xff, 0xff, 0xff), .driver_info = RTL8822B},
```

Because the wrong driver family was loaded first, the system created a nominal Wi-Fi device but it
could not be brought fully online.

### The wrong driver path left the interface in a broken-but-visible state

Once the incorrect Realtek module family was loaded, NetworkManager could see a Wi-Fi device, but
kernel operations to raise the interface failed and `wpa_supplicant` could not initialize it.

Kernel and NetworkManager errors matched that failure mode:

```text
platform-linux: do-change-link[3]: failure changing link: failure 1 (Operation not permitted)
Could not set interface ... flags (UP): Operation not permitted
Couldn't initialize supplicant interface: ... wpa_supplicant couldn't grab this interface.
```

This made the issue look like a permissions or supplicant ownership problem, but the underlying
cause was still the wrong driver binding for the adapter chipset.

---

## Troubleshooting steps taken

1. **Checked basic hardware visibility** — confirmed the node had no wireless interface in `ip link`,
   found no `nmcli` or `rfkill` installed initially, and identified the USB adapter in `lsusb` as
   `Linksys WUSB6300 V2` with ID `13b1:0045`.

2. **Installed base networking tools and tested the first driver hypothesis** — installed
   `network-manager`, `rfkill`, build prerequisites, and Ubuntu's `rtl8812au-dkms`; this ruled out
   missing userspace packages but did not produce a working interface.

3. **Inspected module aliases and kernel behavior** — verified the `rtl8812au` path did not natively
   cover `13b1:0045`, then tried a newer `88XXau` build that exposed an interface but still failed
   to bring it up; this ruled out "driver absent" and narrowed the problem to "wrong driver family".

4. **Matched the USB ID against maintained upstream support tables** — confirmed `13b1:0045` is
   supported by `morrownr/88x2bu-20210702` and not by the original `rtl8812au` mapping used first.

5. **Removed conflicting Realtek modules and installed the correct one** — removed the `rtl8812au`
   and `88XXau` DKMS installs, installed `rtl88x2bu` via DKMS, reloaded NetworkManager, and
   verified the new interface `wlxd8ec5e0233a0` could scan and connect.

---

## Fix

### `rtl88x2bu` DKMS driver — replaced the incorrect Realtek driver family

Removed the `rtl8812au` / `88XXau` driver path and installed the maintained `rtl88x2bu` driver that
explicitly supports USB ID `13b1:0045`.

Before:

```text
lsmod | grep -E '8812au|88XXau'
8812au ...
88XXau ...
nmcli device status
wlx00e04c3178a3  wifi  unavailable
```

After:

```text
lsmod | grep 88x2bu
88x2bu ...
nmcli device status
wlxd8ec5e0233a0  wifi  disconnected
```

This resolved the kernel/interface mismatch and restored normal Wi-Fi behavior.

### NetworkManager connection profile — connected the node to Wi-Fi

After the correct driver loaded, the node successfully scanned local SSIDs and connected to the
operator's network using NetworkManager.

After:

```text
nmcli device wifi list ifname wlxd8ec5e0233a0
Sanchez 2 ...

nmcli connection show --active
Sanchez 2:7e7b315d-3354-48db-b750-1a1299955caf:802-11-wireless:wlxd8ec5e0233a0
```

This left the box in a usable remote-management state over Wi-Fi while keeping Ethernet available
for the Livox Horizon network.

---

## Files changed

- `/etc/modprobe.d/88x2bu.conf` — installed driver options for the correct `rtl88x2bu` module
- `/usr/src/rtl88x2bu-5.13.1/*` — staged source used by DKMS for the supported adapter driver
- `NetworkManager connection profile (UUID 7e7b315d-3354-48db-b750-1a1299955caf)` — added the `Sanchez 2` wireless connection for `wlxd8ec5e0233a0`
