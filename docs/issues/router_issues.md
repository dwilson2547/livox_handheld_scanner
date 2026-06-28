# GL-SFT1200 Wireless WAN — Issues Encountered & How to Avoid Them

Chronological list of what broke during the STA/wwan bringup, what the symptom looked like, and the actual cause. The headline symptom throughout was the same — **wwan showing "device not present" / 0% / NO_DEVICE and never pulling a DHCP lease** — but it had three independent causes stacked on top of each other. Each had to be cleared in turn.

---

## Issue 1 — Red herring: regulatory domain / channel 149
**Theory at the time:** Home APs on channel 149 (U-NII-3, 5.745 GHz) wouldn't associate due to an unset/world regdomain.

**Reality:** Wrong. The logs showed the STA associating fine on 5745 MHz and completing the WPA2/CCMP 4-way handshake every time. Country was already `US`, 149 was legal and usable. Association was **never** the problem.

**Lesson:** Don't chase regdomain/channel theories when the log shows `CTRL-EVENT-CONNECTED` and a completed key negotiation. If it associates, the problem is downstream (L3/DHCP/device binding), not RF.

---

## Issue 2 — GL repeater daemon tearing down the link on a timer
**Symptom:** STA associated successfully, then dropped ~every 90 s with `CTRL-EVENT-DISCONNECTED ... reason=3 locally_generated=1`, re-associated, dropped again. udhcpc never had a stable window to complete a lease.

**Cause:** GL's stock `/usr/sbin/repeater` Lua daemon was running in auto mode:
```
repeater.@main[0].auto='1'
repeater.@main[0].scan_interval='60'
repeater.@main[0].switch_interval='30'
```
It scans/re-evaluates/"switches" the STA on a timer, tearing down and rebuilding wlan1 repeatedly. This is GL's GUI-driven repeater management fighting the manual LuCI/UCI config.

**Fix:**
```
/etc/init.d/repeater stop
/etc/init.d/repeater disable
uci set repeater.@main[0].auto='0'
uci commit repeater
```

**Lesson:** On GL firmware, manual STA config via LuCI/UCI and the stock repeater daemon are mutually exclusive. Pick one. If configuring by hand, kill the daemon **first** — before any other debugging — or the constant teardown masks every other symptom. Don't mix the two.

---

## Issue 3 (root cause) — STA wifi-iface missing `ifname`, so netifd never bound the device
**Symptom:** After the repeater daemon was gone and the link was stable, wwan *still* showed `NO_DEVICE` / `up: false`, and `ps | grep udhcpc` showed **only eth0.2** — no udhcpc on wlan1 at all. The DHCP client was never being launched.

**Cause:** netifd had no device attached to wwan. `ubus call network.wireless status` showed the radio1 STA section with `network: ["wwan"]` but **no `ifname` field** — compare to the radio0 AP section which had `"ifname": "wlan0"`. The kernel device `wlan1` existed and was associated (visible in `iwinfo`), but netifd's wireless subsystem never assigned the STA section an ifname, so it couldn't map wlan1 → wwan. No device → no L3 setup → no udhcpc → no lease.

This is a GL/siwifi (MediaTek) firmware bug. On stock OpenWrt the wifi-iface auto-populates ifname; here it intermittently doesn't, accompanied by `netifd: radio1: uci: Invalid argument` and `Command failed: Not found` during setup.

**Fix — the one line that solved everything:**
```
uci set wireless.cfg053579.ifname='wlan1'
uci commit wireless
wifi down; sleep 3; wifi up; sleep 12; ifup wwan
```
After this, `ps | grep udhcpc` showed the second client (`udhcpc ... -i wlan1`), wwan came up with a lease (192.168.0.119), and everything worked.

**Lesson:** If a STA associates and stays connected but wwan reports `NO_DEVICE` and no udhcpc launches for it, check `ubus call network.wireless status` and confirm the STA interface block has an `ifname`. If it's missing, pin it explicitly with `uci set wireless.<section>.ifname='wlan1'`. This is the fast path — don't waste time rebuilding the wwan interface or chasing firewall/DHCP-server theories first.

---

## Diagnostic sequence that actually isolates the problem
For next time, the efficient order to localize this class of failure:

1. **Does it associate?** `iwinfo wlan1 info` → ESSID + signal + CCMP. If yes, stop blaming RF/regdomain/channel.
2. **Is it stable or cycling?** `logread` → look for repeating `reason=3 locally_generated=1` teardowns. If cycling → kill the repeater daemon (Issue 2).
3. **Is udhcpc running for the STA?** `ps | grep udhcpc` → must show a line with `-i wlan1`. If only eth0.2 → it's a device-binding problem, not a DHCP-server/firewall problem.
4. **Does netifd have a device for wwan?** `ubus call network.interface.wwan status` → check for `l3_device` / `device`. `NO_DEVICE` = nothing bound.
5. **Why no device?** `ubus call network.wireless status` → STA section missing `ifname` → pin it (Issue 3).

The single most useful check is **#3 (`ps | grep udhcpc`)** — it instantly separates "DHCP isn't completing" (server/firewall/isolation problem) from "DHCP never started" (device-binding problem). This whole session would've been shorter by going there earlier.

---

## Things that were NOT the problem (don't re-investigate these)
- Regdomain / country code (was US, correct)
- Channel 149 / U-NII-3 legality (fine)
- Password / encryption mismatch (handshake always completed)
- `classlessroute` key on the wwan interface (rebuilt without it, didn't matter)
- dnsmasq on `0.0.0.0:67` (it listens broadly but doesn't serve on the wan-zone interface; not the cause)
- Firewall zone assignment (wwan was correctly in the wan zone from the start)
