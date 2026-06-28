# GL-SFT1200 Travel Router — Wireless WAN Setup (Scanner Rig)

## Goal
Run the SFT1200 as a self-contained network for the 3D scanner rig:
- **5 GHz radio (radio1 / wlan1):** STA / wireless WAN client → uplinks to home WiFi
- **2.4 GHz radio (radio0 / wlan0):** AP hosting the scanner network, bridged to LAN
- **LAN port:** OptiPlex wired client
- Scanner subnet stays isolated from the home subnet; NAT handles routing out.

## Final working topology
| Segment | Device | Network | Subnet |
|---|---|---|---|
| Uplink to home | wlan1 (STA, 5 GHz) | `wwan` (wan zone) | DHCP client, got 192.168.0.119/24 from home |
| Scanner AP | wlan0 (AP, 2.4 GHz) | `lan` (br-lan) | 192.168.8.x |
| Wired client | LAN port → OptiPlex | `lan` (br-lan) | 192.168.8.x |
| Router LAN gateway | br-lan | — | 192.168.8.1/24 |

Home network is `192.168.0.x`, router LAN is `192.168.8.x` — no collision.

## Setup steps (LuCI + UCI)

### 1. Install wwan support
`luci-proto-wwan` / wwan plugin installed via LuCI (already present on GL firmware in most cases).

### 2. Disable GL's repeater daemon
GL's stock `repeater` daemon fights LuCI/UCI for control of the STA and tears the link down on a timer. With manual UCI config it **must** be off:
```
/etc/init.d/repeater stop
/etc/init.d/repeater disable
uci set repeater.@main[0].auto='0'
uci commit repeater
```
Trade-off: the GL GUI "Repeater" page no longer manages the link. Drive everything from LuCI / UCI instead.

### 3. wwan interface — DHCP client in the wan zone
```
uci set network.wwan=interface
uci set network.wwan.proto='dhcp'
uci commit network
```
Keep it minimal — just `proto=dhcp`. Do **not** add extra keys like `classlessroute`.

Confirm wwan is in the wan firewall zone (it was, by default):
```
uci show firewall | grep -i network
# firewall.@zone[1].network='wan wan6 wwan'   ← wwan present in wan zone = correct
```

### 4. STA wifi-iface on the 5 GHz radio
Created via LuCI "Scan → Join Network" on radio1, then corrected via UCI. Final working config (`wireless.cfg053579`):
```
wireless.cfg053579.device='radio1'
wireless.cfg053579.mode='sta'
wireless.cfg053579.network='wwan'
wireless.cfg053579.ssid='Sanchez 2'
wireless.cfg053579.encryption='psk2'
wireless.cfg053579.key='<home PSK>'
wireless.cfg053579.ifname='wlan1'     ← THE CRITICAL LINE (see issues doc)
```
**The `ifname='wlan1'` assignment is mandatory.** Without it netifd never binds the device to wwan and DHCP never starts. This is the single fix that made the whole thing work.

```
uci set wireless.cfg053579.ifname='wlan1'
uci commit wireless
```

Notes:
- Dropped the hardcoded `bssid=` lock so the STA can associate to whichever home AP (both share SSID "Sanchez 2") is strongest.
- Country code US, channel auto, VHT40 — matches home AP on channel 149.

### 5. 2.4 GHz AP → scanner network on LAN bridge
radio0 default AP (`wireless.default_radio0`), `network='lan'`, bridged into br-lan. Rename SSID/key from the GL defaults:
```
uci set wireless.default_radio0.ssid='<scanner SSID>'
uci set wireless.default_radio0.key='<password>'
uci commit wireless
wifi reload
```

### 6. Bring it up (order matters)
```
wifi down
sleep 3
wifi up
sleep 12
ifup wwan
sleep 8
```

## Verification
```
# DHCP client running on wlan1 (should see a udhcpc line with -i wlan1):
ps | grep udhcpc

# wwan up with a lease:
ubus call network.interface.wwan status
#   → "up": true, "l3_device": "wlan1", ipv4-address present

# internet out the uplink:
ping -c 3 -I wlan1 8.8.8.8

# LAN subnet for the OptiPlex side:
ip addr show br-lan | grep inet
#   → 192.168.8.1/24
```

## Operational notes
- 5 GHz is fully consumed by the uplink, so the hosted AP is **2.4 GHz only**. Fine for control/telemetry to the scanner; keep the OptiPlex on the wired LAN port for any bulk point-cloud transfer.
- The `ifname` line is committed to `/etc/config/wireless` and survives reboot. If GL firmware regenerates wireless config (e.g. after a firmware update), re-check that `ifname='wlan1'` is still present — it's the first thing to break.
- Lan client was configured to static dhcp lease by mac address, pretty easy from the web interface. Client will always be at ip 192.168.8.50