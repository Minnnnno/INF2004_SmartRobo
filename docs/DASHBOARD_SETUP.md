# Flash the robot and open the web dashboard (do it yourself)

Step-by-step for any teammate: build the firmware, flash the robot, and watch
it live in a browser with Start / Stop / E-STOP. No terminal monitoring and no
PuTTY needed once it is running.

**You need:** a Windows laptop with VS Code and the Raspberry Pi Pico extension
(the same setup you already use to build), the robot, and a USB cable.

How it fits together:

```
robot (Pico W) --Wi-Fi--> your laptop's hotspot --> Mosquitto (MQTT broker)
                                                        |
                         browser  <--  tools/web_dashboard.py
```

---

## 1. One-time laptop setup

Do this once per laptop.

1. **Install Mosquitto** (the MQTT broker): download the Windows 64-bit
   installer from https://mosquitto.org/download/ and install with the default
   options. It installs to `C:\Program Files\mosquitto\`.
2. **Install the Python MQTT library.** In a VS Code terminal:
   ```powershell
   pip install paho-mqtt
   ```
3. **Set up the hotspot.** Windows Settings → Network & internet → Mobile hotspot:
   - Name and password: anything. **The team should agree on one shared name
     and password**, so one flashed robot works with every laptop.
   - **Band: 2.4 GHz.** The Pico W cannot join 5 GHz.

## 2. Create your Wi-Fi settings file

The robot learns the hotspot name, password and broker address **at build
time**, so do this before building.

1. Copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h`, in the same
   `config/` folder.
2. Edit the three values:
   ```c
   #define WIFI_SSID        "your-hotspot-name"
   #define WIFI_PASSWORD    "your-hotspot-password"
   #define MQTT_BROKER_IP   "192.168.137.1"
   ```
   Leave `MQTT_BROKER_PORT` (1883) and `MQTT_CLIENT_ID` as they are.

`192.168.137.1` is the address Windows always gives the laptop that hosts the
hotspot. The example file says `192.168.1.10`; **that address will not work**,
so make sure you change it. To check yours, turn the hotspot on and run:

```powershell
Get-NetIPAddress -AddressFamily IPv4 | Where-Object IPAddress -like "192.168.137.*"
```

If it prints nothing, the hotspot is off, or Windows uses a different address.
In that case run `ipconfig` and look for the adapter that only appears while the
hotspot is on (usually "Local Area Connection* 2"), and use its IPv4 address in
all three places this guide uses `192.168.137.1`.

`config/wifi_secrets.h` is ignored by git on purpose. **Never commit it**; it
holds the hotspot password.

## 3. Build

1. Pull the latest `main`.
2. In VS Code press **Ctrl+Shift+B** ("Compile Project").
3. In the build output, check for this line:
   ```
   ROBO_NETWORK=AUTO: WIFI=cyw43 NET=lwip MQTT=ON
   ```
   If you see a yellow warning saying `config/wifi_secrets.h not found`
   instead, the image has **no Wi-Fi**: go back to step 2.

The output is `build/mtkernel_3.uf2`. It contains everything: motion, line
following, barcode, IMU/terrain, obstacle avoidance and Wi-Fi/MQTT.

## 4. Flash

1. **Disconnect motor power** (battery off).
2. Hold the **BOOTSEL** button on the Pico W and plug in the USB cable.
3. A drive called `RPI-RP2` appears. Drag `build/mtkernel_3.uf2` onto it.
4. The Pico reboots by itself and the drive disappears. That's it.

## 5. Every session: start everything

Order matters: **hotspot first**, then Mosquitto.

1. **Turn on the Mobile hotspot.**
2. **Start the broker.** Open a VS Code terminal in the project folder:
   ```powershell
   & "C:\Program Files\mosquitto\mosquitto.exe" -c mosquitto.conf -v
   ```
   It should print `Opening ipv4 listen socket on port 1883` and keep running.
   Leave this terminal open.
   - The first time, Windows Firewall may ask about `mosquitto.exe`: tick
     **Private networks** and click **Allow**. Without this the robot cannot
     connect.
3. **Start the dashboard server** in a **second** terminal:
   ```powershell
   python tools/web_dashboard.py --host 192.168.137.1
   ```
   Leave this open too. To keep a record of the run, add
   `--log logs/run.log --csv logs/frames.csv`.
4. **Open http://localhost:8080** in your browser. Anyone else on the hotspot
   (a phone, a teammate's laptop) can open `http://192.168.137.1:8080`.
5. **Place the robot on the track and power it.** It boots **stopped** and does
   not move until you press **START**. Keep it still and level for a moment at
   power-on (the IMU takes its level reference).

Within about 10 seconds the pills at the top should read **broker connected**,
**robot online** and **telemetry live**, the motion pill **STOPPED (held)**, and
the Link card should show frame layout **v3** with frames counting up (about 10
per second). When the button bar says "Robot is stopped and waiting", press
**START** and it follows the line.

To finish: power the robot off, then press Ctrl+C in both terminals.

## 6. Using the buttons

| Button | What it does |
|---|---|
| **START** | Makes the robot go: after power-on, and after every STOP. Only works when it is STOPPED, still, fault-free and with trusted encoders (greyed out until then). |
| **STOP** | Controlled stop: motors stop and the robot stays stopped until you press START. Nothing else on the robot (obstacle avoidance, line following) can restart it. |
| **E-STOP** | Emergency stop. Latches a fault that **START cannot clear**: you must **power-cycle the robot** (battery and USB off, then on). Only for real emergencies. |

The robot answering "queued" in the log means the command arrived, not that it
was carried out. Watch the Motion card for the actual result.

### Barcode route letters

On the day, type one letter in each of the **Left**, **Right**, **Straight** and **U-turn** boxes
in the **Line & barcode** card and press **Send route**. The log shows the robot's reply:
`robot route: ok …`, or `rejected …` (all four needed, each letter different; the old route stays).
The message is retained, so the robot gets it again after a reboot. Until a route arrives the
robot uses `app_program/subsystem3/route.h`. Turns happen at the junction after the code, the
U-turn at once, and straight ignores the junction.

Without the dashboard: `mosquitto_pub -h 192.168.137.1 -t robot/route -r -q 1 -m "L=B R=C S=D U=A"`.
To clear an old retained route: `mosquitto_pub -h 192.168.137.1 -t robot/route -r -n`.

## 7. Troubleshooting

The **Troubleshooting** card on the page lists what it thinks is wrong. Common cases:

| What you see | Cause and fix |
|---|---|
| "Dashboard cannot reach the MQTT broker" / broker `connecting` | Mosquitto is not running, or it quit. Check its terminal. `Cannot assign requested address` = the hotspot was off when you started it: turn the hotspot on and start Mosquitto again. `Address in use` = another Mosquitto is running: run `net stop mosquitto` in an **administrator** terminal, then start again. |
| "Robot has never connected to this broker" | The robot is not reaching the laptop. Check: hotspot name/password match `wifi_secrets.h` exactly; the hotspot is on **2.4 GHz**; `MQTT_BROKER_IP` is `192.168.137.1` (not the example's `192.168.1.10`); the firewall allows `mosquitto.exe`; you rebuilt **and** reflashed after editing `wifi_secrets.h`. The hotspot settings page lists connected devices: the Pico should appear there. |
| "Robot frames rejected … version" | The robot runs older firmware than your checkout. Pull `main`, rebuild and reflash. The robot and the dashboard must come from the same commit. |
| Motion **FAULT: ESTOP** | Someone pressed E-STOP. Power-cycle the robot. |
| Robot online but **READY** and not moving, "Line sensor not publishing" | The image has no line following (wrong or old UF2). Rebuild with Ctrl+Shift+B and flash `build/mtkernel_3.uf2`. |
| "Line sensor: publishing" but "On the line: no" | The IR sensors cannot see the line: check that the line is under the centre sensor and check the sensor height. |
| "Losing more than 5% of frames" | Weak Wi-Fi: keep the robot closer to the laptop. A few lost frames while connecting are normal. |
| Robot shows **offline** after being online | Wi-Fi dropped, or the Pico reset or lost power. Check the battery. |

**Still stuck?** Plug in USB and open the serial console (PuTTY or the VS Code
Serial Monitor, on the Pico's COM port). It prints the Wi-Fi and MQTT connection
attempts, faults, and after a crash a repeating `LAST CRASH: ...` line. That
usually shows where it fails. Mosquitto's own terminal also prints a
`New client connected from 192.168.137.x` line when the robot joins.

## 8. Try the dashboard without the robot

To see the page working (or check your Python setup) with no robot and no broker:

```powershell
python tools/web_dashboard.py --demo
```

Open http://localhost:8080: a simulated robot drives, crosses a hump, meets an
obstacle and gets lifted. Press Ctrl+C to stop.

## Mac users

This guide assumes Windows. A Mac cannot easily host a hotspot over Wi-Fi while
using Wi-Fi itself. Easiest: join a Windows teammate's hotspot and open
`http://192.168.137.1:8080` in your browser.
