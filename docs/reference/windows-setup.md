# Vayu on Windows — fresh-machine setup (build · flash · telemetry)

Everything needed to take a **clean Windows 10/11 (x64)** machine to a working
Vayu workstation: cross-build the flight-controller firmware, flash it to the
board over ST-Link, monitor live telemetry over the ESP Wi-Fi bridge, and build
the **GCS** (the `Navigator` Qt6 desktop app — see §8).

Every step here was verified end-to-end (Windows 10 Pro 22H2 / 19045). The build
target is natively Linux (`tools/build.sh`); Windows needs a few extra pieces and
one CMake-invocation tweak, all covered below — none of it requires editing the
repo.

> Running Windows inside a Linux/KVM VM instead of on bare metal? Do everything
> below, then see **Appendix A** for the two VM-only extras (USB passthrough +
> a telemetry relay).

---

## 0. What you install

| Tool | Version (pinned) | Purpose |
|------|------------------|---------|
| Arm GNU toolchain (`arm-none-eabi-gcc`) | 15.2.1 (xPack) | cross compiler, `objcopy`, `size` |
| CMake | 4.3.3 | build-system generator |
| Ninja | 1.13.2 | build tool |
| Git (MinGit) | 2.54.0 | clone repo + CMake `FetchContent` of NavHAL |
| Python | 3.13.1 | NavHAL Kconfig + navlink codegen |
| pip: `kconfiglib`, `numpy` | latest | NavHAL build tooling |
| stlink | 1.8.0 | `st-flash` / `st-info` / `st-util` |
| libusb (x64 `libusb-1.0.dll`) | 1.0.30 | stlink runtime dep (missing from the stlink zip) |
| WinUSB driver (via Zadig) | — | binds the ST-Link so `st-flash` can use it |

Prerequisites: 64-bit Windows 10/11, an **administrator** shell, internet, ~5 GB free.

---

## 1. One-shot install

From an **elevated** PowerShell, run the bundled installer. It downloads pinned
releases into `C:\tools`, installs Python + pip deps, adds a `python3` shim,
supplies the x64 `libusb-1.0.dll`, installs the stlink chip DB, and updates the
**Machine PATH**:

```powershell
powershell -ExecutionPolicy Bypass -File tools\windows\install-vayu-build-deps.ps1
```

Then **open a new terminal** (PATH is read at shell start) and verify:

```powershell
arm-none-eabi-gcc --version
cmake --version
ninja --version
git --version
python --version ; python3 --version
st-flash --version
```

<details>
<summary>Manual alternative (what the script automates)</summary>

1. Unzip each release under `C:\tools\<name>\`:
   - arm — <https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases>
   - cmake — <https://github.com/Kitware/CMake/releases> (`*-windows-x86_64.zip`)
   - ninja — <https://github.com/ninja-build/ninja/releases> (`ninja-win.zip`)
   - stlink — <https://github.com/stlink-org/stlink/releases> (`*-win32.zip`)
   - git — <https://github.com/git-for-windows/git/releases> (`MinGit-*-64-bit.zip`)
2. Install Python from <https://www.python.org/downloads/windows/> with
   "Add python.exe to PATH" ticked, then
   `copy "C:\Program Files\Python313\python.exe" "...\python3.exe"` and
   `python -m pip install kconfiglib numpy`.
3. stlink runtime: drop a **64-bit** `libusb-1.0.dll`
   (from <https://github.com/libusb/libusb/releases>, `VS2022\MS64\dll\`) next to
   `st-flash.exe`, and copy the stlink `config\` (chips DB) to
   `C:\Program Files (x86)\stlink\config`.
4. Add each tool's `bin` dir to the **System** `Path`; open a new terminal.
</details>

---

## 2. ST-Link driver (one-time, GUI)

`st-flash` talks to the ST-Link through **libusb**, which needs the **WinUSB**
driver — Windows' default ST driver will not work. Install it with Zadig:

1. Download Zadig: <https://zadig.akeo.ie/>
2. Plug in the ST-Link. In Zadig: **Options → ✓ List All Devices**.
3. Select **`STM32 STLink`** (USB ID `0483 3748`).
4. Set the target driver to **`WinUSB`** → **Replace Driver**.
5. Confirm:

```powershell
st-info --probe
```

Expected:

```
Found 1 stlink programmers
  ...
  flash:      524288 (pagesize: 16384)
  chipid:     0x433
  dev-type:   STM32F401xD_xE
```

If `flash:` is `0`, the chip DB isn't where st-flash looks — re-run the installer
(it populates `C:\Program Files (x86)\stlink\config`).

---

## 3. Get the source

```powershell
git clone --recursive https://github.com/ragnar-vallhala/vayu.git C:\src\vayu
```

`vayu` is private — sign in when Git prompts (or use a PAT). The `extern/vaios`
submodule and NavHAL (fetched at configure time) are public.

---

## 4. Configure

The repo's `CMakeLists.txt` calls `project()` *before* it sets the cross
compiler, which works on Linux (a host `cc` exists) but not on a clean Windows
box. Pass the cross compiler as **cache variables** so they apply before
`project()`:

```powershell
cmake -S C:\src\vayu -B C:\src\vayu\build -G Ninja `
  -DCMAKE_SYSTEM_NAME=Generic `
  -DCMAKE_C_COMPILER=arm-none-eabi-gcc `
  -DCMAKE_ASM_COMPILER=arm-none-eabi-gcc `
  -DCMAKE_TRY_COMPILE_TARGET_TYPE=STATIC_LIBRARY `
  -DNAVHAL=ON -DEXTERNAL_LINKER=ON
```

A good configure ends with `navlink: 46 messages ... CRC-16 self-check OK` and
`Build files have been written to: C:/src/vayu/build`.

---

## 5. Build

```powershell
cmake --build C:\src\vayu\build
arm-none-eabi-objcopy -O binary C:\src\vayu\build\main C:\src\vayu\build\main.bin
arm-none-eabi-size C:\src\vayu\build\main
```

Expected: `[94/94] Linking C executable main`, then `main` (ELF) + `main.bin`
(~140 KB). The `LOAD segment with RWX permissions` linker warning is benign.

---

## 6. Flash

Same command the Linux `tools/flash.sh` uses:

```powershell
st-flash --connect-under-reset --reset write C:\src\vayu\build\main.bin 0x8000000
```

Success ends with `Flash written and verified! jolly good!`. The
`NRST is not connected` warning is fine — st-flash falls back to a software
(AIRCR) reset.

---

## 7. Telemetry over the ESP Wi-Fi bridge

The FC's telemetry reaches a ground station through the on-board **ESP8266**
bridge (`10.42.0.30:14555`, NavLink v2 UDP). The bridge **streams to the
sender's IP at fixed UDP port 14555** — i.e. send it one packet from port 14555
and it streams back to you.

1. Join the drone's Wi-Fi (the ESP's `10.42.0.x` network) or be on the same LAN.
2. Point the GCS / your client at **`10.42.0.30:14555`**.
3. Quick check (Python):

```python
import socket, time
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 14555))           # must be port 14555
s.sendto(b"\x00", ("10.42.0.30", 14555))
s.settimeout(1); n = 0; t = time.time()
while time.time() - t < 5:
    try: s.recvfrom(2048); n += 1
    except socket.timeout: pass
print("packets in 5s:", n)           # expect a few hundred
```

Healthy link ≈ 250 packets/s of `5602…`-framed NavLink data.

---

## 8. Building the GCS (Navigator desktop app)

The firmware steps above cross-compile for the board. The **GCS** (`navigator/`,
the `Navigator` Qt6 app) is instead a *native* Windows program — it needs a host
C++ compiler **and Qt6 + assimp**, none of which the Arm toolchain provides. The
cleanest fully-headless way to get all of them from one package manager is
**MSYS2** (UCRT64).

> **SITL / autotune is POSIX-only.** The in-app simulator and the autotune engine
> talk to a firmware host-sim over a pty (`termios`/`poll`/pthread scheduling),
> which MinGW doesn't provide. Build the GCS with `-DNAVIGATOR_SITL=OFF`: you get
> a full live-vehicle GCS (telemetry, calibration, commands, packet analyzer,
> replay, 3D attitude) — just without the Simulator and Autotune tabs.

### 8.1 Install MSYS2 + the toolchain

From an **elevated** PowerShell:

```powershell
# Download + silent-install MSYS2 into C:\msys64
$u="https://github.com/msys2/msys2-installer/releases/download/2025-08-30/msys2-x86_64-20250830.exe"
Invoke-WebRequest $u -OutFile C:\msys64-installer.exe
C:\msys64-installer.exe in --confirm-command --accept-messages --root C:/msys64
```

Then, from `C:\msys64\usr\bin\bash.exe`, update and install the build deps.
The first `pacman -Syuu` updates the core runtime and closes the shell — just run
it again:

```bash
pacman -Syuu --noconfirm     # run twice (first pass updates msys2-runtime)
pacman -Syuu --noconfirm
pacman -S --needed --noconfirm \
  mingw-w64-ucrt-x86_64-gcc mingw-w64-ucrt-x86_64-cmake \
  mingw-w64-ucrt-x86_64-ninja mingw-w64-ucrt-x86_64-qt6-base \
  mingw-w64-ucrt-x86_64-qt6-serialport mingw-w64-ucrt-x86_64-assimp \
  mingw-w64-ucrt-x86_64-python python
```

`qt6-base` covers Core/Widgets/Network/OpenGL/Concurrent; `qt6-serialport` adds
the serial link. `assimp` is only needed when SITL is ON.

### 8.2 Build

Everything below runs in the **UCRT64** environment (so the right gcc + Qt6 are
on `PATH`). From a normal `bash.exe`:

```bash
export MSYSTEM=UCRT64; source /etc/profile
cd /c/src/vayu/software
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DNAVIGATOR_SITL=OFF -DNAVIGATOR_BUILD_TESTS=OFF
cmake --build build
```

A good configure prints `navlink: 46 messages ... CRC-16 self-check OK` and
`Navigator: SITL OFF (NAVIGATOR_SITL=OFF) -- GCS-only build`. The build ends with
`Linking CXX executable Navigator.exe` (~2 MB). Deprecation warnings from Qt are
benign.

> Want the simulator/autotune too? Drop `-DNAVIGATOR_SITL=OFF` — but those
> sources won't compile under MinGW (see the POSIX note above). SITL builds are
> Linux/macOS only.

### 8.3 Make it run anywhere (deploy)

`Navigator.exe` needs its Qt + runtime DLLs next to it. MSYS2's `windeployqt6`
copies the Qt DLLs/plugins but **not** the MinGW C++ runtime or Qt's third-party
deps, so a bare deploy fails to start on a clean machine. The bundled script
finishes the job (runs `windeployqt6`, adds the runtime DLLs, then resolves the
full transitive DLL closure with `ldd`):

```bash
bash tools/windows/deploy-navigator-msys.sh
# -> navigator/build/dist/  (Navigator.exe + ~30 DLLs, fully self-contained)
```

Smoke-test it headless (no display) from a clean shell:

```bash
cd /c/src/vayu/navigator/build/dist
QT_QPA_PLATFORM=offscreen ./Navigator.exe   # event loop stays alive = OK
```

Ship the whole `dist\` folder, or run `dist\Navigator.exe` directly on the
desktop.

---

## Troubleshooting (every issue hit during bring-up)

| Symptom | Cause | Fix |
|---------|-------|-----|
| `No CMAKE_C_COMPILER could be found` | `project()` runs before the cross compiler is set; no host compiler on Windows | pass the `-DCMAKE_*_COMPILER` / `-DCMAKE_SYSTEM_NAME=Generic` cache vars (§4) |
| `Could NOT find Python3` | Python not installed | install Python (§1) |
| `ModuleNotFoundError: kconfiglib` | NavHAL tooling deps missing | `python -m pip install kconfiglib numpy` |
| `navlink codegen failed (rc=9009)` + "Python was not found … Microsoft Store" | bare `python3` hits the Store alias stub | add `python3.exe` next to real `python.exe` (installer does this) |
| `st-info`/`st-flash` print nothing, exit silently | **`libusb-1.0.dll` missing** (a dialog pops on the desktop) | supply **x64** `libusb-1.0.dll` next to `st-flash.exe` (installer does this) |
| `st-info --probe` shows `flash: 0` | chip DB not found | chip DB → `C:\Program Files (x86)\stlink\config\chips` (installer does this) |
| `st-info --probe` finds 0 programmers | ST-Link not on WinUSB | run Zadig (§2) |
| Telemetry probe returns 0 packets | not bound to port 14555, or wrong subnet | bind UDP **14555**; be on the `10.42.0.x` network |
| Tools "not found" right after install | PATH is read at shell start | open a **new** terminal |
| GCS build: `unknown type name 'pid_t'` / no `termios.h`/`poll.h` | SITL/autotune is POSIX-only and was left ON | configure the GCS with `-DNAVIGATOR_SITL=OFF` (§8.2) |
| `Navigator.exe` exits instantly, no console error | missing MinGW runtime / Qt dep DLLs (windeployqt6 under-deploys) | run `tools/windows/deploy-navigator-msys.sh` and launch from `dist\` (§8.3) |
| GCS: `Could not find Qt6` / `assimp` at configure | building outside UCRT64, or deps not installed | `export MSYSTEM=UCRT64; source /etc/profile`; `pacman -S` the qt6/assimp pkgs (§8.1) |

## Verified configuration

Windows 10 Pro 22H2 (19045) · Arm GNU 15.2.1 · CMake 4.3.3 · Ninja 1.13.2 ·
Python 3.13.1 · Git 2.54.0 · stlink 1.8.0 + libusb 1.0.30 → `main.bin`
143,148 bytes, ELF `Machine: ARM`, entry `0x80001c0`; flashed + verified to a
STM32F401xD_xE; ~250 NavLink pkt/s telemetry.

---

## Appendix A — Running Windows inside a Linux/KVM VM

Two things differ from bare metal: the ST-Link is a USB device on the **host**,
and the ESP is on a host-side network the VM can't reach directly.

**ST-Link → VM (USB passthrough).** On the libvirt host:

```bash
# stlink-hostdev.xml
# <hostdev mode='subsystem' type='usb' managed='yes'>
#   <source><vendor id='0x0483'/><product id='0x3748'/></source>
# </hostdev>
virsh -c qemu:///system attach-device <vm> stlink-hostdev.xml --live --config
```

Then do Zadig (§2) inside the VM. **Gotcha:** Zadig's driver swap re-enumerates
the device, which breaks the address-pinned passthrough — `detach-device` then
`attach-device` again to re-bind at the new address.

**ESP telemetry → VM (relay).** The host's hotspot NAT won't return ESP→VM, and
libvirt blocks guest→host ports, so run a small UDP relay on the host and add two
firewall rules (not persistent across reboot):

```bash
sudo iptables -I FORWARD 1 -i virbr0 -o <wifi-if> -j ACCEPT
sudo iptables -I INPUT   1 -i virbr0 -p udp --dport 14555 -j ACCEPT
python3 vayu-esp-relay.py          # binds 192.168.122.1:14555 <-> 10.42.0.1:14555
```

In the VM, point the GCS at the **relay** (`192.168.122.1:14555`), not the ESP.
The relay must talk to the ESP *from* port 14555 (the ESP replies to a fixed
port). See the host runbook (`~/win10vm-README.md`) for the relay script.
