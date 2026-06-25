<#
.SYNOPSIS
  Install everything needed to build the Vayu firmware on Windows x64.

.DESCRIPTION
  Installs into C:\tools (portable zips) plus Python (official installer), and
  adds every bin dir to the Machine PATH. Idempotent: re-running re-downloads
  only what is missing and de-dups PATH.

  Components (pinned):
    arm-none-eabi-gcc 15.2.1 (xPack)   - cross compiler / objcopy / size
    CMake             4.3.3            - build system generator
    Ninja             1.13.2           - build tool
    Git               2.54.0 (MinGit)  - clone + CMake FetchContent(NavHAL)
    stlink            1.8.0            - st-flash / st-info / st-util
    libusb            1.0.30 (x64 dll) - stlink runtime dep (not in the stlink zip)
    Python            3.13.1           - NavHAL Kconfig + navlink codegen
    pip: kconfiglib, numpy             - NavHAL tooling deps

  Also places the stlink chip DB at C:\Program Files (x86)\stlink\config so
  st-flash can read flash geometry. The ST-Link still needs the WinUSB driver
  (one-time Zadig step) before flashing -- see windows-setup.md.

.NOTES
  Run from an ELEVATED PowerShell:
    powershell -ExecutionPolicy Bypass -File install-vayu-build-deps.ps1
  Open a NEW terminal afterwards so the updated PATH is visible.
#>
$ErrorActionPreference = 'Stop'
$ProgressPreference     = 'SilentlyContinue'   # Invoke-WebRequest is glacial otherwise
[Net.ServicePointManager]::SecurityProtocol = [Net.SecurityProtocolType]::Tls12

$Tools = 'C:\tools'
$Dl    = Join-Path $Tools '_downloads'
New-Item -ItemType Directory -Force -Path $Tools, $Dl | Out-Null

# ---- 1. Portable zip tools -------------------------------------------------
$pkgs = [ordered]@{
  'arm-none-eabi-gcc' = 'https://github.com/xpack-dev-tools/arm-none-eabi-gcc-xpack/releases/download/v15.2.1-1.1/xpack-arm-none-eabi-gcc-15.2.1-1.1-win32-x64.zip'
  'cmake'             = 'https://github.com/Kitware/CMake/releases/download/v4.3.3/cmake-4.3.3-windows-x86_64.zip'
  'ninja'             = 'https://github.com/ninja-build/ninja/releases/download/v1.13.2/ninja-win.zip'
  'stlink'            = 'https://github.com/stlink-org/stlink/releases/download/v1.8.0/stlink-1.8.0-win32.zip'
  'git'               = 'https://github.com/git-for-windows/git/releases/download/v2.54.0.windows.1/MinGit-2.54.0-64-bit.zip'
}
foreach ($name in $pkgs.Keys) {
  $zip = Join-Path $Dl ($name + '.zip')
  $dst = Join-Path $Tools $name
  if (-not (Test-Path $zip)) { Write-Host "downloading $name"; Invoke-WebRequest $pkgs[$name] -OutFile $zip }
  if (Test-Path $dst) { Remove-Item -Recurse -Force $dst }
  Write-Host "extracting  $name"
  Expand-Archive -Path $zip -DestinationPath $dst -Force
}

# ---- 1b. stlink runtime fixups (needed before st-flash actually works) ------
# The stlink "win32" zip ships a 64-bit st-flash.exe but NO libusb-1.0.dll, and
# its chip DB lives under a nested "Program Files (x86)" folder that st-flash
# does not search. Supply the x64 libusb and place the chip DB where it looks.
$stflash = Get-ChildItem $Tools -Recurse -Filter st-flash.exe -ErrorAction SilentlyContinue | Select-Object -First 1
if ($stflash) {
  $stBin  = $stflash.Directory.FullName
  $stRoot = $stflash.Directory.Parent.FullName          # ...\stlink-1.8.0-win32

  # 64-bit libusb-1.0.dll next to st-flash.exe
  $lu7z = Join-Path $Dl 'libusb.7z'
  if (-not (Test-Path $lu7z)) { Write-Host 'downloading libusb'; Invoke-WebRequest 'https://github.com/libusb/libusb/releases/download/v1.0.30/libusb-1.0.30.7z' -OutFile $lu7z }
  $luDir = Join-Path $Dl 'libusb'
  if (Test-Path $luDir) { Remove-Item -Recurse -Force $luDir }
  New-Item -ItemType Directory -Force -Path $luDir | Out-Null
  & tar.exe -xf $lu7z -C $luDir                          # Win10/11 tar.exe reads .7z
  $luDll = Get-ChildItem $luDir -Recurse -Filter libusb-1.0.dll | Where-Object { $_.FullName -match 'MS64\\dll' } | Select-Object -First 1
  if ($luDll) { Copy-Item $luDll.FullName (Join-Path $stBin 'libusb-1.0.dll') -Force; Write-Host "placed libusb-1.0.dll in $stBin" }
  else { Write-Warning 'libusb x64 dll not found in archive' }

  # chip DB at the hardcoded path st-flash falls back to
  $cfgSrc = Join-Path $stRoot 'Program Files (x86)\stlink\config'
  $cfgDst = 'C:\Program Files (x86)\stlink\config'
  if (Test-Path $cfgSrc) {
    New-Item -ItemType Directory -Force -Path $cfgDst | Out-Null
    Copy-Item "$cfgSrc\*" $cfgDst -Recurse -Force
    Write-Host "installed chip DB -> $cfgDst"
  }
} else { Write-Warning 'st-flash.exe not found; skipping stlink fixups' }

# ---- 2. Python (official silent installer) ---------------------------------
$pyUrl = 'https://www.python.org/ftp/python/3.13.1/python-3.13.1-amd64.exe'
$pyExe = Join-Path $Dl 'python-3.13.1-amd64.exe'
if (-not (Get-Command python -ErrorAction SilentlyContinue)) {
  if (-not (Test-Path $pyExe)) { Write-Host 'downloading python'; Invoke-WebRequest $pyUrl -OutFile $pyExe }
  Write-Host 'installing python (silent)'
  Start-Process $pyExe -ArgumentList '/quiet','InstallAllUsers=1','PrependPath=1','Include_test=0','Include_launcher=1' -Wait
}

# python3.exe shim: NavHAL/navlink call bare `python3`, which on Windows hits the
# Microsoft Store alias stub. Put a real python3.exe next to python.exe (which
# precedes the Store stub on PATH).
$pyDir = Split-Path (Get-Command python -ErrorAction SilentlyContinue).Source -Parent
if (-not $pyDir) { $pyDir = 'C:\Program Files\Python313' }
Copy-Item (Join-Path $pyDir 'python.exe') (Join-Path $pyDir 'python3.exe') -Force

# ---- 3. Python deps for NavHAL tooling -------------------------------------
& (Join-Path $pyDir 'python.exe') -m pip install --quiet --upgrade pip kconfiglib numpy

# ---- 4. Merge tool bin dirs into Machine PATH (de-duped) -------------------
$keyExes = 'arm-none-eabi-gcc.exe','cmake.exe','ninja.exe','st-flash.exe','git.exe'
$bins = @($pyDir)
foreach ($exe in $keyExes) {
  $hit = Get-ChildItem $Tools -Recurse -Filter $exe -ErrorAction SilentlyContinue | Select-Object -First 1
  if ($hit) { $bins += $hit.Directory.FullName } else { Write-Warning "NOT FOUND: $exe" }
}
$parts = ([Environment]::GetEnvironmentVariable('Path','Machine') -split ';') | Where-Object { $_ -ne '' }
foreach ($b in $bins) { if ($parts -notcontains $b) { $parts += $b } }
[Environment]::SetEnvironmentVariable('Path', ($parts -join ';'), 'Machine')

Write-Host "`nAdded to Machine PATH:`n  $($bins -join "`n  ")"
Write-Host "`nDONE. Open a NEW terminal to pick up PATH. Then run Zadig once for the"
Write-Host "ST-Link (WinUSB) and build/flash per docs/reference/windows-setup.md."
