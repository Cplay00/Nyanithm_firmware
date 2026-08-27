#!/usr/bin/env pwsh
param([string]$Variant = "hw_v1", [string]$BuildDir = "build")
$ErrorActionPreference = "Stop"
$TC = "C:\Users\HP\.pico-sdk\toolchain\14_2_Rel1\bin"
$NJ = "C:\Users\HP\AppData\Local\Programs\Python\Python312\Scripts"
$PY = "C:\Users\HP\AppData\Local\Programs\Python\Python312\python.exe"
$GM = "C:\Program Files\Git\mingw64\bin"
$SDK = "D:\pico-sdk"
$S = @{ "hw_v1" = "D:\Opencode,OCCM\Items\Nyanithm_Firmware\Nyanithm_firmware_hw_v1"; "cplay" = "D:\Opencode,OCCM\Items\Nyanithm_Firmware\Nyanithm_firmware_hw_v1_for_cplay" }
$J = @{ "hw_v1" = "D:\Nyanithm_build\fw_v1"; "cplay" = "D:\Nyanithm_build\fw_cplay" }
if (-not $S.ContainsKey($Variant)) { Write-Error "Use hw_v1 or cplay"; exit 1 }
$env:PICO_SDK_PATH = $SDK; $env:PATH = "$GM;$TC;$NJ;$env:PATH"
if (-not (Test-Path $J[$Variant])) { New-Item -ItemType Junction -Path $J[$Variant] -Target $S[$Variant] -Force | Out-Null }
$B = "$($J[$Variant])\$BuildDir"
if (-not (Test-Path "$B\build.ninja")) { New-Item -ItemType Directory -Path $B -Force | Out-Null; Push-Location $B; cmake -G Ninja -DCMAKE_BUILD_TYPE=Release $J[$Variant]; Pop-Location }
Push-Location $B
# round73: 构建时间一致性保障。固件把 __DATE__/__TIME__ 烧进版本串(0xBD 响应/启动日志),
# 但增量构建下未被改动的编译单元不会重编,UF2 会带着陈旧的构建时刻。
# 每次构建前扫描全部引用 __DATE__/__TIME__ 的源文件并刷新 mtime,强制 Ninja 重编它们;
# 以后新增使用时间宏的文件会被自动纳入,无需维护清单。
Write-Host "=== Refreshing __DATE__/__TIME__ sources ===" -ForegroundColor Cyan
Get-ChildItem "$($S[$Variant])\src", "$($S[$Variant])\include" -Recurse -Include *.c, *.cpp, *.h -ErrorAction SilentlyContinue |
    Select-String -Pattern '__DATE__|__TIME__' -List |
    ForEach-Object {
        Write-Host ("  touch " + $_.Path)
        (Get-Item $_.Path).LastWriteTime = Get-Date
    }
Write-Host "=== Compiling ($Variant) ===" -ForegroundColor Cyan
& "$NJ\ninja.exe" -j4 2>&1 | Where-Object { $_ -match "^\[" }
$lb = "$B\_link.bat"
[System.IO.File]::WriteAllText($lb, "@echo off`r`nset PATH=$TC;$GM;%PATH%`r`ncd /D $B`r`narm-none-eabi-g++.exe -mcpu=cortex-m0plus -mthumb -g -O3 -DNDEBUG -Wl,--build-id=none -Wl,-Map=Nyanithm.elf.map --specs=nosys.specs -Wl,-L$B -Wl,--script=$SDK/src/rp2_common/pico_crt0/rp2040/memmap_default.ld -Wl,-z,max-page-size=4096 -Wl,--gc-sections -Wl,--no-warn-rwx-segments @CMakeFiles\Nyanithm.rsp -o Nyanithm.elf`r`necho LINK_EXIT=%ERRORLEVEL%", [System.Text.Encoding]::ASCII)
Write-Host "=== Linking ===" -ForegroundColor Cyan
cmd /c $lb 2>&1
if (-not (Test-Path "$B\Nyanithm.elf")) { Write-Error "Link failed"; Pop-Location; exit 1 }
& "$TC\arm-none-eabi-objcopy.exe" -Obinary Nyanithm.elf Nyanithm.bin 2>&1
$e2u = if (Test-Path "$($J[$Variant])\tools\elf2uf2.py") { "$($J[$Variant])\tools\elf2uf2.py" } else { "D:\Nyanithm_build\elf2uf2.py" }
& $PY $e2u Nyanithm.elf Nyanithm.uf2 2>&1
if (Test-Path "Nyanithm.uf2") { Write-Host "=== Done ===" -ForegroundColor Green; Write-Host "UF2: $((Get-Item 'Nyanithm.uf2').Length) bytes"; Write-Host "SHA1: $((Get-FileHash 'Nyanithm.uf2' -Algorithm SHA1).Hash)" } else { Write-Error "UF2 failed" }
Pop-Location