<#
.SYNOPSIS
  Monitor Nyanithm touch data via COM port to detect false triggers.
.DESCRIPTION
  Sends CMD_GET_INPUT (0xB1) to the device and logs any touch state changes
  with timestamps. Useful for diagnosing periodic false triggers.
.USAGE
  .\monitor_touch.ps1 -Port COM1 [-Duration 30]
#>
param(
    [string]$Port = "COM1",
    [int]$Duration = 30  # seconds to monitor
)

$CMD_DEV_DETECT = [byte]0xB0
$CMD_GET_INPUT  = [byte]0xB1
$RESPONSE_SIZE  = 33  # 32 slider bytes + 1 air byte

Write-Host "Opening $Port ..."
$serial = New-Object System.IO.Ports.SerialPort($Port, 115200, [System.IO.Ports.Parity]::None, 8, [System.IO.Ports.StopBits]::One)
$serial.ReadTimeout = 500
$serial.WriteTimeout = 500
$serial.Open()

Start-Sleep -Milliseconds 100

# Verify device
$serial.Write(@($CMD_DEV_DETECT), 0, 1)
Start-Sleep -Milliseconds 50
if ($serial.BytesToRead -gt 0) {
    $resp = New-Object byte[] 1
    $serial.Read($resp, 0, 1) | Out-Null
    if ($resp[0] -eq $CMD_DEV_DETECT) {
        Write-Host "Device detected on $Port" -ForegroundColor Green
    } else {
        Write-Host "Unexpected response: 0x$($resp[0].ToString('X2'))" -ForegroundColor Yellow
    }
} else {
    Write-Host "No response from device" -ForegroundColor Red
    $serial.Close()
    exit 1
}

Write-Host "Monitoring touch state for $Duration seconds..."
Write-Host "Timestamp | Changed cells | Touch bits (MPR0,MPR1,MPR2)"
Write-Host "-" * 70

$prevSlider = New-Object byte[] 32
$startTime = Get-Date
$changeCount = 0

while ((Get-Date) - $startTime -lt (New-TimeSpan -Seconds $Duration)) {
    try {
        $serial.Write(@($CMD_GET_INPUT), 0, 1)
        Start-Sleep -Milliseconds 1

        $buf = New-Object byte[] $RESPONSE_SIZE
        $read = 0
        while ($read -lt $RESPONSE_SIZE -and $serial.BytesToRead -gt 0) {
            $read += $serial.Read($buf, $read, $RESPONSE_SIZE - $read)
        }

        if ($read -eq $RESPONSE_SIZE) {
            # Check for changes in slider data
            $changed = $false
            $changedCells = @()
            for ($i = 0; $i -lt 32; $i++) {
                if ($buf[$i] -ne $prevSlider[$i]) {
                    $changed = $true
                    if ($buf[$i] -gt 0) {
                        $changedCells += "ON:$i"
                    } else {
                        $changedCells += "OFF:$i"
                    }
                }
                $prevSlider[$i] = $buf[$i]
            }

            if ($changed) {
                $changeCount++
                $ts = (Get-Date).ToString("HH:mm:ss.fff")
                $touchBits = ""
                # Reconstruct 16-bit touch words from slider data
                # This is approximate - just show which cells are active
                $activeCells = @()
                for ($i = 0; $i -lt 32; $i++) {
                    if ($buf[$i] -gt 0) { $activeCells += $i }
                }
                $cellStr = if ($activeCells.Count -gt 0) { $activeCells -join "," } else { "none" }
                Write-Host "$ts | $($changedCells -join ' ') | Cells: $cellStr"
            }
        }
    } catch {
        # Timeout or read error, continue
    }

    Start-Sleep -Milliseconds 2
}

Write-Host ""
Write-Host "Monitoring complete. Total state changes: $changeCount"
Write-Host "If changes occurred without touching the device, those are false triggers."
$serial.Close()
