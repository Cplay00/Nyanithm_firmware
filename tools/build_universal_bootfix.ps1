#!/usr/bin/env pwsh
param([string]$Variant = 'hw_v1', [string]$BuildDir = 'build_release')
# Compatibility entry point; the universal hw_v1 uses one build implementation.
& "$PSScriptRoot/build_firmware.ps1" -Variant $Variant -BuildDir $BuildDir
