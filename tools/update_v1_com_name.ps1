# Windows caches COM FriendlyName independently of USB iProduct/iInterface.
# Use the supported SetupAPI property; keep Microsoft's usbser.sys driver.
param(
    [switch]$Apply,
    [string[]]$DeviceSerial = @('5303284739002C9C', '50443405C0B5A81C'),
    [string]$ReportPath = (Join-Path $env:TEMP 'Nyanithm_V1_COM_name.json')
)
$ErrorActionPreference = 'Stop'
$expectedName = 'Nyanithm Controller V1'
$plan = @(
    Get-PnpDevice -Class Ports -PresentOnly | Where-Object InstanceId -like 'USB\VID_CAFE&PID_4009&MI_02\*' | ForEach-Object {
        $instance = $_.InstanceId
        $parent = (Get-PnpDeviceProperty -InstanceId $instance -KeyName DEVPKEY_Device_Parent).Data
        $serialNumber = ($parent -split '\\')[-1]
        if ($serialNumber -notin $DeviceSerial) { return }
        $reportedName = (Get-PnpDeviceProperty -InstanceId $instance -KeyName DEVPKEY_Device_BusReportedDeviceDesc).Data
        if ($reportedName -ne $expectedName) { throw "Flash the V1 release before renaming $instance" }
        $oldName = (Get-PnpDeviceProperty -InstanceId $instance -KeyName DEVPKEY_Device_FriendlyName).Data
        if ($oldName -notmatch '\((COM\d+)\)$') { throw "Cannot preserve COM number: $oldName" }
        [pscustomobject]@{ InstanceId = $instance; Serial = $serialNumber; Before = $oldName; After = "$expectedName ($($Matches[1]))" }
    }
)
if ($plan.Count -ne $DeviceSerial.Count) { throw 'All explicitly selected V1 boards must be present; no names changed' }
$plan | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $ReportPath -Encoding utf8
if (-not $Apply) { $plan; return }
$administrator = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $administrator) { throw 'SetupAPI requires Administrator membership; rerun this script as Administrator with -Apply' }

Add-Type -TypeDefinition @'
using System;
using System.ComponentModel;
using System.Runtime.InteropServices;
using System.Text;
public static class NyanithmComName {
    [StructLayout(LayoutKind.Sequential)]
    struct DeviceInfo { public uint Size; public Guid ClassGuid; public uint DevInst; public IntPtr Reserved; }
    [DllImport("setupapi.dll", SetLastError=true)]
    static extern IntPtr SetupDiCreateDeviceInfoList(IntPtr classGuid, IntPtr parent);
    [DllImport("setupapi.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern bool SetupDiOpenDeviceInfoW(IntPtr set, string instance, IntPtr parent, uint flags, ref DeviceInfo data);
    [DllImport("setupapi.dll", CharSet=CharSet.Unicode, SetLastError=true)]
    static extern bool SetupDiSetDeviceRegistryPropertyW(IntPtr set, ref DeviceInfo data, uint property, byte[] value, uint size);
    [DllImport("setupapi.dll", SetLastError=true)]
    static extern bool SetupDiDestroyDeviceInfoList(IntPtr set);
    public static void Set(string instance, string name) {
        IntPtr set = SetupDiCreateDeviceInfoList(IntPtr.Zero, IntPtr.Zero);
        if (set == new IntPtr(-1)) throw new Win32Exception(Marshal.GetLastWin32Error());
        try {
            DeviceInfo data = new DeviceInfo(); data.Size = (uint)Marshal.SizeOf(data);
            if (!SetupDiOpenDeviceInfoW(set, instance, IntPtr.Zero, 0, ref data))
                throw new Win32Exception(Marshal.GetLastWin32Error());
            byte[] value = Encoding.Unicode.GetBytes(name + "\0");
            if (!SetupDiSetDeviceRegistryPropertyW(set, ref data, 12, value, (uint)value.Length))
                throw new Win32Exception(Marshal.GetLastWin32Error());
        } finally { SetupDiDestroyDeviceInfoList(set); }
    }
}
'@
foreach ($item in $plan) {
    [NyanithmComName]::Set($item.InstanceId, $item.After)
    $actual = (Get-PnpDeviceProperty -InstanceId $item.InstanceId -KeyName DEVPKEY_Device_FriendlyName).Data
    if ($actual -ne $item.After) { throw "FriendlyName verification failed for $($item.InstanceId)" }
}
$plan | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath ($ReportPath + '.verified.json') -Encoding utf8
$plan
