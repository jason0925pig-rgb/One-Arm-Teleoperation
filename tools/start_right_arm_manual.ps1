[CmdletBinding()]
param(
    [string]$UbuntuHost = "nvidia@192.168.2.170",
    [string]$UbuntuProject = "/home/nvidia/work/telop/One-Arm-Teleoperation",
    [string]$WindowsSourceIp = "192.168.2.130",
    [string]$SshIdentityFile = "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519"
)

$ErrorActionPreference = "Stop"

if (-not (Test-Path -LiteralPath $SshIdentityFile -PathType Leaf)) {
    throw "SSH identity file is missing: $SshIdentityFile"
}

$remoteCommand = (
    "cd '$UbuntuProject' && " +
    "env ROS_DISTRO=humble bash tools/right_arm_manual_mode.sh"
)

Write-Host "Right-arm manual drag launcher"
Write-Host "SSH target : $UbuntuHost"
Write-Host "Wired IP   : $WindowsSourceIp"
Write-Host "Ctrl+C performs drag-off, disable, and power-off."

& ssh.exe `
    -tt `
    -i $SshIdentityFile `
    -b $WindowsSourceIp `
    -o BatchMode=yes `
    -o ServerAliveInterval=2 `
    -o ServerAliveCountMax=3 `
    $UbuntuHost `
    $remoteCommand

if ($LASTEXITCODE -ne 0) {
    throw "Right-arm manual mode exited with SSH code $LASTEXITCODE."
}
