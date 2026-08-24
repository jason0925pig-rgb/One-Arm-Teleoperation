[CmdletBinding()]
param(
    [string]$ComPort = "COM10",
    [ValidateSet("new-humble", "legacy-jazzy")]
    [string]$DeploymentProfile = "new-humble",
    [string]$UbuntuHost = "",
    [string]$UbuntuProject = "",
    [string]$UbuntuUdpTarget = "",
    [string]$RequiredWindowsSourceIp = "",
    [string]$UbuntuRosDistro = "",
    [string]$UbuntuOrbbecSetup = "",
    [string]$UbuntuLerobotPython = "",
    [string]$UbuntuDatasetDataRoot = "",
    [string]$UbuntuHeadSerial = "",
    [string]$UbuntuWristSerial = "",
    [string]$UbuntuPrimaryCameraRole = "",
    [string]$UbuntuGripperDevice = "",
    [string]$SshIdentityFile = "$env:USERPROFILE\.ssh\one_arm_teleop_ed25519",
    [ValidateRange(1.0, 30.0)]
    [double]$RateHz = 15.0,
    [string]$Task = "",
    [string]$Operator = "Lucky",
    [string]$DatasetRepoId = "local/onearm_tele",
    [ValidateRange(1024, 65535)]
    [int]$CameraPreviewPort = 8088,
    [switch]$NoCameraPreview
)

# SmolVLA/LeRobot attended demonstration entry point.
#
# This intentionally delegates lifecycle handling to the verified persistent
# collector used by the LingBot source-data path. The data contract remains
# the SmolVLA contract: One-Arm's ubuntu_dataset_episode.sh writes the normal
# dual-camera ROS bag and exports the standard LeRobot episode on save.
#
# Per round: STOP -> mapping off + JAKA servo off only.
# Between rounds: robot power, robot enable, camera processes and ROS nodes
# stay alive. Q performs the only full disable/power-off cleanup.

$ErrorActionPreference = "Stop"

$collector = Join-Path $PSScriptRoot "start_lingbot_va_teleop_collection.ps1"
if (-not (Test-Path -LiteralPath $collector -PathType Leaf)) {
    throw "Persistent teleoperation collector is missing: $collector"
}

# The shared collector's own default is the separate LingBot source root.
# Retain the historical SmolVLA/LeRobot dataset location unless the operator
# deliberately supplies a different root.
if ([string]::IsNullOrWhiteSpace($UbuntuDatasetDataRoot)) {
    $UbuntuDatasetDataRoot = if ($DeploymentProfile -eq "new-humble") {
        "/home/nvidia/work/telop/onearm_Tele"
    }
    else {
        "/home/tele/onearm_teleop/One-Arm-Teleoperation/datasets/onearm_Tele"
    }
}

$forward = @{
    ComPort = $ComPort
    DeploymentProfile = $DeploymentProfile
    UbuntuHost = $UbuntuHost
    UbuntuProject = $UbuntuProject
    UbuntuUdpTarget = $UbuntuUdpTarget
    RequiredWindowsSourceIp = $RequiredWindowsSourceIp
    UbuntuRosDistro = $UbuntuRosDistro
    UbuntuOrbbecSetup = $UbuntuOrbbecSetup
    UbuntuLerobotPython = $UbuntuLerobotPython
    UbuntuDatasetDataRoot = $UbuntuDatasetDataRoot
    UbuntuHeadSerial = $UbuntuHeadSerial
    UbuntuWristSerial = $UbuntuWristSerial
    UbuntuPrimaryCameraRole = $UbuntuPrimaryCameraRole
    UbuntuGripperDevice = $UbuntuGripperDevice
    SshIdentityFile = $SshIdentityFile
    RateHz = $RateHz
    Task = $Task
    Operator = $Operator
    DatasetRepoId = $DatasetRepoId
    CameraPreviewPort = $CameraPreviewPort
    SessionPrefix = "smolvla_teleop"
    CollectorLabel = "SmolVLA / LeRobot"
}
if ($NoCameraPreview) {
    $forward.NoCameraPreview = $true
}

& $collector @forward
exit $LASTEXITCODE
