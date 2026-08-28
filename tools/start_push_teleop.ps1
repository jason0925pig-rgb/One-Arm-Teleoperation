param(
    [string]$ComPort = "COM3",
    [string]$Operator = "Chenchao",
    [ValidateRange(1.0, 30.0)]
    [double]$RateHz = 15.0,
    [switch]$NoCameraPreview
)

# Push-parcel-into-frame task teleoperation entry point (2026-08-28).
#
# Thin wrapper over start_full_teleop.ps1, same pattern as the mug/stapler
# tasks: pins the frozen task text and the dedicated LeRobot repo id, then
# delegates the whole verified multi-round collection lifecycle to the
# existing launcher. All other task invocations are unchanged.
#
# Demonstration protocol agreed for this task (NON-PREHENSILE - no grasping):
#   close the gripper first, then use the closed gripper as a pusher to
#   slide the parcel along the table INTO the black tape frame.
#   The gripper never opens around the parcel. Frame position fixed,
#   only the parcel's start position varies between episodes.

$ErrorActionPreference = "Stop"

$collector = Join-Path $PSScriptRoot "start_full_teleop.ps1"
if (-not (Test-Path -LiteralPath $collector -PathType Leaf)) {
    throw "Base teleop launcher is missing: $collector"
}

# Frozen task text (base64 so no console codepage can corrupt it):
# 把包裹推到黑色方框里
$taskB64 = "5oqK5YyF6KO55o6o5Yiw6buR6Imy5pa55qGG6YeM"
$task = [System.Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($taskB64))

$forward = @{
    ComPort = $ComPort
    Task = $task
    Operator = $Operator
    DatasetRepoId = "local/onearm_push"
    RateHz = $RateHz
}
if ($NoCameraPreview) { $forward.NoCameraPreview = $true }

Write-Host "Push-parcel teleop collection; dataset repo id: local/onearm_push"
Write-Host "Task: $task"
& $collector @forward
exit $LASTEXITCODE
