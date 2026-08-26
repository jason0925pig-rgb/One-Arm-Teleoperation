param(
    [string]$ComPort = "COM3",
    [string]$Operator = "Chenchao",
    [ValidateRange(1.0, 30.0)]
    [double]$RateHz = 15.0,
    [switch]$NoCameraPreview
)

# Mug-rotation task teleoperation entry point (task 3, 2026-08-26).
#
# Thin wrapper over start_full_teleop.ps1: it only pins the frozen task text
# and the dedicated LeRobot repo id for the mug task, then delegates the whole
# verified multi-round collection lifecycle (cameras, stack, sender window,
# space toggle, S/D labelling, Q shutdown) to the existing launcher.
# The parcel and water-bottle invocations are unchanged.
#
# Demonstration protocol agreed for this task:
#   grasp the mug BY THE HANDLE -> lift -> place ON TOP of the purple box.
#   First version: handle initial orientation and box position fixed, only
#   the mug's start position varies between episodes.

$ErrorActionPreference = "Stop"

$collector = Join-Path $PSScriptRoot "start_full_teleop.ps1"
if (-not (Test-Path -LiteralPath $collector -PathType Leaf)) {
    throw "Base teleop launcher is missing: $collector"
}

# Frozen task text (base64 so no console codepage can corrupt it):
# 把水杯放到紫色的箱子上
$taskB64 = "5oqK5rC05p2v5pS+5Yiw57Sr6Imy55qE566x5a2Q5LiK"
$task = [System.Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($taskB64))

$forward = @{
    ComPort = $ComPort
    Task = $task
    Operator = $Operator
    DatasetRepoId = "local/onearm_mug"
    RateHz = $RateHz
}
if ($NoCameraPreview) { $forward.NoCameraPreview = $true }

Write-Host "Mug-rotation teleop collection; dataset repo id: local/onearm_mug"
Write-Host "Task: $task"
& $collector @forward
exit $LASTEXITCODE
