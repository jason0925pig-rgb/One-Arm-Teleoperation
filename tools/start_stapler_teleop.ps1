param(
    [string]$ComPort = "COM3",
    [string]$Operator = "Chenchao",
    [ValidateRange(1.0, 30.0)]
    [double]$RateHz = 15.0,
    [switch]$NoCameraPreview
)

# Stapler-into-parcel-box task teleoperation entry point (2026-08-27).
#
# Thin wrapper over start_full_teleop.ps1: it only pins the frozen task text
# and the dedicated LeRobot repo id for the stapler task, then delegates the
# whole verified multi-round collection lifecycle (cameras, stack, sender
# window, space toggle, S/D labelling, Q shutdown) to the existing launcher.
# The mug, parcel and water-bottle invocations are unchanged.
#
# Demonstration protocol agreed for this task:
#   grasp the stapler -> lift -> place INSIDE the parcel cardboard box.
#   First version: box position fixed, only the stapler's start position
#   varies between episodes.

$ErrorActionPreference = "Stop"

$collector = Join-Path $PSScriptRoot "start_full_teleop.ps1"
if (-not (Test-Path -LiteralPath $collector -PathType Leaf)) {
    throw "Base teleop launcher is missing: $collector"
}

# Frozen task text (base64 so no console codepage can corrupt it):
# 把订书机放进快递纸盒
$taskB64 = "5oqK6K6i5Lmm5py65pS+6L+b5b+r6YCS57q455uS"
$task = [System.Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($taskB64))

$forward = @{
    ComPort = $ComPort
    Task = $task
    Operator = $Operator
    DatasetRepoId = "local/onearm_stapler"
    RateHz = $RateHz
}
if ($NoCameraPreview) { $forward.NoCameraPreview = $true }

Write-Host "Stapler-into-box teleop collection; dataset repo id: local/onearm_stapler"
Write-Host "Task: $task"
& $collector @forward
exit $LASTEXITCODE
