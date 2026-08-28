param(
    [string]$ComPort = "COM3",
    [string]$Operator = "Chenchao",
    [ValidateRange(1.0, 30.0)]
    [double]$RateHz = 15.0,
    [switch]$NoCameraPreview
)

# Flip-parcel task teleoperation entry point (2026-08-28).
#
# Thin wrapper over start_full_teleop.ps1, same pattern as the mug/stapler
# tasks: pins the frozen task text and the dedicated LeRobot repo id, then
# delegates the whole verified multi-round collection lifecycle to the
# existing launcher. All other task invocations are unchanged.
#
# Demonstration protocol agreed for this task:
#   parcel starts SHIPPING-LABEL-DOWN. Grasp one edge of the parcel ->
#   lift slightly -> rotate the wrist to flip it over -> lay it down
#   label-up, roughly in place. Same flip direction in every episode.
#   Only the parcel's start position varies between episodes.

$ErrorActionPreference = "Stop"

$collector = Join-Path $PSScriptRoot "start_full_teleop.ps1"
if (-not (Test-Path -LiteralPath $collector -PathType Leaf)) {
    throw "Base teleop launcher is missing: $collector"
}

# Frozen task text (base64 so no console codepage can corrupt it):
# 把包裹翻过来
$taskB64 = "5oqK5YyF6KO557+76L+H5p2l"
$task = [System.Text.Encoding]::UTF8.GetString([Convert]::FromBase64String($taskB64))

$forward = @{
    ComPort = $ComPort
    Task = $task
    Operator = $Operator
    DatasetRepoId = "local/onearm_flip"
    RateHz = $RateHz
}
if ($NoCameraPreview) { $forward.NoCameraPreview = $true }

Write-Host "Flip-parcel teleop collection; dataset repo id: local/onearm_flip"
Write-Host "Task: $task"
& $collector @forward
exit $LASTEXITCODE
