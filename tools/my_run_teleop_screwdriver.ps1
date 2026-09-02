# Screwdriver-into-box teleoperation demo collection.
#
# Wraps start_lingbot_va_teleop_collection.ps1 - the multi-round collector that
# produced the mug and stapler demos on 2026-08-27 (episode names
# "<ts>_smolvla_teleop_<session>_rNNN": same "<prefix>_<session ts>_r<NNN>"
# structure this script still builds).  That earlier version was never
# committed; it was cleaned up, re-prefixed and committed as the LingBot
# collector in 4c16414.  Two of its defaults point at the LingBot lineage, so
# this wrapper points them back at the shared onearm_Tele dataset the previous
# tasks used:
#     UbuntuDatasetDataRoot  /home/nvidia/work/telop/onearm_Tele
#     DatasetRepoId          local/onearm_tele
#
# The task string is carried as base64 on purpose.  Windows PowerShell 5.1 reads
# a BOM-less UTF-8 script as ANSI, which would silently corrupt a literal
# Chinese prompt - and that string is written verbatim into every episode's
# metadata and every exported LeRobot frame, so a corrupted one poisons the
# dataset.  Keeping this file pure ASCII removes the failure mode entirely.

param(
    # "auto" lets the recorder find the ZLink2 adapter.  As of 2026-09-02 the
    # only serial device known to this machine is a CH340 that enumerates as
    # COM3, not the COM10 the upstream script defaults to.
    [string]$ComPort = "auto",

    [string]$Operator = "Lucky",

    # Override only if the wording genuinely changes.  It must stay byte-identical
    # across all 50 demos AND at rollout time.
    [string]$Task = "",

    [switch]$NoCameraPreview
)

$ErrorActionPreference = "Stop"
$here = Split-Path -Parent $MyInvocation.MyCommand.Path
$upstream = Join-Path $here "start_lingbot_va_teleop_collection.ps1"
if (-not (Test-Path -LiteralPath $upstream -PathType Leaf)) {
    throw "Upstream collector not found: $upstream"
}

# Decodes to the screwdriver task prompt (13 chars, 39 UTF-8 bytes).
$pinnedTask = [System.Text.Encoding]::UTF8.GetString(
    [Convert]::FromBase64String("5oqK5p2v5a2Q6YeM55qE6J665Lid5YiA5pS+6L+b57q455uS6YeM")
)
if ([string]::IsNullOrWhiteSpace($Task)) {
    $Task = $pinnedTask
}
elseif ($Task -ne $pinnedTask) {
    Write-Host ""
    Write-Host "  WARNING: task string differs from the pinned one." -ForegroundColor Yellow
    Write-Host "    pinned : $pinnedTask" -ForegroundColor Yellow
    Write-Host "    given  : $Task" -ForegroundColor Yellow
    Write-Host "  Every episode stores this verbatim and rollout must match it exactly." -ForegroundColor Yellow
    Write-Host ""
}

$dataRoot = "/home/nvidia/work/telop/onearm_Tele"
$repoId = "local/onearm_tele"

Write-Host "============================================================"
Write-Host "Screwdriver into box  -  teleoperation demo collection"
Write-Host "============================================================"
Write-Host "  task          : $Task"
Write-Host "  dataset root  : $dataRoot   (shared with mug / stapler / red parcel)"
Write-Host "  lerobot repo  : $repoId"
Write-Host "  leader port   : $ComPort"
Write-Host "  operator      : $Operator"
Write-Host ""
Write-Host "  This POWERS ON, ENABLES and SERVOS the arm, and physically OPENS" -ForegroundColor Yellow
Write-Host "  the gripper on the first round.  The e-stop must be RELEASED -" -ForegroundColor Yellow
Write-Host "  arming refuses to proceed while robot_emergency_stop is set." -ForegroundColor Yellow
Write-Host ""
Write-Host "  Attended config: no timeout will stop or power down the arm" -ForegroundColor Yellow
Write-Host "  (stop_on_command/feedback/packet/movement_timeout are all false)." -ForegroundColor Yellow
Write-Host "  Motion itself is gated by the deadman, packet age <= 2.0 s and" -ForegroundColor Yellow
Write-Host "  >= 5 Hz packet rate.  You are the watchdog." -ForegroundColor Yellow
Write-Host ""
Write-Host "  Per round: Enter = capture start pose | Space = begin | Space = end"
Write-Host "             then S = save (blocks for LeRobot export) / D = discard"
Write-Host "             then Enter = next round / Q = full stop and power off"
Write-Host "============================================================"
Write-Host ""

$arguments = @{
    Task                  = $Task
    Operator              = $Operator
    ComPort               = $ComPort
    UbuntuDatasetDataRoot = $dataRoot
    DatasetRepoId         = $repoId
}
if ($NoCameraPreview) { $arguments["NoCameraPreview"] = $true }

& $upstream @arguments
exit $LASTEXITCODE
