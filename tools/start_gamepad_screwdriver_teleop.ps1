[CmdletBinding()]
param(
    [ValidateRange(1, 200)][int]$EpisodeCount = 20,
    [string]$Task = "",
    [string]$Operator = "Lucky",
    [string]$DatasetRepoId = "local/onearm_tele",
    [ValidateRange(0, 15)][int]$GamepadId = 0,
    [string]$WindowsPython = "C:\Python314\python.exe",
    [string]$WindowsSourceIp = "192.168.2.131",
    [string]$OrinHost = "armstrong-orin",
    [string]$OrinUdpTarget = "192.168.2.170:5010",
    [string]$PreviewUrl = "http://192.168.2.170:8088",
    [string]$OrinProject = "/home/nvidia/work/telop/One-Arm-Teleoperation",
    [string]$DatasetRoot = "/home/nvidia/work/telop/onearm_Tele"
)

# Windows reads the HID gamepad and sends directly to .170 over wired UDP.
Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

# Keep the source ASCII-only so Windows PowerShell 5.1 does not misread a
# UTF-8 file without a BOM. The Base64 value is the default Chinese task.
if ([string]::IsNullOrWhiteSpace($Task)) {
    $Task = [Text.Encoding]::UTF8.GetString(
        [Convert]::FromBase64String("5oqK5p2v5a2Q6YeM55qE6J665Lid5YiA5pS+6L+b57q455uS6YeM")
    )
}

function Quote-Bash([string]$Text) {
    if ($Text.Contains("'")) { throw "A remote argument may not contain a single quote." }
    return "'$Text'"
}
function Invoke-Orin([string]$Command) {
    & ssh.exe -o BatchMode=yes -o ConnectTimeout=8 $OrinHost $Command
    if ($LASTEXITCODE -ne 0) { throw "Orin command failed (exit $LASTEXITCODE)." }
}
function Ask-Outcome {
    while ($true) {
        $value = (Read-Host "Round finished: S=save success, F=save failure, D=discard").Trim().ToLowerInvariant()
        if ($value -in @("s", "success")) { return "success" }
        if ($value -in @("f", "failure")) { return "failure" }
        if ($value -in @("d", "discard")) { return "discard" }
    }
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$senderPath = Join-Path $PSScriptRoot "windows_gamepad_udp_sender.py"
$urdfPath = Join-Path $repoRoot "walle_description\urdf\walle.urdf"
foreach ($required in @($WindowsPython, $senderPath, $urdfPath)) {
    if (-not (Test-Path -LiteralPath $required -PathType Leaf)) { throw "Required local file is missing: $required" }
}
if ($Task -ne $Task.Trim() -or [string]::IsNullOrWhiteSpace($Task)) { throw "Task must be nonempty with no leading/trailing spaces." }
if ($DatasetRepoId -notmatch "^[^/\s]+/[^/\s]+$") { throw "DatasetRepoId must have owner/name form." }
if ($null -eq (Get-NetIPAddress -AddressFamily IPv4 -IPAddress $WindowsSourceIp -ErrorAction SilentlyContinue)) {
    throw "Windows wired source address is absent: $WindowsSourceIp"
}
$targetHost = ($OrinUdpTarget -split ":", 2)[0]
& ping.exe -n 2 -w 1000 -S $WindowsSourceIp $targetHost *> $null
if ($LASTEXITCODE -ne 0) { throw "Wired path $WindowsSourceIp -> $targetHost is unavailable." }

# Read-only local HID preflight: no network packet or robot request.
& $WindowsPython $senderPath --device-id $GamepadId --urdf $urdfPath --probe-only
if ($LASTEXITCODE -ne 0) { throw "Windows gamepad preflight failed." }

$taskBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Task))
$operatorBase64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Operator))
$envPrefix = "ROS_DISTRO='humble' ONE_ARM_ORBBEC_SETUP='/opt/ros/humble/setup.bash' ONE_ARM_LEROBOT_PYTHON='/home/nvidia/work/telop/.venvs/onearm-lerobot/bin/python' ONE_ARM_DATASET_DATA_ROOT=$(Quote-Bash $DatasetRoot) ONE_ARM_HEAD_SERIAL='CP8284100034' ONE_ARM_WRIST_SERIAL='CPCD75300083' ONE_ARM_PRIMARY_CAMERA_ROLE='chest' ONE_ARM_GRIPPER_DEVICE='/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0' ONE_ARM_GAMEPAD_SOURCE_IP='$WindowsSourceIp' ONE_ARM_GAMEPAD_UDP_PORT='5010'"
$orinProjectQuoted = Quote-Bash $OrinProject

Write-Host "============================================================"
Write-Host "Windows HID gamepad -> wired UDP -> new Orin (.170)"
Write-Host "Task: $Task"
Write-Host "Dataset: $DatasetRoot; existing task episodes=200-213"
Write-Host "New saved rounds append after the current highest index"
Write-Host "Gamepad id=$GamepadId; button[4]=initialize/start/finish; button[5]=gripper"
Write-Host "============================================================"

$senderProcess = $null
$senderScript = $null
$currentEpisode = ""
$recorderActive = $false
$stackStarted = $false
$datasetStarted = $false
$fatal = $false
try {
    Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_gamepad_teleop_stack.sh preflight"
    Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh start"
    $datasetStarted = $true
    Start-Process $PreviewUrl
    Write-Host "Camera preview opened: $PreviewUrl"
    Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_gamepad_teleop_stack.sh start"
    $stackStarted = $true

    for ($round = 1; $round -le $EpisodeCount; $round++) {
        $currentEpisode = "gamepad_screwdriver_$((Get-Date).ToString('yyyyMMdd_HHmmss'))_r$($round.ToString('D3'))"
        Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh record-start '$currentEpisode' '$taskBase64' '$operatorBase64'"
        $recorderActive = $true
        $suffix = "${PID}_${round}_$([Guid]::NewGuid().ToString('N').Substring(0,8))"
        Write-Host "Preparing power, enable and an open gripper. Servo remains OFF."
        Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_gamepad_teleop_stack.sh prepare"
        $senderLog = Join-Path ([IO.Path]::GetTempPath()) "windows_gamepad_${suffix}.log"
        $senderScript = Join-Path ([IO.Path]::GetTempPath()) "windows_gamepad_${suffix}.ps1"
        $body = @"
`$Host.UI.RawUI.WindowTitle = 'Windows Gamepad -> Armstrong Orin'
Write-Host 'HARDWARE ENABLED: gripper is open and servo is OFF. Centre all sticks, then press button[4] to initialize.' -ForegroundColor Green
& '$($WindowsPython -replace "'", "''")' '$($senderPath -replace "'", "''")' --device-id '$GamepadId' --bind-host '$WindowsSourceIp' --target '$OrinUdpTarget' --urdf '$($urdfPath -replace "'", "''")' 2>&1 | Tee-Object -FilePath '$($senderLog -replace "'", "''")'
exit `$LASTEXITCODE
"@
        Set-Content -LiteralPath $senderScript -Value $body -Encoding utf8
        $senderProcess = Start-Process powershell.exe -ArgumentList @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $senderScript) -WorkingDirectory $repoRoot -WindowStyle Normal -PassThru
        $deadline = (Get-Date).AddSeconds(10)
        while ((Get-Date) -lt $deadline) {
            if ($senderProcess.HasExited) { throw "Gamepad sender exited before initialization (code $($senderProcess.ExitCode)); log=$senderLog" }
            if ((Test-Path $senderLog -PathType Leaf) -and ((Get-Content $senderLog -Tail 20) -match "WINDOWS_GAMEPAD_READY")) { break }
            Start-Sleep -Milliseconds 200
        }
        if (-not (Test-Path $senderLog) -or -not ((Get-Content $senderLog -Tail 20) -match "WINDOWS_GAMEPAD_READY")) {
            throw "Gamepad sender did not become ready; log=$senderLog"
        }
        Write-Host "ROUND PREPARED: centre all axes, then press button[4] once to capture the initial pose."
        $initializationDeadline = (Get-Date).AddHours(1)
        while ((Get-Date) -lt $initializationDeadline) {
            if ($senderProcess.HasExited) { throw "Gamepad sender exited before initialization (code $($senderProcess.ExitCode)); log=$senderLog" }
            if ((Get-Content $senderLog -Tail 30 -ErrorAction SilentlyContinue) -match "GAMEPAD_INITIALIZED") { break }
            Start-Sleep -Milliseconds 100
        }
        if (-not ((Get-Content $senderLog -Tail 30 -ErrorAction SilentlyContinue) -match "GAMEPAD_INITIALIZED")) {
            throw "Timed out waiting for button[4] initialization; log=$senderLog"
        }
        Write-Host "INITIALIZATION SUCCESS: press button[4] again to start teleoperation immediately."
        $activationDeadline = (Get-Date).AddHours(1)
        while ((Get-Date) -lt $activationDeadline) {
            if ($senderProcess.HasExited) { throw "Gamepad sender exited before servo start (code $($senderProcess.ExitCode)); log=$senderLog" }
            if ((Get-Content $senderLog -Tail 30 -ErrorAction SilentlyContinue) -match "GAMEPAD_TELEOP_START_REQUESTED") { break }
            Start-Sleep -Milliseconds 100
        }
        if (-not ((Get-Content $senderLog -Tail 30 -ErrorAction SilentlyContinue) -match "GAMEPAD_TELEOP_START_REQUESTED")) {
            throw "Timed out waiting for button[4] to start servo; log=$senderLog"
        }
        Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_gamepad_teleop_stack.sh motion-start"
        Write-Host "SERVO LIVE: move the sticks; button[5] toggles gripper; button[4] stops this round."
        $senderProcess.WaitForExit()
        if ($senderProcess.ExitCode -ne 0) { throw "Gamepad sender failed (code $($senderProcess.ExitCode)); log=$senderLog" }
        Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_gamepad_teleop_stack.sh round-stop"
        Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh record-stop"
        $recorderActive = $false
        $result = Ask-Outcome
        if ($result -eq "discard") {
            Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh discard '$currentEpisode'"
        } else {
            Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh finalize '$result' '$DatasetRepoId' '$currentEpisode'"
        }
        $currentEpisode = ""
        Remove-Item -LiteralPath $senderScript -Force -ErrorAction SilentlyContinue
        $senderScript = $null
        if ($round -lt $EpisodeCount) {
            $next = Read-Host "Press Enter for the next round, or Q to stop"
            if ($next.Trim().ToLowerInvariant() -eq "q") { break }
        }
    }
}
catch {
    $fatal = $true
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    if ($null -ne $senderProcess -and -not $senderProcess.HasExited) { & taskkill.exe /PID $senderProcess.Id /T /F 2>$null | Out-Null }
    if ($recorderActive) {
        try { Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh record-stop" } catch { Write-Warning $_ }
    }
    if (-not [string]::IsNullOrWhiteSpace($currentEpisode)) {
        try { Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh discard '$currentEpisode'" } catch { Write-Warning $_ }
    }
}
finally {
    if ($null -ne $senderProcess -and -not $senderProcess.HasExited) { & taskkill.exe /PID $senderProcess.Id /T /F 2>$null | Out-Null }
    if ($null -ne $senderScript) { Remove-Item -LiteralPath $senderScript -Force -ErrorAction SilentlyContinue }
    if ($stackStarted) { try { Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_gamepad_teleop_stack.sh stop" } catch { Write-Warning $_ } }
    if ($datasetStarted) { try { Invoke-Orin "cd $orinProjectQuoted && $envPrefix bash tools/ubuntu_dataset_episode.sh stop" } catch { Write-Warning $_ } }
}
if ($fatal) { exit 1 }
Write-Host "WINDOWS_GAMEPAD_COLLECTION_COMPLETE"
