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
    [string]$DatasetRepoId = "local/lingbot_va_source",
    [ValidateRange(1024, 65535)]
    [int]$CameraPreviewPort = 8088,
    [switch]$NoCameraPreview
)

# A persistent, attended demonstration collector for LingBot-VA source data.
# It intentionally does NOT replace start_full_teleop.ps1.  Each normal round
# ends with /teleop disabled and the JAKA servo mode exited, while the ROS
# nodes, controller power and robot-enable state remain available for the next
# round.  Only Q or an abnormal launch failure runs full power-off cleanup.

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$profileDefaults = if ($DeploymentProfile -eq "new-humble") {
    @{
        UbuntuHost = "nvidia@192.168.2.170"
        UbuntuProject = "/home/nvidia/work/telop/One-Arm-Teleoperation"
        UbuntuUdpTarget = "192.168.2.170:5005"
        RequiredWindowsSourceIp = "192.168.2.130"
        UbuntuRosDistro = "humble"
        UbuntuOrbbecSetup = "/opt/ros/humble/setup.bash"
        UbuntuLerobotPython = "/home/nvidia/work/telop/.venvs/onearm-lerobot/bin/python"
        # Keep LingBot source episodes separate from the existing SmolVLA/QGF
        # recordings.  It is a LeRobot v3 source dataset, not a trained adapter.
        UbuntuDatasetDataRoot = "/home/nvidia/work/telop/lingbot_va_teleop"
        UbuntuHeadSerial = "CP8284100034"
        UbuntuWristSerial = "CPCD75300083"
        UbuntuPrimaryCameraRole = "chest"
        UbuntuGripperDevice = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
    }
}
else {
    @{
        UbuntuHost = "armstrong-host"
        UbuntuProject = "/home/tele/onearm_teleop/One-Arm-Teleoperation"
        UbuntuUdpTarget = "192.168.2.116:5005"
        RequiredWindowsSourceIp = "192.168.2.130"
        UbuntuRosDistro = "jazzy"
        UbuntuOrbbecSetup = "/home/tele/ros2_ws/install/setup.bash"
        UbuntuLerobotPython = "/home/tele/.venvs/onearm-lerobot/bin/python"
        UbuntuDatasetDataRoot = "/home/tele/onearm_teleop/One-Arm-Teleoperation/datasets/lingbot_va_teleop"
        UbuntuHeadSerial = "CPCD7530003J"
        UbuntuWristSerial = "CPCBC5300077"
        UbuntuPrimaryCameraRole = "head"
        UbuntuGripperDevice = "/dev/serial/by-id/usb-1a86_USB_Single_Serial_5ABB000800-if00"
    }
}
foreach ($name in $profileDefaults.Keys) {
    if ([string]::IsNullOrWhiteSpace((Get-Variable -Name $name).Value)) {
        Set-Variable -Name $name -Value $profileDefaults[$name]
    }
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$recorder = Join-Path $PSScriptRoot "run_zlink2_recorder.cmd"
if (-not (Test-Path -LiteralPath $recorder -PathType Leaf)) {
    throw "ZLink2 recorder launcher is missing: $recorder"
}

function ConvertTo-BashSingleQuoted {
    param([Parameter(Mandatory = $true)][AllowEmptyString()][string]$Value)
    if ($Value.Contains("'")) {
        throw "Remote shell values may not contain a single quote: $Value"
    }
    return "'" + $Value + "'"
}

function ConvertTo-Utf8Base64 {
    param([Parameter(Mandatory = $true)][AllowEmptyString()][string]$Value)
    return [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Value))
}

function Get-UdpSourceAddress {
    param([Parameter(Mandatory = $true)][string]$Target)
    $targetHost = ($Target -split ":", 2)[0]
    $client = [System.Net.Sockets.UdpClient]::new()
    try {
        $client.Connect($targetHost, 5005)
        return ([System.Net.IPEndPoint]$client.Client.LocalEndPoint).Address.ToString()
    }
    finally {
        $client.Dispose()
    }
}

function Assert-WiredPath {
    param([Parameter(Mandatory = $true)][string]$Target, [Parameter(Mandatory = $true)][string]$SourceIp)
    if ($null -eq (Get-NetIPAddress -AddressFamily IPv4 -IPAddress $SourceIp -ErrorAction SilentlyContinue)) {
        throw "Required wired Windows IPv4 address is absent: $SourceIp"
    }
    $targetHost = ($Target -split ":", 2)[0]
    & ping.exe -n 2 -w 1000 -S $SourceIp $targetHost *> $null
    if ($LASTEXITCODE -ne 0) {
        throw "Wired route $SourceIp -> $targetHost is unreachable. No remote process was started."
    }
}

$targetParts = $UbuntuUdpTarget -split ":", 2
if ($targetParts.Count -ne 2) {
    throw "UbuntuUdpTarget must be HOST:PORT."
}
$windowsSourceIp = if ([string]::IsNullOrWhiteSpace($RequiredWindowsSourceIp)) {
    Get-UdpSourceAddress -Target $UbuntuUdpTarget
}
else {
    $RequiredWindowsSourceIp
}
Assert-WiredPath -Target $UbuntuUdpTarget -SourceIp $windowsSourceIp

function Get-SshArguments {
    $arguments = @(
        "-o", "BatchMode=yes",
        "-o", "ConnectTimeout=8",
        "-o", "ServerAliveInterval=15",
        "-o", "ServerAliveCountMax=8",
        "-o", "TCPKeepAlive=yes",
        "-b", $windowsSourceIp
    )
    if (-not [string]::IsNullOrWhiteSpace($SshIdentityFile)) {
        if (-not (Test-Path -LiteralPath $SshIdentityFile -PathType Leaf)) {
            throw "SSH identity file is missing: $SshIdentityFile"
        }
        $arguments += @("-i", $SshIdentityFile)
    }
    return $arguments
}

function Invoke-CheckedSsh {
    param([Parameter(Mandatory = $true)][string]$RemoteCommand)
    $sshArguments = Get-SshArguments
    & ssh.exe @sshArguments $UbuntuHost $RemoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "SSH command failed with exit code $LASTEXITCODE."
    }
}

function Get-RemoteEnvironmentPrefix {
    $pairs = [ordered]@{
        ROS_DISTRO = $UbuntuRosDistro
        ONE_ARM_ORBBEC_SETUP = $UbuntuOrbbecSetup
        ONE_ARM_LEROBOT_PYTHON = $UbuntuLerobotPython
        ONE_ARM_DATASET_DATA_ROOT = $UbuntuDatasetDataRoot
        ONE_ARM_HEAD_SERIAL = $UbuntuHeadSerial
        ONE_ARM_WRIST_SERIAL = $UbuntuWristSerial
        ONE_ARM_PRIMARY_CAMERA_ROLE = $UbuntuPrimaryCameraRole
        ONE_ARM_GRIPPER_DEVICE = $UbuntuGripperDevice
        ONE_ARM_CAMERA_PREVIEW_PORT = $CameraPreviewPort.ToString([Globalization.CultureInfo]::InvariantCulture)
    }
    return (($pairs.Keys | ForEach-Object { $_ + "=" + (ConvertTo-BashSingleQuoted -Value ([string]$pairs[$_])) }) -join " ") + " "
}

$remoteProject = ConvertTo-BashSingleQuoted -Value $UbuntuProject
$remoteEnvironment = Get-RemoteEnvironmentPrefix
$taskBase64 = ""
$operatorBase64 = ""

$script:RemoteStackStarted = $false
$script:DatasetStarted = $false
$script:RoundServoActive = $false
$script:SenderProcess = $null
$script:SenderLog = $null
$script:SenderPathFile = $null
$script:ReadyFile = $null
$script:SenderScript = $null
$script:CurrentEpisode = ""
$script:CurrentLocalSession = ""
$script:CurrentRecorderStarted = $false

function Stop-CurrentRoundServo {
    if (-not $script:RemoteStackStarted -or -not $script:RoundServoActive) {
        return
    }
    Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh round-stop"
    $script:RoundServoActive = $false
}

function Stop-CurrentEpisodeRecorder {
    if (-not $script:CurrentRecorderStarted) {
        return
    }
    Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh record-stop"
    $script:CurrentRecorderStarted = $false
}

function Remove-CurrentLocalSession {
    if ([string]::IsNullOrWhiteSpace($script:CurrentLocalSession)) {
        return
    }
    $root = [IO.Path]::GetFullPath((Join-Path $repoRoot "recordings"))
    $path = [IO.Path]::GetFullPath($script:CurrentLocalSession)
    if ([IO.Path]::GetDirectoryName($path) -ne $root -or -not (Test-Path -LiteralPath $path -PathType Container)) {
        throw "Refusing to discard an unexpected leader-session path: $path"
    }
    $item = Get-Item -LiteralPath $path -Force
    if (($item.Attributes -band [IO.FileAttributes]::ReparsePoint) -ne 0) {
        throw "Refusing to delete a reparse-point leader session: $path"
    }
    Remove-Item -LiteralPath $path -Recurse -Force
    Write-Host "WINDOWS_SENDER_SESSION_DISCARDED=$path"
    $script:CurrentLocalSession = ""
}

function Discard-CurrentEpisode {
    Stop-CurrentRoundServo
    Stop-CurrentEpisodeRecorder
    if (-not [string]::IsNullOrWhiteSpace($script:CurrentEpisode)) {
        Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh discard '$($script:CurrentEpisode)'"
    }
    Remove-CurrentLocalSession
}

function Stop-AllRemoteResources {
    if ($script:RemoteStackStarted) {
        Write-Host "Stopping mapping, servo, gripper, robot enable and power..."
        try {
            Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh stop"
        }
        catch {
            Write-Warning "Remote full shutdown was not fully confirmed: $($_.Exception.Message)"
        }
        $script:RemoteStackStarted = $false
    }
    if ($script:DatasetStarted) {
        Write-Host "Stopping the two cameras and preview..."
        try {
            Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh stop"
        }
        catch {
            Write-Warning "Remote camera shutdown was not fully confirmed: $($_.Exception.Message)"
        }
        $script:DatasetStarted = $false
    }
}

function Wait-SenderBaseline {
    $deadline = (Get-Date).AddSeconds(180)
    while ((Get-Date) -lt $deadline) {
        if (Test-Path -LiteralPath $script:SenderPathFile -PathType Leaf) {
            $candidate = (Get-Content -LiteralPath $script:SenderPathFile -Raw).Trim()
            if (-not [string]::IsNullOrWhiteSpace($candidate)) {
                $script:CurrentLocalSession = $candidate
                Write-Host "SENDER_BASELINE_CAPTURED=$candidate"
                return
            }
        }
        if ($script:SenderProcess.HasExited) {
            throw "Leader sender exited before Enter/baseline (code $($script:SenderProcess.ExitCode))."
        }
        Start-Sleep -Milliseconds 250
    }
    throw "Leader sender did not capture Enter/baseline within 180 seconds."
}

function Wait-SenderPreview {
    $deadline = (Get-Date).AddSeconds(30)
    while ((Get-Date) -lt $deadline) {
        if (Test-Path -LiteralPath $script:SenderLog -PathType Leaf) {
            if ((Get-Content -LiteralPath $script:SenderLog -Tail 50) | Select-String -Pattern "frames=.*complete=" -Quiet) {
                return
            }
        }
        if ($script:SenderProcess.HasExited) {
            throw "Leader sender exited before a live preview frame (code $($script:SenderProcess.ExitCode))."
        }
        Start-Sleep -Milliseconds 250
    }
    throw "Leader sender did not produce a live preview frame within 30 seconds."
}

function Start-LeaderSender {
    param([Parameter(Mandatory = $true)][string]$EpisodeName, [Parameter(Mandatory = $true)][int]$Round)
    $suffix = "${PID}_${Round}_$([Guid]::NewGuid().ToString('N').Substring(0, 8))"
    $script:ReadyFile = Join-Path ([IO.Path]::GetTempPath()) "lingbot_va_ready_${suffix}.flag"
    $script:SenderPathFile = Join-Path ([IO.Path]::GetTempPath()) "lingbot_va_session_${suffix}.txt"
    $script:SenderLog = Join-Path ([IO.Path]::GetTempPath()) "lingbot_va_sender_${suffix}.log"
    $script:SenderScript = Join-Path ([IO.Path]::GetTempPath()) "lingbot_va_sender_${suffix}.ps1"
    foreach ($path in @($script:ReadyFile, $script:SenderPathFile, $script:SenderLog, $script:SenderScript)) {
        if (Test-Path -LiteralPath $path) { Remove-Item -LiteralPath $path -Force }
    }

    $senderArguments = @(
        "--port", $ComPort,
        "--rate-hz", $RateHz.ToString([Globalization.CultureInfo]::InvariantCulture),
        "--udp-target", $UbuntuUdpTarget,
        "--udp-bind-host", $windowsSourceIp,
        "--max-consecutive-incomplete", "0",
        # Enter captures this round's absolute-offset baseline.  A just-opened
        # USB serial port can need longer than the recorder default for all
        # eight PRAD replies to settle; retain the strict 8/8 requirement.
        "--baseline-retry-seconds", "20",
        "--deadman",
        "--activation-file", $script:ReadyFile,
        "--session-path-file", $script:SenderPathFile,
        "--session-name", $EpisodeName,
        "--task", $Task,
        "--operator", $Operator
    )
    $quoted = $senderArguments | ForEach-Object { "'" + ($_ -replace "'", "''") + "'" }
    $senderBody = @"
`$Host.UI.RawUI.WindowTitle = 'LingBot-VA ZLink2 Teleoperation Sender'
`$ErrorActionPreference = 'Continue'
`$logPath = '$($script:SenderLog -replace "'", "''")'
`$recorder = '$($recorder -replace "'", "''")'
`$arguments = @(
    $($quoted -join ",`r`n    ")
)
'LINGBOT_VA_SENDER_STARTED=' + (Get-Date -Format o) | Tee-Object -FilePath `$logPath
& `$recorder @arguments 2>&1 | Tee-Object -FilePath `$logPath -Append
`$exitCode = `$LASTEXITCODE
'LINGBOT_VA_SENDER_EXIT_CODE=' + `$exitCode | Tee-Object -FilePath `$logPath -Append
exit `$exitCode
"@
    Set-Content -LiteralPath $script:SenderScript -Value $senderBody -Encoding utf8
    Write-Host "A separate ZLink2 sender window is opening for round $Round."
    Write-Host "  Enter = capture this round's start pose."
    Write-Host "  First Space = start teleoperation after REMOTE ROUND READY."
    Write-Host "  Second Space = finish round, send STOP and exit servo only."
    $script:SenderProcess = Start-Process -FilePath "powershell.exe" -ArgumentList @(
        "-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $script:SenderScript
    ) -WorkingDirectory $repoRoot -WindowStyle Normal -PassThru
}

function Ask-Outcome {
    while ($true) {
        $answer = (Read-Host "Round finished: S=save this LeRobot source episode, D=discard it").Trim().ToLowerInvariant()
        if ($answer -in @("s", "save")) { return "save" }
        if ($answer -in @("d", "discard", "f", "failure")) { return "discard" }
        Write-Host "Please enter S or D."
    }
}

function Ask-NextRound {
    while ($true) {
        $answer = Read-Host "Press Enter for the next round, or Q to fully stop/power off"
        if ([string]::IsNullOrWhiteSpace($answer)) { return $true }
        if ($answer.Trim().ToLowerInvariant() -eq "q") { return $false }
        Write-Host "Press Enter for the next round or Q to quit."
    }
}

if ([string]::IsNullOrWhiteSpace($Task)) {
    $Task = Read-Host "Task prompt (exact text stored in every saved episode)"
}
if ([string]::IsNullOrWhiteSpace($Task) -or $Task -ne $Task.Trim()) {
    throw "Task must be nonempty and may not have leading/trailing whitespace."
}
if ($DatasetRepoId -notmatch "^[^/\s]+/[^/\s]+$") {
    throw "DatasetRepoId must have owner/name form."
}
$taskBase64 = ConvertTo-Utf8Base64 -Value $Task
$operatorBase64 = ConvertTo-Utf8Base64 -Value $Operator

Write-Host "Checking all eight ZLink2 encoder IDs before remote startup..."
& $recorder "--port" $ComPort "--rate-hz" $RateHz.ToString([Globalization.CultureInfo]::InvariantCulture) "--probe-only"
if ($LASTEXITCODE -ne 0) {
    throw "ZLink2 8-ID preflight failed; no remote camera/robot process was started."
}

$previewUrl = "http://$($targetParts[0]):$CameraPreviewPort/"
Write-Host "============================================================"
Write-Host "LingBot-VA multi-round teleoperation source collector"
Write-Host "Task            : $Task"
Write-Host "Dataset source  : $UbuntuDatasetDataRoot"
Write-Host "LeRobot repo id : $DatasetRepoId"
Write-Host "Robot host      : $UbuntuHost (wired source $windowsSourceIp)"
Write-Host "Round boundary  : mapping OFF + servo OFF; power/enable/nodes stay ON"
Write-Host "Quit boundary   : ordered full stop, disable and power off"
Write-Host "============================================================"

$round = 0
$firstRound = $true
$fatal = $false
try {
    $script:DatasetStarted = $true
    Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh start"
    if (-not $NoCameraPreview) {
        Start-Process $previewUrl | Out-Null
    }
    $script:RemoteStackStarted = $true
    Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh start '$windowsSourceIp'"

    while ($true) {
        $round += 1
        $script:CurrentEpisode = "lingbot_va_$((Get-Date).ToString('yyyyMMdd_HHmmss'))_r$($round.ToString('D3'))"
        $script:CurrentLocalSession = ""
        $script:CurrentRecorderStarted = $true
        Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh record-start '$($script:CurrentEpisode)' '$taskBase64' '$operatorBase64'"
        Start-LeaderSender -EpisodeName $script:CurrentEpisode -Round $round
        Wait-SenderBaseline
        Wait-SenderPreview

        if ($firstRound) {
            Write-Host "First round: the robot will now power on, enable, open its gripper and enter servo mode. It still will not move until Space."
            Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh arm"
            $firstRound = $false
        }
        else {
            Write-Host "Next round: re-entering servo mode only; robot power/enable and nodes are retained."
            Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh round-arm"
        }
        $script:RoundServoActive = $true
        Set-Content -LiteralPath $script:ReadyFile -Value "LINGBOT_VA_ROUND_READY" -Encoding ascii
        Write-Host "REMOTE ROUND READY. Use the sender window: Space starts, Space again ends this round."

        $script:SenderProcess.WaitForExit()
        if ($script:SenderProcess.ExitCode -ne 0) {
            throw "ZLink2 sender exited with code $($script:SenderProcess.ExitCode)."
        }
        Stop-CurrentRoundServo
        Stop-CurrentEpisodeRecorder

        if ((Ask-Outcome) -eq "save") {
            Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh finalize success '$DatasetRepoId' '$($script:CurrentEpisode)'"
            Write-Host "ROUND_SAVED=$($script:CurrentEpisode)"
        }
        else {
            Invoke-CheckedSsh "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh discard '$($script:CurrentEpisode)'"
            Remove-CurrentLocalSession
            Write-Host "ROUND_DISCARDED=$($script:CurrentEpisode)"
        }
        $script:CurrentEpisode = ""
        if (-not (Ask-NextRound)) { break }
    }
}
catch {
    $fatal = $true
    Write-Host "ERROR: $($_.Exception.Message)" -ForegroundColor Red
    if ($null -ne $script:SenderProcess -and -not $script:SenderProcess.HasExited) {
        & taskkill.exe /PID $script:SenderProcess.Id /T /F 2>$null | Out-Null
    }
    try {
        Discard-CurrentEpisode
        Write-Host "CURRENT_ROUND_DISCARDED_AFTER_ERROR"
    }
    catch {
        Write-Warning "Could not fully discard the interrupted episode: $($_.Exception.Message)"
    }
}
finally {
    foreach ($path in @($script:ReadyFile, $script:SenderPathFile, $script:SenderScript)) {
        if (-not [string]::IsNullOrWhiteSpace($path) -and (Test-Path -LiteralPath $path)) {
            Remove-Item -LiteralPath $path -Force
        }
    }
    Stop-AllRemoteResources
}

if ($fatal) { exit 1 }
Write-Host "LINGBOT_VA_COLLECTION_SESSION_COMPLETE"
