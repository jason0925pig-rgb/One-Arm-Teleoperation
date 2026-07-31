[CmdletBinding()]
param(
    [string]$ComPort = "COM10",
    [string]$UbuntuHost = "armstrong-host",
    [string]$UbuntuProject = "/home/tele/onearm_teleop/One-Arm-Teleoperation",
    [string]$UbuntuUdpTarget = "192.168.2.116:5005",
    [string]$RequiredWindowsSourceIp = "192.168.2.130",
    [string]$UbuntuRosDistro = "",
    [string]$UbuntuOrbbecSetup = "",
    [string]$UbuntuLerobotPython = "",
    [string]$UbuntuDatasetDataRoot = "",
    [ValidateRange(1.0, 100.0)]
    [double]$RateHz = 100.0,
    [string]$SessionName = "full_teleop",
    [string]$Task = "",
    [string]$Operator = "Lucky",
    [string]$DatasetRepoId = "local/onearm_tele"
)

$ErrorActionPreference = "Stop"
$script:RemoteStackStarted = $false
$script:SenderProcess = $null
$script:ReadyFile = $null
$script:NormalSenderExit = $false
$script:LaunchFailed = $false
$script:DatasetCaptureStarted = $false
$script:EpisodeRecordingStarted = $false

function Invoke-CheckedSsh {
    param([Parameter(Mandatory = $true)][string]$RemoteCommand)

    & ssh.exe `
        -o BatchMode=yes `
        -o ConnectTimeout=8 `
        -o ServerAliveInterval=15 `
        -o ServerAliveCountMax=8 `
        -o TCPKeepAlive=yes `
        $UbuntuHost `
        $RemoteCommand
    if ($LASTEXITCODE -ne 0) {
        throw "SSH command failed with exit code $LASTEXITCODE."
    }
}

function Get-UdpSourceAddress {
    param([Parameter(Mandatory = $true)][string]$Target)

    $hostPart = ($Target -split ":", 2)[0]
    $client = [System.Net.Sockets.UdpClient]::new()
    try {
        $client.Connect($hostPart, 5005)
        return ([System.Net.IPEndPoint]$client.Client.LocalEndPoint).Address.ToString()
    }
    finally {
        $client.Dispose()
    }
}

function Assert-MotionNetworkPath {
    param(
        [Parameter(Mandatory = $true)][string]$Target,
        [Parameter(Mandatory = $true)][string]$SourceIp
    )

    $localAddress = Get-NetIPAddress `
        -AddressFamily IPv4 `
        -IPAddress $SourceIp `
        -ErrorAction SilentlyContinue
    if ($null -eq $localAddress) {
        throw (
            "Required motion source address {0} is not configured on Windows. " +
            "No camera or robot process was started."
        ) -f $SourceIp
    }

    $hostPart = ($Target -split ":", 2)[0]
    & ping.exe -n 2 -w 1000 -S $SourceIp $hostPart *> $null
    if ($LASTEXITCODE -ne 0) {
        throw (
            "Wired motion path {0} -> {1} is unreachable. No camera or " +
            "robot process was started. Check the cable/switch/Ubuntu " +
            "Ethernet link; do not fall back to Wi-Fi for teleoperation."
        ) -f $SourceIp, $hostPart
    }
}

function Stop-RemoteStack {
    if (-not $script:RemoteStackStarted) {
        return
    }
    Write-Host ""
    Write-Host "Stopping Ubuntu mapping, servo mode, gripper, robot enable and power..."
    & ssh.exe `
        -o BatchMode=yes `
        -o ConnectTimeout=8 `
        -o ServerAliveInterval=15 `
        -o ServerAliveCountMax=8 `
        -o TCPKeepAlive=yes `
        $UbuntuHost `
        "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh stop"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Ubuntu did not fully confirm shutdown. Press the physical emergency stop and inspect the Ubuntu logs."
    }
    $script:RemoteStackStarted = $false
}

function Stop-DatasetCapture {
    if (-not $script:DatasetCaptureStarted) {
        return
    }
    Write-Host ""
    Write-Host "Stopping passive episode recorder and the two dataset cameras..."
    & ssh.exe `
        -o BatchMode=yes `
        -o ConnectTimeout=8 `
        -o ServerAliveInterval=15 `
        -o ServerAliveCountMax=8 `
        -o TCPKeepAlive=yes `
        $UbuntuHost `
        "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh stop"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning (
            "Ubuntu did not fully confirm dataset capture shutdown. " +
            "Inspect tools/ubuntu_dataset_episode.sh status."
        )
    }
    $script:DatasetCaptureStarted = $false
}

function ConvertTo-Utf8Base64 {
    param([Parameter(Mandatory = $true)][AllowEmptyString()][string]$Value)
    return [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($Value))
}

function ConvertTo-BashSingleQuoted {
    param([Parameter(Mandatory = $true)][AllowEmptyString()][string]$Value)

    if ($Value.Contains("'")) {
        throw "Remote shell values may not contain single quotes: $Value"
    }
    return "'" + $Value + "'"
}

function Get-RemoteEnvironmentPrefix {
    $pairs = [ordered]@{}
    if (-not [string]::IsNullOrWhiteSpace($UbuntuRosDistro)) {
        $pairs["ROS_DISTRO"] = $UbuntuRosDistro
    }
    if (-not [string]::IsNullOrWhiteSpace($UbuntuOrbbecSetup)) {
        $pairs["ONE_ARM_ORBBEC_SETUP"] = $UbuntuOrbbecSetup
    }
    if (-not [string]::IsNullOrWhiteSpace($UbuntuLerobotPython)) {
        $pairs["ONE_ARM_LEROBOT_PYTHON"] = $UbuntuLerobotPython
    }
    if (-not [string]::IsNullOrWhiteSpace($UbuntuDatasetDataRoot)) {
        $pairs["ONE_ARM_DATASET_DATA_ROOT"] = $UbuntuDatasetDataRoot
    }
    if ($pairs.Count -eq 0) {
        return ""
    }

    $assignments = foreach ($key in $pairs.Keys) {
        $key + "=" + (ConvertTo-BashSingleQuoted -Value $pairs[$key])
    }
    return ($assignments -join " ") + " "
}

$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$recorder = Join-Path $PSScriptRoot "run_zlink2_recorder.cmd"
if (-not (Test-Path -LiteralPath $recorder -PathType Leaf)) {
    throw "Recorder launcher not found: $recorder"
}

$targetParts = $UbuntuUdpTarget -split ":", 2
if ($targetParts.Count -ne 2) {
    throw "UbuntuUdpTarget must be HOST:PORT."
}
if ([string]::IsNullOrWhiteSpace($RequiredWindowsSourceIp)) {
    $windowsSourceIp = Get-UdpSourceAddress -Target $UbuntuUdpTarget
}
else {
    $windowsSourceIp = $RequiredWindowsSourceIp
}
Assert-MotionNetworkPath `
    -Target $UbuntuUdpTarget `
    -SourceIp $windowsSourceIp
if ([string]::IsNullOrEmpty($Task)) {
    $Task = Read-Host "Task prompt (exact text stored in LeRobot)"
}
if ([string]::IsNullOrWhiteSpace($Task)) {
    throw "Task prompt must not be empty."
}
if ($Task -ne $Task.Trim()) {
    throw "Task prompt must not contain leading or trailing whitespace."
}
if ($SessionName -notmatch "^[A-Za-z0-9_-]+$") {
    throw "SessionName may contain only A-Z, a-z, 0-9, _ or -."
}
if ($DatasetRepoId -notmatch "^[^/\s]+/[^/\s]+$") {
    throw "DatasetRepoId must have owner/name form."
}
$taskBase64 = ConvertTo-Utf8Base64 -Value $Task
$operatorBase64 = ConvertTo-Utf8Base64 -Value $Operator
$remoteProject = ConvertTo-BashSingleQuoted -Value $UbuntuProject
$remoteEnvironment = Get-RemoteEnvironmentPrefix
$script:ReadyFile = Join-Path `
    ([System.IO.Path]::GetTempPath()) `
    ("one_arm_teleop_ready_{0}.flag" -f $PID)
if (Test-Path -LiteralPath $script:ReadyFile) {
    Remove-Item -LiteralPath $script:ReadyFile -Force
}

Write-Host "============================================================"
Write-Host "One-Arm Teleoperation / PowerShell launcher"
Write-Host "Windows source IP : $windowsSourceIp"
Write-Host "SSH control path  : $UbuntuHost"
Write-Host "UDP motion target : $UbuntuUdpTarget"
Write-Host "Ubuntu project    : $UbuntuProject"
Write-Host "ZLink2 port       : $ComPort"
Write-Host "Task              : $Task"
Write-Host "Dataset repo id   : $DatasetRepoId"
if (-not [string]::IsNullOrWhiteSpace($UbuntuDatasetDataRoot)) {
    Write-Host "Dataset root      : $UbuntuDatasetDataRoot"
}
Write-Host "============================================================"
Write-Host "Stage 0 starts only the head/right-wrist RGB cameras and checks 30 FPS."
Write-Host "The two chest cameras are excluded. The robot will NOT move."

try {
    # Treat the result as uncertain until SSH finishes. Cleanup is safe even
    # when startup stopped at the SSD/camera preflight.
    $script:DatasetCaptureStarted = $true
    Invoke-CheckedSsh `
        "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh start"

    Write-Host ""
    Write-Host "Stage 1 starts ROS 2 robot interfaces only. The robot will NOT move."
    # Treat the result as uncertain until the remote start command finishes.
    # If SSH drops after creating any PID file, finally will still issue stop.
    $script:RemoteStackStarted = $true
    Invoke-CheckedSsh `
        "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh start '$windowsSourceIp'"

    Write-Host ""
    Write-Host "Starting passive ROS bag recording..."
    $script:EpisodeRecordingStarted = $true
    Invoke-CheckedSsh `
        "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh record-start '$SessionName' '$taskBase64' '$operatorBase64'"

    $arguments = @(
        "--port", $ComPort,
        "--rate-hz", $RateHz.ToString(
            [System.Globalization.CultureInfo]::InvariantCulture
        ),
        "--udp-target", $UbuntuUdpTarget,
        "--udp-bind-host", $windowsSourceIp,
        "--deadman",
        "--activation-file", $script:ReadyFile,
        "--session-name", $SessionName
    )
    $quotedArguments = foreach ($argument in $arguments) {
        "'" + ($argument -replace "'", "''") + "'"
    }
    $quotedRecorder = "'" + ($recorder -replace "'", "''") + "'"
    $senderCommand = @'
$Host.UI.RawUI.WindowTitle = "ZLink2 Teleoperation Sender"
& __RECORDER__ __ARGUMENTS__
exit $LASTEXITCODE
'@
    $senderCommand = $senderCommand.Replace("__RECORDER__", $quotedRecorder)
    $senderCommand = $senderCommand.Replace(
        "__ARGUMENTS__",
        ($quotedArguments -join " ")
    )
    $encodedSenderCommand = [Convert]::ToBase64String(
        [Text.Encoding]::Unicode.GetBytes($senderCommand)
    )

    Write-Host ""
    Write-Host "A dedicated ZLink2 sender window will open now."
    Write-Host "In that window, place the leader arm comfortably and press Enter once."
    Write-Host "Do not press Space until it displays REMOTE STACK READY."
    $script:SenderProcess = Start-Process `
        -FilePath "powershell.exe" `
        -ArgumentList @(
            "-NoProfile",
            "-ExecutionPolicy", "Bypass",
            "-EncodedCommand", $encodedSenderCommand
        ) `
        -WorkingDirectory $repoRoot `
        -WindowStyle Normal `
        -PassThru

    Write-Host ""
    Write-Host "Stage 2 is waiting for Enter/baseline, fresh UDP preview, and robot checks."
    Write-Host "After these checks, the robot WILL be powered, enabled, and put in servo mode,"
    Write-Host "but it still receives no motion target until you press Space."
    Invoke-CheckedSsh `
        "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_full_teleop_stack.sh arm"

    Set-Content `
        -LiteralPath $script:ReadyFile `
        -Value "FULL_TELEOP_READY" `
        -Encoding Ascii
    Write-Host ""
    Write-Host "READY gate opened. Follow the sender window:"
    Write-Host "  Space      start teleoperation"
    Write-Host "  Space again / Esc / Ctrl+C   STOP"
    Write-Host "Keep the physical emergency stop within reach."

    $script:SenderProcess.WaitForExit()
    if ($script:SenderProcess.ExitCode -ne 0) {
        throw "ZLink2 sender exited with code $($script:SenderProcess.ExitCode)."
    }
    $script:NormalSenderExit = $true
}
catch {
    $script:LaunchFailed = $true
    Write-Host ""
    Write-Host ("ERROR: " + $_.Exception.Message) -ForegroundColor Red
}
finally {
    if (
        $null -ne $script:SenderProcess -and
        -not $script:SenderProcess.HasExited -and
        -not $script:NormalSenderExit
    ) {
        & taskkill.exe /PID $script:SenderProcess.Id /T /F 2>$null | Out-Null
    }
    Stop-RemoteStack
    Stop-DatasetCapture
    if ($null -ne $script:ReadyFile -and (Test-Path -LiteralPath $script:ReadyFile)) {
        Remove-Item -LiteralPath $script:ReadyFile -Force
    }
}

if ($script:EpisodeRecordingStarted) {
    $outcome = "failure"
    if ($script:NormalSenderExit -and -not $script:LaunchFailed) {
        while ($true) {
            $answer = (
                Read-Host "Episode outcome: S=success, F=failure"
            ).Trim().ToLowerInvariant()
            if ($answer -in @("s", "success")) {
                $outcome = "success"
                break
            }
            if ($answer -in @("f", "failure")) {
                $outcome = "failure"
                break
            }
            Write-Host "Please enter S or F."
        }
    }
    else {
        Write-Host (
            "The session did not end normally; it will be marked failure " +
            "and kept only as a raw diagnostic episode."
        )
    }
    try {
        Invoke-CheckedSsh `
            "cd $remoteProject && ${remoteEnvironment}bash tools/ubuntu_dataset_episode.sh finalize '$outcome' '$DatasetRepoId'"
    }
    catch {
        $script:LaunchFailed = $true
        Write-Host ""
        Write-Host (
            "DATASET FINALIZATION ERROR: " + $_.Exception.Message
        ) -ForegroundColor Red
        Write-Host "The raw ROS bag was preserved and can be exported later."
    }
}

if ($script:LaunchFailed -or -not $script:NormalSenderExit) {
    exit 1
}
Write-Host ""
Write-Host "Teleoperation ended; robot shutdown and dataset capture stop were confirmed."
