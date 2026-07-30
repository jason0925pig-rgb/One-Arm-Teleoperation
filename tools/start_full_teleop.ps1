[CmdletBinding()]
param(
    [string]$ComPort = "COM10",
    [string]$UbuntuHost = "armstrong-host",
    [string]$UbuntuProject = "/home/tele/onearm_teleop/One-Arm-Teleoperation",
    [string]$UbuntuUdpTarget = "192.168.0.36:5005",
    [ValidateRange(1.0, 100.0)]
    [double]$RateHz = 100.0,
    [string]$SessionName = "full_teleop"
)

$ErrorActionPreference = "Stop"
$script:RemoteStackStarted = $false
$script:SenderProcess = $null
$script:ReadyFile = $null
$script:NormalSenderExit = $false
$script:LaunchFailed = $false

function Invoke-CheckedSsh {
    param([Parameter(Mandatory = $true)][string]$RemoteCommand)

    & ssh.exe `
        -o BatchMode=yes `
        -o ConnectTimeout=8 `
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

function Stop-RemoteStack {
    if (-not $script:RemoteStackStarted) {
        return
    }
    Write-Host ""
    Write-Host "Stopping Ubuntu mapping, servo mode, gripper, robot enable and power..."
    & ssh.exe `
        -o BatchMode=yes `
        -o ConnectTimeout=8 `
        $UbuntuHost `
        "cd '$UbuntuProject' && bash tools/ubuntu_full_teleop_stack.sh stop"
    if ($LASTEXITCODE -ne 0) {
        Write-Warning "Ubuntu did not fully confirm shutdown. Press the physical emergency stop and inspect the Ubuntu logs."
    }
    $script:RemoteStackStarted = $false
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
$windowsSourceIp = Get-UdpSourceAddress -Target $UbuntuUdpTarget
$script:ReadyFile = Join-Path `
    ([System.IO.Path]::GetTempPath()) `
    ("one_arm_teleop_ready_{0}.flag" -f $PID)
if (Test-Path -LiteralPath $script:ReadyFile) {
    Remove-Item -LiteralPath $script:ReadyFile -Force
}

Write-Host "============================================================"
Write-Host "One-Arm Teleoperation / PowerShell launcher"
Write-Host "Windows source IP : $windowsSourceIp"
Write-Host "Ubuntu SSH target : $UbuntuHost"
Write-Host "ZLink2 port       : $ComPort"
Write-Host "============================================================"
Write-Host "Stage 1 starts ROS 2 processes only. The robot will NOT move."

try {
    # Treat the result as uncertain until the remote start command finishes.
    # If SSH drops after creating any PID file, finally will still issue stop.
    $script:RemoteStackStarted = $true
    Invoke-CheckedSsh `
        "cd '$UbuntuProject' && bash tools/ubuntu_full_teleop_stack.sh start '$windowsSourceIp'"

    $arguments = @(
        "--port", $ComPort,
        "--rate-hz", $RateHz.ToString(
            [System.Globalization.CultureInfo]::InvariantCulture
        ),
        "--udp-target", $UbuntuUdpTarget,
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
        "cd '$UbuntuProject' && bash tools/ubuntu_full_teleop_stack.sh arm"

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
    if ($null -ne $script:ReadyFile -and (Test-Path -LiteralPath $script:ReadyFile)) {
        Remove-Item -LiteralPath $script:ReadyFile -Force
    }
}

if ($script:LaunchFailed -or -not $script:NormalSenderExit) {
    exit 1
}
Write-Host ""
Write-Host "Teleoperation session ended and the Ubuntu shutdown sequence was requested."
