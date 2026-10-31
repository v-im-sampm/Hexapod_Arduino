#Requires -Version 5.1

[CmdletBinding(DefaultParameterSetName = 'Flash')]
param(
    [Parameter(ParameterSetName = 'Flash')]
    [Parameter(ParameterSetName = 'Upload')]
    [Parameter(ParameterSetName = 'Monitor')]
    [string]$Port,

    [Parameter(Mandatory = $true, ParameterSetName = 'Build')]
    [switch]$BuildOnly,

    [Parameter(Mandatory = $true, ParameterSetName = 'Upload')]
    [switch]$UploadOnly,

    [Parameter(Mandatory = $true, ParameterSetName = 'Monitor')]
    [switch]$MonitorOnly,

    [string]$ArduinoCli,
    [string]$ConfigFile
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false

$repoRoot = Split-Path -Parent $PSScriptRoot
$buildPath = Join-Path $repoRoot '.build\esp32'
$fqbn = 'esp32:esp32:esp32'

if (-not $BuildOnly) {
    if ([string]::IsNullOrWhiteSpace($Port)) {
        $Port = Read-Host 'ESP32 COM port (for example COM5)'
    }
    $Port = $Port.Trim().ToUpperInvariant()
    if ($Port -notmatch '^COM[1-9][0-9]*$') {
        throw 'Invalid serial port. Specify a Windows COM port, for example -Port COM5.'
    }
}

if (-not $ArduinoCli) {
    $installed = Get-Command arduino-cli -CommandType Application -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if ($installed) {
        $ArduinoCli = $installed.Source
    } else {
        $candidates = @(
            (Join-Path $env:LOCALAPPDATA 'Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe'),
            (Join-Path $env:ProgramFiles 'Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe')
        )
        foreach ($candidate in $candidates) {
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                $ArduinoCli = $candidate
                break
            }
        }
    }
}
if (-not $ArduinoCli -or -not (Test-Path -LiteralPath $ArduinoCli -PathType Leaf)) {
    throw 'Arduino CLI not found. Install Arduino IDE 2.x / Arduino CLI, or provide -ArduinoCli with its full path.'
}
$ArduinoCli = (Get-Item -LiteralPath $ArduinoCli).FullName

if (-not $ConfigFile) {
    $ideConfig = Join-Path $env:USERPROFILE '.arduinoIDE\arduino-cli.yaml'
    if (Test-Path -LiteralPath $ideConfig -PathType Leaf) {
        $ConfigFile = $ideConfig
    }
}
$cliConfig = @()
if ($ConfigFile) {
    if (-not (Test-Path -LiteralPath $ConfigFile -PathType Leaf)) {
        throw "Arduino CLI configuration not found: $ConfigFile"
    }
    $cliConfig = @('--config-file', (Get-Item -LiteralPath $ConfigFile).FullName)
}

function Invoke-Arduino {
    param([string]$Stage, [string[]]$Arguments)

    Write-Host "`n--- $Stage ---"
    # Keep native stdin/stdout attached to the terminal, especially for the monitor.
    & $ArduinoCli @cliConfig @Arguments
    if ($LASTEXITCODE -ne 0) {
        throw "$Stage failed (Arduino CLI exit code $LASTEXITCODE). See the output above."
    }
}

Write-Host "Arduino CLI: $ArduinoCli"
if ($ConfigFile) {
    Write-Host "Configuration: $ConfigFile"
}

if (-not $MonitorOnly) {
    if (-not $UploadOnly) {
        Invoke-Arduino -Stage 'Build' -Arguments @(
            'compile', '--fqbn', $fqbn, '--build-path', $buildPath, $repoRoot
        )
        if ($BuildOnly) {
            Write-Host "`nBuild complete: $buildPath"
            return
        }
    } else {
        Write-Host "`nUploading existing artifacts from $buildPath; source changes are not rebuilt."
    }

    Write-Host "`nKeep servo power disconnected. Close any other monitor using $Port."
    Invoke-Arduino -Stage 'Upload' -Arguments @(
        'upload', '--fqbn', $fqbn, '--port', $Port, '--protocol', 'serial',
        '--input-dir', $buildPath, '--discovery-timeout', '5s', $repoRoot
    )
    if ($UploadOnly) {
        Write-Host "`nUpload complete on $Port."
        return
    }
}

Write-Host "`nSerial console on $Port at 115200 baud."
Write-Host 'Type help and press Enter. Send off + Enter before leaving; Ctrl+C closes the monitor.'
Write-Host 'Input is line-buffered: even ! needs Enter in this terminal mode.'
Invoke-Arduino -Stage 'Serial monitor' -Arguments @(
    'monitor', '--fqbn', $fqbn, '--port', $Port, '--protocol', 'serial',
    '--config', 'baudrate=115200', '--discovery-timeout', '5s'
)
