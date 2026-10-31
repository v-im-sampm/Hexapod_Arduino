#Requires -Version 5.1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false

$scriptPath = Join-Path (Split-Path -Parent $PSScriptRoot) 'tools\flash.ps1'
$fakeCli = Join-Path $PSScriptRoot 'fixtures\arduino-cli.cmd'
$testDirectory = Join-Path ([IO.Path]::GetTempPath()) ("hexapod-flash-test-" + [guid]::NewGuid().ToString())
$null = New-Item -ItemType Directory -Path $testDirectory
$log = Join-Path $testDirectory 'calls.jsonl'
$config = Join-Path $testDirectory 'arduino config.yaml'
Set-Content -LiteralPath $config -Value '# Test configuration; never read by a real CLI.'
$variableNames = @('HEXAPOD_CLI_TEST_LOG', 'HEXAPOD_CLI_TEST_FAIL', 'HEXAPOD_CLI_TEST_INPUT')
$previous = @{}
foreach ($name in $variableNames) {
    $previous[$name] = [Environment]::GetEnvironmentVariable($name, 'Process')
}

function Assert-True {
    param([bool]$Condition, [string]$Message)
    if (-not $Condition) {
        throw $Message
    }
}

function Run-Case {
    param(
        [string[]]$Options,
        [string[]]$ExpectedCommands,
        [string]$Fail = '',
        [bool]$Success = $true,
        [bool]$CheckInput = $false
    )
    if (Test-Path -LiteralPath $log) {
        Remove-Item -LiteralPath $log
    }
    $env:HEXAPOD_CLI_TEST_LOG = $log
    $env:HEXAPOD_CLI_TEST_FAIL = $Fail
    $env:HEXAPOD_CLI_TEST_INPUT = if ($CheckInput) { '1' } else { '0' }
    $arguments = @(
        '-NoProfile', '-ExecutionPolicy', 'Bypass', '-File', $scriptPath,
        '-ArduinoCli', $fakeCli, '-ConfigFile', $config
    ) + $Options
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo.FileName = (Get-Command powershell.exe -CommandType Application).Source
    $process.StartInfo.Arguments = ($arguments | ForEach-Object { '"' + $_ + '"' }) -join ' '
    $process.StartInfo.UseShellExecute = $false
    $process.StartInfo.RedirectStandardInput = $true
    $process.StartInfo.RedirectStandardOutput = $true
    $process.StartInfo.RedirectStandardError = $true
    try {
        $null = $process.Start()
        $stdout = $process.StandardOutput.ReadToEndAsync()
        $stderr = $process.StandardError.ReadToEndAsync()
        if ($CheckInput) {
            $process.StandardInput.WriteLine('help')
        }
        $process.StandardInput.Close()
        if (-not $process.WaitForExit(60000)) {
            Stop-Process -Id $process.Id
            throw "Test process timed out for $Options"
        }
        $output = $stdout.GetAwaiter().GetResult() + $stderr.GetAwaiter().GetResult()
        $exitCode = $process.ExitCode
    } finally {
        $process.Dispose()
    }
    Assert-True (($exitCode -eq 0) -eq $Success) "Unexpected exit $exitCode for $Options`n$output"
    $calls = @()
    if (Test-Path -LiteralPath $log) {
        $calls = @(Get-Content -LiteralPath $log | ForEach-Object { $_ | ConvertFrom-Json })
    }
    $commands = @($calls | ForEach-Object { $_.command })
    Assert-True (($commands -join ',') -eq ($ExpectedCommands -join ',')) "Incorrect command sequence: $commands"
    foreach ($call in $calls) {
        $configIndex = [Array]::IndexOf($call.arguments, '--config-file')
        Assert-True ($configIndex -ge 0 -and $call.arguments[$configIndex + 1] -eq $config) 'Configuration path with spaces was not preserved.'
        if ($call.command -ne 'compile') {
            $portIndex = [Array]::IndexOf($call.arguments, '--port')
            Assert-True ($portIndex -ge 0 -and $call.arguments[$portIndex + 1] -eq 'COM5') 'COM port was not passed correctly.'
        }
        if ($call.command -eq 'monitor') {
            Assert-True ($call.arguments -contains 'baudrate=115200') 'Monitor baud rate is missing.'
            Assert-True (-not ($call.arguments -contains '--raw')) 'Monitor should allow normal line input.'
            if ($CheckInput) {
                Assert-True ($call.input -eq 'help') 'Keyboard/stdin input did not reach the monitor.'
            }
        }
    }
    if ($commands -contains 'upload') {
        $buildArgs = $calls[0].arguments
        $uploadArgs = $calls[1].arguments
        Assert-True ($buildArgs[[Array]::IndexOf($buildArgs, '--build-path') + 1] -eq
                     $uploadArgs[[Array]::IndexOf($uploadArgs, '--input-dir') + 1]) 'Upload did not use the freshly built artifacts.'
    }
}

try {
    Run-Case -Options @('-BuildOnly') -ExpectedCommands @('compile')
    Run-Case -Options @('-Port', 'com5') -ExpectedCommands @('compile', 'upload', 'monitor')
    Run-Case -Options @('-Port', 'COM5') -ExpectedCommands @('compile') -Fail compile -Success $false
    Run-Case -Options @('-Port', 'COM5') -ExpectedCommands @('compile', 'upload') -Fail upload -Success $false
    Run-Case -Options @('-Port', 'COM5') -ExpectedCommands @('compile', 'upload', 'monitor') -Fail monitor -Success $false
    Run-Case -Options @('-MonitorOnly', '-Port', 'COM5') -ExpectedCommands @('monitor') -CheckInput $true
    Run-Case -Options @('-Port', 'COM0') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-Port', 'COM5;echo bad') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-BuildOnly', '-MonitorOnly') -ExpectedCommands @() -Success $false
    Write-Host 'PASS: flash stages, artifact paths, failure handling, COM validation and interactive monitor stdin.'
} finally {
    foreach ($name in $variableNames) {
        [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process')
    }
    Remove-Item -LiteralPath $testDirectory -Recurse
}
