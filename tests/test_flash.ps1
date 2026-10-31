#Requires -Version 5.1
Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$PSNativeCommandUseErrorActionPreference = $false

$repoRoot = Split-Path -Parent $PSScriptRoot
$scriptPath = Join-Path $repoRoot 'tools\flash.ps1'
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
        [string]$ExpectedPort = 'COM5',
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
            Assert-True ($portIndex -ge 0 -and $call.arguments[$portIndex + 1] -eq $ExpectedPort) 'COM port was not passed correctly.'
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
        $uploadArgs = ($calls | Where-Object { $_.command -eq 'upload' }).arguments
        $inputIndex = [Array]::IndexOf($uploadArgs, '--input-dir')
        Assert-True ($inputIndex -ge 0 -and $uploadArgs[$inputIndex + 1] -eq
                     (Join-Path $repoRoot '.build\esp32')) 'Upload did not use the cached build directory.'
        if ($commands -contains 'compile') {
            $buildArgs = ($calls | Where-Object { $_.command -eq 'compile' }).arguments
            Assert-True ($buildArgs[[Array]::IndexOf($buildArgs, '--build-path') + 1] -eq
                         $uploadArgs[$inputIndex + 1]) 'Upload did not use the freshly built artifacts.'
        }
    }
}

try {
    $tasks = Get-Content -LiteralPath (Join-Path $repoRoot '.vscode\tasks.json') -Raw | ConvertFrom-Json
    $flashTasks = @($tasks.tasks | Where-Object { $_.label -eq 'Hexapod: Flash + Serial Console' })
    Assert-True ($flashTasks.Count -eq 1) 'Flash + Serial Console task is missing or duplicated.'
    $flashTask = $flashTasks[0]
    Assert-True ($flashTask.args -contains '${workspaceFolder}\tools\flash.ps1') 'Flash + Serial Console task does not use the flash script.'
    $flashPortIndex = [Array]::IndexOf($flashTask.args, '-Port')
    Assert-True ($flashPortIndex -ge 0 -and $flashTask.args[$flashPortIndex + 1] -eq 'COM12') 'Flash + Serial Console task must use COM12 without prompting.'
    $flashOptions = $flashTask.args[$flashPortIndex..($flashTask.args.Count - 1)]
    Run-Case -Options $flashOptions -ExpectedCommands @('compile', 'upload', 'monitor') -ExpectedPort 'COM12' -CheckInput $true
    $uploadTasks = @($tasks.tasks | Where-Object { $_.label -eq 'Hexapod: Upload Only (COM12)' })
    Assert-True ($uploadTasks.Count -eq 1) 'Upload-only task is missing or duplicated.'
    $uploadTask = $uploadTasks[0]
    $uploadOnlyIndex = [Array]::IndexOf($uploadTask.args, '-UploadOnly')
    Assert-True ($uploadOnlyIndex -ge 0) 'Upload-only task does not skip building.'
    Assert-True ($uploadTask.args -contains '${workspaceFolder}\tools\flash.ps1') 'Upload-only task does not use the flash script.'
    Assert-True (-not ($uploadTask.PSObject.Properties.Name -contains 'dependsOn')) 'Upload-only task must not depend on a build.'
    $uploadOptions = $uploadTask.args[$uploadOnlyIndex..($uploadTask.args.Count - 1)]
    Run-Case -Options $uploadOptions -ExpectedCommands @('upload') -ExpectedPort 'COM12'
    Run-Case -Options $uploadOptions -ExpectedCommands @('upload') -ExpectedPort 'COM12' -Fail upload -Success $false
    Run-Case -Options @('-BuildOnly') -ExpectedCommands @('compile')
    Run-Case -Options @('-Port', 'com5') -ExpectedCommands @('compile', 'upload', 'monitor')
    Run-Case -Options @('-Port', 'COM5') -ExpectedCommands @('compile') -Fail compile -Success $false
    Run-Case -Options @('-Port', 'COM5') -ExpectedCommands @('compile', 'upload') -Fail upload -Success $false
    Run-Case -Options @('-Port', 'COM5') -ExpectedCommands @('compile', 'upload', 'monitor') -Fail monitor -Success $false
    Run-Case -Options @('-MonitorOnly', '-Port', 'COM5') -ExpectedCommands @('monitor') -CheckInput $true
    Run-Case -Options @('-Port', 'COM0') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-Port', 'COM5;echo bad') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-BuildOnly', '-MonitorOnly') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-UploadOnly', '-Port', 'COM0') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-UploadOnly', '-BuildOnly') -ExpectedCommands @() -Success $false
    Run-Case -Options @('-UploadOnly', '-MonitorOnly') -ExpectedCommands @() -Success $false
    Write-Host 'PASS: flash stages, COM12 flash/console and upload-only tasks, artifact paths, failure handling, COM validation and interactive monitor stdin.'
} finally {
    foreach ($name in $variableNames) {
        [Environment]::SetEnvironmentVariable($name, $previous[$name], 'Process')
    }
    Remove-Item -LiteralPath $testDirectory -Recurse
}
