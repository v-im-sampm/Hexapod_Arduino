$ErrorActionPreference = 'Stop'

if (-not $env:HEXAPOD_CLI_TEST_LOG) {
    throw 'This fake CLI must only be used by test_flash.ps1.'
}

$command = @($args | Where-Object { $_ -in @('compile', 'upload', 'monitor') })
if ($command.Count -ne 1) {
    throw "Unexpected fake CLI arguments: $args"
}
$entry = @{
    command = $command[0]
    arguments = @($args)
}
if ($command[0] -eq 'monitor' -and $env:HEXAPOD_CLI_TEST_INPUT -eq '1') {
    $entry.input = [Console]::ReadLine()
}
$entry | ConvertTo-Json -Compress | Add-Content -LiteralPath $env:HEXAPOD_CLI_TEST_LOG
if ($command[0] -eq $env:HEXAPOD_CLI_TEST_FAIL) {
    [Console]::Error.WriteLine("Simulated $($command[0]) failure.")
    exit 23
}
exit 0
