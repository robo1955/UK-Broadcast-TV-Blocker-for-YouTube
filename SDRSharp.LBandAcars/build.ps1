$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $root

# The round-trip test targets net10. The plugin targets net9, which is what SDR# loads.
$dotnet10 = "dotnet"
$dotnet9 = Join-Path $env:USERPROFILE ".dotnet\dotnet.exe"
if (-not (Test-Path $dotnet9)) { $dotnet9 = "dotnet" }

& $dotnet10 run --project (Join-Path $root "src\SDRSharp.LBandAcars.Tests") --configuration Release
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

& $dotnet9 build (Join-Path $root "src\SDRSharp.LBandAcars\SDRSharp.LBandAcars.csproj") --configuration Release
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

$dll = Join-Path $root "src\SDRSharp.LBandAcars\bin\Release\net9.0-windows\SDRSharp.LBandAcars.dll"
if (-not (Test-Path $dll)) { throw "Plugin DLL was not produced: $dll" }
Write-Host "Plugin: $dll"
