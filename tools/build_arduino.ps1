param(
  [Parameter(Mandatory = $true)]
  [string]$Fqbn,
  [switch]$RealInputs
)

$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
$sketchName = 'PalpeurTRAXX'
$staging = Join-Path $root "build\arduino\$sketchName"

if (-not (Get-Command arduino-cli -ErrorAction SilentlyContinue)) {
  throw 'arduino-cli est requis. Installer le paquet Industrial Shields du PLC cible avant le build.'
}
$arduinoLibraries = arduino-cli lib list 2>$null | Out-String
if ($arduinoLibraries -notmatch 'ArduinoJson') {
  throw 'Installer ArduinoJson 6 avec: arduino-cli lib install ArduinoJson@6.21.5'
}

New-Item -ItemType Directory -Force -Path $staging | Out-Null
Copy-Item -LiteralPath (Join-Path $root 'firmware\PalpeurTRAXX.ino') -Destination (Join-Path $staging "$sketchName.ino") -Force
Copy-Item -LiteralPath (Join-Path $root 'src') -Destination $staging -Recurse -Force
Copy-Item -LiteralPath (Join-Path $root 'data') -Destination $staging -Recurse -Force

$simulationFlag = if ($RealInputs) { '-DPALPEUR_SIMULATION=0' } else { '-DPALPEUR_SIMULATION=1' }
arduino-cli compile --fqbn $Fqbn --build-property "compiler.cpp.extra_flags=$simulationFlag" $staging
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }