$ErrorActionPreference = "Stop"
$path = Join-Path $PSScriptRoot "calibration.json"
$c = Get-Content $path -Raw | ConvertFrom-Json
if ($c.title -or $c.status -ne "success") { Write-Host "calibration.json is not a successful calibration. Run steps 1-2 first." -ForegroundColor Red; exit 1 }
if (-not $c.hardIronCorrection -or -not $c.softIronCorrection) { Write-Host "Correction fields missing in calibration.json" -ForegroundColor Red; exit 1 }
$out = @{ hardIronCorrection = $c.hardIronCorrection; softIronCorrection = $c.softIronCorrection } | ConvertTo-Json -Compress
Set-Content -Path (Join-Path $PSScriptRoot "apply.json") -Value $out -Encoding ascii
Write-Host "apply.json written:"
Write-Host $out
