$ErrorActionPreference = "Stop"
$path = Join-Path $PSScriptRoot "settings_check.json"
$c = Get-Content $path -Raw | ConvertFrom-Json
Write-Host ("useMagData         : " + $c.useMagData)
Write-Host ("hardIronCorrection : " + ($c.hardIronCorrection -join ", "))
Write-Host ("softIronCorrection : " + ($c.softIronCorrection -join ", "))
$hardOk = ($c.hardIronCorrection | Where-Object { [math]::Abs($_) -gt 1e-9 }).Count -gt 0
$useOk = ($c.useMagData -eq "auto") -or ($c.useMagData -eq "always")
if ($hardOk -and $useOk) { Write-Host "VERIFIED: calibration stored and mag fusion enabled." -ForegroundColor Green }
elseif (-not $hardOk) { Write-Host "WARNING: hard iron correction is still all zeros, calibration not applied." -ForegroundColor Yellow }
else { Write-Host "WARNING: useMagData is not auto/always, mag fusion still disabled." -ForegroundColor Yellow }
