$ErrorActionPreference = "Stop"
$path = Join-Path $PSScriptRoot "calibration.json"
if (!(Test-Path $path)) { Write-Host "calibration.json not found. Run step 2 first." -ForegroundColor Red; exit 1 }
$c = Get-Content $path -Raw | ConvertFrom-Json
if ($c.title) {
    Write-Host ("COMPUTE FAILED: " + $c.detail) -ForegroundColor Red
    Write-Host "Not enough valid motion or no mag data received yet."
    Write-Host "Keep rotating / flying the pattern, then run step 2 again."
    exit 1
}
Write-Host ("status  : " + $c.status)
Write-Host ("quality : " + $c.quality + "   trust: " + $c.trust)
Write-Host ("points  : used " + $c.numPointsUsed + " / total " + $c.numPointsTotal + " (max " + $c.maxNumPoints + ")")
Write-Host ("motion  : roll=" + $c.rollMotionValid + " pitch=" + $c.pitchMotionValid + " yaw=" + $c.yawMotionValid + " enoughPts=" + $c.enoughPts)
$ok = ($c.status -eq "success") -and $c.enoughPts -and $c.rollMotionValid -and $c.pitchMotionValid -and $c.yawMotionValid
if ($ok) {
    Write-Host "CALIBRATION GOOD. You can run step 3 (apply)." -ForegroundColor Green
    exit 0
} else {
    Write-Host "NOT READY. Continue the motion pattern and run step 2 again." -ForegroundColor Yellow
    Write-Host "(Computing does not stop the acquisition, more data keeps improving it.)"
    exit 1
}
