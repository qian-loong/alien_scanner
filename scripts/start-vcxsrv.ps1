# Start VcXsrv for Docker X11 (display :0, disable access control)
$VcXsrv = "E:\VcXsrv\vcxsrv.exe"
if (-not (Test-Path $VcXsrv)) {
    Write-Error "VcXsrv not found: $VcXsrv"
    exit 1
}

$running = Get-Process -Name "vcxsrv" -ErrorAction SilentlyContinue
if ($running) {
    Write-Host "VcXsrv already running (PID: $($running.Id -join ','))"
    exit 0
}

Write-Host "Starting VcXsrv :0 with -ac (allow Docker)..."
Start-Process -FilePath $VcXsrv -ArgumentList @(
    ":0",
    "-multiwindow",
    "-clipboard",
    "-wgl",
    "-ac"
)

Start-Sleep -Seconds 2
Write-Host "VcXsrv started. Check system tray for X icon."
