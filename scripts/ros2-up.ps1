# Build/start Dev Container compose stack (for manual shell / GUI test before CLion connects)
$ErrorActionPreference = "Stop"
Set-Location (Join-Path $PSScriptRoot "..")

$ComposeFile = ".devcontainer/docker-compose.yml"

Write-Host ">>> Checking Docker..."
docker info *> $null
if ($LASTEXITCODE -ne 0) {
    Write-Error "Docker not ready. Start Docker Desktop first."
    exit 1
}

if (-not (Test-Path ".\ws\src")) {
    New-Item -ItemType Directory -Path ".\ws\src" -Force | Out-Null
    Write-Host "Created ws\src"
}

Write-Host ">>> docker compose build ..."
docker compose -f $ComposeFile build

Write-Host ">>> docker compose up -d ..."
docker compose -f $ComposeFile up -d

Write-Host ""
Write-Host "Done. Dev Container service: alien-scanner-dev"
Write-Host "Manual shell:"
Write-Host "  docker compose -f $ComposeFile exec ros2 bash"
Write-Host "Colcon (inside container):"
Write-Host "  cd /workspaces/alien-scanner/ws && colcon build"
Write-Host ""
Write-Host "CLion: open project -> devcontainer.json gutter -> Create Dev Container and Mount Sources"
