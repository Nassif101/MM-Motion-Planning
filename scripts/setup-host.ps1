[CmdletBinding()]
param(
    [switch]$InstallEditor,
    [switch]$BuildContainer
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
$unityProject = Join-Path $repoRoot "motion-planning-sim"
$composeFile = Join-Path $repoRoot ".devcontainer/docker-compose.yml"

foreach ($tool in @("git", "docker", "unity")) {
    if (-not (Get-Command $tool -ErrorAction SilentlyContinue)) {
        throw "$tool is required and was not found on PATH"
    }
}

$unityCommands = @(Get-Command unity -All -ErrorAction SilentlyContinue | Select-Object -ExpandProperty Source -Unique)
if ($unityCommands.Count -gt 1) {
    Write-Warning "Multiple Unity CLI binaries are on PATH; using $($unityCommands[0]). Found: $($unityCommands -join ', ')"
}
Write-Host "Unity CLI: $($unityCommands[0]) ($(unity --version))"

git -C $repoRoot submodule update --init --recursive
if ($LASTEXITCODE -ne 0) { throw "Submodule initialization failed" }

docker compose -f $composeFile config --quiet
if ($LASTEXITCODE -ne 0) { throw "Docker Compose validation failed" }

if ($InstallEditor) {
    unity install 6000.5.2f1 --architecture x86_64 --yes --accept-eula
    if ($LASTEXITCODE -ne 0) { throw "Unity Editor installation failed" }
    unity pipeline install --project-path $unityProject
    if ($LASTEXITCODE -ne 0) { throw "Unity Pipeline installation failed" }
}

if ($BuildContainer) {
    docker compose -f $composeFile build
    if ($LASTEXITCODE -ne 0) { throw "Container build failed" }
}

unity projects info $unityProject --format json
if ($LASTEXITCODE -ne 0) { throw "Unity project inspection failed" }
Write-Host "Windows x86-64 host setup checks passed."
