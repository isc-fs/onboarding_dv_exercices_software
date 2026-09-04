# Compose cannot attach `docker compose watch` to `up -d` from the compose file
# (watch is a host-side process; `-d` and `--watch` are mutually exclusive on the
# CLI). Use this instead of plain `docker compose up -d` when you want pipeline
# sources synced into the dv_pipeline_stack named volumes.
#
# Usage:
#   .\tools\compose-up-and-watch.ps1
#   .\tools\compose-up-and-watch.ps1 dv_pipeline_stack
#
# Detaches containers, then runs watch in the foreground. Ctrl+C stops watch only.

param(
  [Parameter(ValueFromRemainingArguments = $true)]
  [string[]]$ComposeArgs
)

$ErrorActionPreference = 'Stop'
$Root = Resolve-Path (Join-Path $PSScriptRoot '..')
Set-Location $Root

docker compose up -d @ComposeArgs
docker compose watch @ComposeArgs
