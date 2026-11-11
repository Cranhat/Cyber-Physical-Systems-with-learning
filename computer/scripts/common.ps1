$RepoDir = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path

# Server address the firmware connects to (robot/main/main.cpp: DataExchanger("ip", port, ...))
function Get-RobotTarget {
    $main = Join-Path $RepoDir "robot\main\main.cpp"
    $m = Select-String -Path $main -Pattern 'DataExchanger\s+\w+\(\s*"([\d.]+)"\s*,\s*(\d+)' -ErrorAction SilentlyContinue |
        Select-Object -First 1
    if (-not $m) { return $null }
    $wifi = [bool](Select-String -Path $main -Pattern 'wifiEnabled\s*=\s*true' -Quiet)
    return [pscustomobject]@{ Ip = $m.Matches[0].Groups[1].Value; Port = [int]$m.Matches[0].Groups[2].Value; Wifi = $wifi }
}

function Test-RobotNetwork($port) {
    $t = Get-RobotTarget
    if (-not $t) { Write-Host "WARN  robot address not found in robot/main/main.cpp" -ForegroundColor Yellow; return }
    if (-not $t.Wifi) { Write-Host "WARN  firmware has wifiEnabled = false: the robot will not connect" -ForegroundColor Yellow }
    if ($t.Port -ne $port) { Write-Host "WARN  firmware connects to port $($t.Port), server listens on $port" -ForegroundColor Yellow }
    $ips = @(Get-NetIPAddress -AddressFamily IPv4 -ErrorAction SilentlyContinue | ForEach-Object { $_.IPAddress })
    if ($ips -contains $t.Ip) {
        Write-Host "PASS  this PC has the robot's server address $($t.Ip)"
    } else {
        Write-Host "WARN  robot connects to $($t.Ip), this PC has: $(($ips | Where-Object { $_ -ne '127.0.0.1' }) -join ', ')" -ForegroundColor Yellow
        Write-Host "      join the robot's Wi-Fi / set this IP, or change it in robot/main/main.cpp" -ForegroundColor Yellow
    }
}

function Start-DockerIfNeeded {
    function Test-Docker { cmd /c "docker info >nul 2>&1"; return $LASTEXITCODE -eq 0 }
    if (Test-Docker) { return $true }
    $desktop = "C:\Program Files\Docker\Docker\Docker Desktop.exe"
    if (-not (Test-Path $desktop)) { Write-Host "FAIL  Docker Desktop not found" -ForegroundColor Red; return $false }
    Write-Host "== starting Docker Desktop"
    Start-Process $desktop
    $deadline = (Get-Date).AddMinutes(3)
    while (-not (Test-Docker)) {
        if ((Get-Date) -gt $deadline) { Write-Host "FAIL  Docker did not start in 3 min" -ForegroundColor Red; return $false }
        Start-Sleep -Seconds 3
    }
    return $true
}
