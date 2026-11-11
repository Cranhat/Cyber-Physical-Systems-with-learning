# ROS 2 bridge in Docker against the real robot. Build output is cached in a Docker volume (Ninja),
# so after the first run only changed files are recompiled.
#
#   computer\scripts\ros.cmd              build + start the node, waits for the robot
#
# From a second terminal, while the node runs:
#   computer\scripts\ros.cmd move 0.2     move 0.2 m
#   computer\scripts\ros.cmd rotate 90    rotate o 90°
#   computer\scripts\ros.cmd status
#   computer\scripts\ros.cmd result
#   computer\scripts\ros.cmd odom
#   computer\scripts\ros.cmd scan
#   computer\scripts\ros.cmd clear
#   computer\scripts\ros.cmd shell
#   computer\scripts\ros.cmd stop
#   computer\scripts\ros.cmd rebuild

param(
    [Parameter(Position = 0)][string]$Action = "start",
    [Parameter(Position = 1)][string]$Value = "",
    [switch]$Fake
)

. "$PSScriptRoot\common.ps1"

$computer = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$container = "robot_bridge"
$image = "robot_bridge:jazzy"
$volume = "robot_bridge_ws"
$ros = "source /opt/ros/jazzy/setup.bash && source /ws/install/setup.bash"
# -it only when typed in a terminal (Ctrl+C, colors); plain when the output is redirected
# (assigned in two steps: `$x = if ... { @("-it") }` unwraps the array into a string, which @x splats char by char)
$tty = @()
if (-not ([Console]::IsInputRedirected -or [Console]::IsOutputRedirected)) { $tty = @("-it") }

function Test-Running {
    $state = cmd /c "docker inspect -f {{.State.Running}} $container 2>nul"
    return $state -eq "true"
}

function Invoke-Ros([string]$command) {
    if (-not (Test-Running)) {
        Write-Host "FAIL  the node is not running, start it first: computer\scripts\ros.cmd" -ForegroundColor Red
        exit 1
    }
    docker exec @tty $container bash -c "$ros && $command"
    exit $LASTEXITCODE
}

function Get-Number {
    $n = 0.0
    $ok = [double]::TryParse(($Value -replace ',', '.'), [Globalization.NumberStyles]::Float,
        [Globalization.CultureInfo]::InvariantCulture, [ref]$n)
    if (-not $ok) { Write-Host "FAIL  '$Action' needs a number, e.g. $Action 0.5" -ForegroundColor Red; exit 1 }
    return $n.ToString([Globalization.CultureInfo]::InvariantCulture)
}

switch ($Action) {
    "move"    { Invoke-Ros "ros2 topic pub --once /robot_bridge/move std_msgs/msg/Float32 '{data: $(Get-Number)}' > /dev/null && echo 'sent move $(Get-Number) m'" }
    "rotate"  { Invoke-Ros "ros2 topic pub --once /robot_bridge/rotate std_msgs/msg/Float32 '{data: $(Get-Number)}' > /dev/null && echo 'sent rotate $(Get-Number) deg'" }
    "status"  { Invoke-Ros "ros2 topic echo --once /robot_bridge/status" }
    "result"  { Invoke-Ros "ros2 topic echo /robot_bridge/result" }
    "odom"    { Invoke-Ros "ros2 topic echo --once /odom" }
    "scan"    { Invoke-Ros "ros2 topic echo --once /scan" }
    "clear"   { Invoke-Ros "ros2 service call /robot_bridge/clear_queue std_srvs/srv/Trigger" }
    "shell"   { Invoke-Ros "exec bash" }
    "stop"    { cmd /c "docker rm -f $container >nul 2>&1"; Write-Host "container removed"; exit 0 }
    "rebuild" {
        cmd /c "docker rm -f $container >nul 2>&1"
        cmd /c "docker volume rm $volume >nul 2>&1"
        Write-Host "build cache removed"; exit 0
    }
    "start"   { }
    default   { Write-Host "FAIL  unknown action '$Action' (see the top of ros.ps1)" -ForegroundColor Red; exit 1 }
}

# ---- start ----------------------------------------------------------------------------------------
if (-not (Start-DockerIfNeeded)) { exit 1 }

$t = Get-RobotTarget
$port = if ($t) { $t.Port } else { 3000 }

cmd /c "docker image inspect $image >nul 2>&1"
if ($LASTEXITCODE -ne 0) {
    Write-Host "== building image $image (first run only, downloads ros:jazzy ~1 GB)"
    docker build -t $image -f (Join-Path $PSScriptRoot "Dockerfile") $PSScriptRoot
    if ($LASTEXITCODE -ne 0) { Write-Host "FAIL  docker build" -ForegroundColor Red; exit 1 }
}

cmd /c "docker rm -f $container >nul 2>&1"
# sources read-only, /ws (build, install, log) in a volume; the port is published so the robot can connect
docker run -d --name $container -p "${port}:3000" -v "${volume}:/ws" -v "${computer}:/ws/src/robot_bridge:ro" `
    $image sleep infinity | Out-Null
if ($LASTEXITCODE -ne 0) { Write-Host "FAIL  docker run (is port $port already used?)" -ForegroundColor Red; exit 1 }

try {
    Write-Host "== colcon build (Ninja)"
    docker exec $container bash -c "source /opt/ros/jazzy/setup.bash && cd /ws && colcon build --cmake-args -G Ninja > /tmp/build.log 2>&1 || { tail -40 /ws/log/latest_build/robot_bridge/stdout_stderr.log; exit 1; }"
    if ($LASTEXITCODE -ne 0) { Write-Host "FAIL  colcon build" -ForegroundColor Red; exit 1 }
    Write-Host "PASS  build"

    if ($Fake -and -not (Test-Path (Join-Path $computer "testing\fake_robot.py"))) {
        Write-Host "FAIL  -Fake needs computer/testing/fake_robot.py (local only, not in the repo)" -ForegroundColor Red; exit 1
    }
    if ($Fake) {
        docker exec -d $container bash -c "sleep 2 && python3 -u /ws/src/robot_bridge/testing/fake_robot.py --port 3000 > /tmp/fake.log 2>&1"
        Write-Host "      fake robot starts in 2 s (log in the container: /tmp/fake.log)"
    } else {
        Test-RobotNetwork $port
    }

    Write-Host "== node on port $port, waiting for the robot. Ctrl+C stops it."
    Write-Host "   commands from a 2nd terminal: computer\scripts\ros.cmd move 0.2 | rotate 90 | status | result"
    docker exec @tty $container bash -c "$ros && ros2 launch robot_bridge robot_bridge.launch.xml"
} finally {
    cmd /c "docker rm -f $container >nul 2>&1"
}
