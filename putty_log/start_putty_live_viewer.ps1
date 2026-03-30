param(
    [string]$Port = "COM5",
    [int]$Baud = 115200,
    [string]$ViewerHost = "127.0.0.1",
    [int]$ViewerPort = 8765,
    [switch]$NoPutty,
    [switch]$NoBrowser
)

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$BridgeScript = Join-Path $ScriptDir "putty_live_bridge.py"
$LogFile = Join-Path $ScriptDir "putty.log"
$ViewerUrl = "http://{0}:{1}/Doc/putty/index.html?autoconnect=bridge&path=putty_log/putty.log" -f $ViewerHost, $ViewerPort
$BridgeStdout = Join-Path $ScriptDir "putty_live_bridge.stdout.log"
$BridgeStderr = Join-Path $ScriptDir "putty_live_bridge.stderr.log"

$PythonCmd = (Get-Command python -ErrorAction SilentlyContinue)
if (-not $PythonCmd) {
    Write-Host "python not found. Install Python 3 first." -ForegroundColor Red
    exit 1
}

if (-not (Test-Path $BridgeScript)) {
    Write-Host "Bridge script not found: $BridgeScript" -ForegroundColor Red
    exit 1
}

$PuttyCandidates = @(
    "C:\Program Files\PuTTY\putty.exe",
    "C:\Program Files (x86)\PuTTY\putty.exe"
)
$PuttyPath = $PuttyCandidates | Where-Object { Test-Path $_ } | Select-Object -First 1

if (-not $NoPutty -and -not $PuttyPath) {
    Write-Host "PuTTY not found. Install PuTTY first or rerun with -NoPutty." -ForegroundColor Red
    exit 1
}

$BridgeAlreadyListening = $null
try {
    $BridgeAlreadyListening = Get-NetTCPConnection -LocalPort $ViewerPort -State Listen -ErrorAction Stop | Select-Object -First 1
} catch {
    $BridgeAlreadyListening = $null
}

if (-not $BridgeAlreadyListening) {
    Write-Host "Starting Python bridge on http://$ViewerHost`:$ViewerPort ..."
    Start-Process -FilePath $PythonCmd.Source `
        -ArgumentList @(
            $BridgeScript,
            "--host", $ViewerHost,
            "--port", "$ViewerPort",
            "--root", $ProjectRoot,
            "--log", "putty_log/putty.log"
        ) `
        -WorkingDirectory $ProjectRoot `
        -WindowStyle Hidden `
        -RedirectStandardOutput $BridgeStdout `
        -RedirectStandardError $BridgeStderr | Out-Null
    Start-Sleep -Milliseconds 900
} else {
    Write-Host "Python bridge is already listening on port $ViewerPort."
}

if (-not $NoBrowser) {
    Write-Host "Opening viewer: $ViewerUrl"
    Start-Process $ViewerUrl | Out-Null
}

if (-not $NoPutty) {
    Write-Host "Starting PuTTY logging..."
    Write-Host "Port   : $Port"
    Write-Host "Baud   : $Baud"
    Write-Host "Log    : $LogFile"
    Start-Process -FilePath $PuttyPath `
        -ArgumentList "-serial $Port -sercfg $Baud,8,n,1,N -sessionlog `"$LogFile`" -logoverwrite" | Out-Null
} else {
    Write-Host "PuTTY launch skipped (-NoPutty)."
}

Write-Host ""
Write-Host "Viewer bridge stdout: $BridgeStdout"
Write-Host "Viewer bridge stderr: $BridgeStderr"
Write-Host "If your STM32 port is not COM5, rerun for example:"
Write-Host "  powershell -ExecutionPolicy Bypass -File .\putty_log\start_putty_live_viewer.ps1 -Port COM6"
