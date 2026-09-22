param(
	[string]$JavaHome
)

$ErrorActionPreference = 'Stop'
$repoRoot = (Resolve-Path (Join-Path $PSScriptRoot '..')).Path
$packageDir = Join-Path $repoRoot 'gui\dist\artifacts\win\win-unpacked'
$appExe = Join-Path $packageDir 'SlimeVR.exe'
$requiredFiles = @(
	$appExe,
	(Join-Path $packageDir 'slimevr.jar'),
	(Join-Path $packageDir 'SlimeVR-Bindings-Provider.exe'),
	(Join-Path $packageDir 'openvr_api.dll')
)

foreach ($required in $requiredFiles) {
	if (-not (Test-Path -LiteralPath $required -PathType Leaf)) {
		throw "Portable Windows app file is missing: $required"
	}
}

function Test-LocalPort([int]$Port) {
	$client = [System.Net.Sockets.TcpClient]::new()
	try {
		$task = $client.ConnectAsync('127.0.0.1', $Port)
		return $task.Wait(250) -and $client.Connected
	} catch {
		return $false
	} finally {
		$client.Dispose()
	}
}

if (Test-LocalPort 21110) {
	throw 'Port 21110 is already in use. Stop its current process before starting the packaged server.'
}

# SlimeVR uses this directory as a portable-mode marker and keeps vrconfig.yml beside the app.
New-Item -ItemType Directory -Force -Path (Join-Path $packageDir 'config') | Out-Null

$originalJavaHome = $env:JAVA_HOME
try {
	if ($JavaHome) {
		$resolvedJavaHome = (Resolve-Path -LiteralPath $JavaHome).Path
		$javaExe = Join-Path $resolvedJavaHome 'bin\java.exe'
		if (-not (Test-Path -LiteralPath $javaExe -PathType Leaf)) {
			throw "JAVA_HOME does not contain bin\java.exe: $resolvedJavaHome"
		}
		$env:JAVA_HOME = $resolvedJavaHome
	}

	$process = Start-Process -FilePath $appExe -WorkingDirectory $packageDir -PassThru
	Write-Host "Started portable SlimeVR from $packageDir (PID $($process.Id))."
	Write-Host 'The package keeps its server settings in vrconfig.yml beside the app.'
} finally {
	if ($JavaHome) { $env:JAVA_HOME = $originalJavaHome }
}
