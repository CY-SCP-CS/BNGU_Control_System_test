param([string]$Compiler = 'C:\Program Files (x86)\Dev-Cpp\MinGW64\bin\gcc.exe')
$ErrorActionPreference = 'Stop'
$repo = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '..\..'))
New-Item -ItemType Directory -Force -Path (Join-Path $PSScriptRoot 'build') | Out-Null
$includeDirs = @('tests/chassis/stubs', 'Self_core/App/sentry',
    'Self_core/App/sentry/chassis', 'Self_core/App/common/communication',
    'Self_core/Lib/Lib_Typedef', 'Self_core/Lib/Lib_math',
    'Self_core/Lib/Lib_filter', 'Self_core/Lib/Lib_pid',
    'Self_core/Drv/Drv_motor', 'Self_core/Drv/Drv_power_measure')
$ccArgs = @('-std=c11', '-Wall', '-Werror')
foreach ($dir in $includeDirs) { $ccArgs += '-I' + $dir }
$sources = @('tests/chassis/test_chassis.c', 'Self_core/Lib/Lib_math/lib_math.c',
    'Self_core/Lib/Lib_filter/lib_filter.c', 'Self_core/Lib/Lib_pid/lib_pid.c',
    'Self_core/Drv/Drv_motor/drv_motor.c', 'Self_core/Drv/Drv_power_measure/drv_power_measure.c')
$previousPath = $env:PATH
$env:PATH = (Split-Path -Parent $Compiler) + ';' + $env:PATH
Push-Location $repo
try {
    foreach ($enabled in @(1, 0)) {
        $exe = 'tests/chassis/build/ramp-' + $enabled + '.exe'
        & $Compiler @ccArgs "-DSENTRY_CHASSIS_RAMP_ENABLE=$enabled" @sources '-lm' '-o' $exe
        if ($LASTEXITCODE -ne 0) { throw 'Host compile failed.' }
        & $exe
        if ($LASTEXITCODE -ne 0) { throw 'Regression failed.' }
    }
} finally {
    Pop-Location
    $env:PATH = $previousPath
}
