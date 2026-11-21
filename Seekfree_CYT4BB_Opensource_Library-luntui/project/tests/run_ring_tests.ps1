param([string[]]$Only = @())
$ErrorActionPreference = 'Stop'
$ringProject = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$ringCompiler = (Get-Command gcc -ErrorAction Stop).Source
$ringTemporary = Join-Path ([IO.Path]::GetTempPath()) ('seekfree-ring-' + [Guid]::NewGuid().ToString('N'))
$ringExecutables = @()
$ringCode = Join-Path $ringProject 'project/code'
$ringIntegration = @('ring.c', 'ring_link.c', 'ring_vision.c', 'ring_runtime.c')
$ringJobs = @()
foreach ($ringRoute in @(0, -1, 1)) {
    $ringJobs += @{ Name="route-$ringRoute"; Test='test_ring.c'; Sources=@('ring.c'); Defines=@("-DRING_ROUTE=$ringRoute") }
}
$ringJobs += @{ Name='control'; Test='test_ring_control.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='run-buttons'; Test='test_run_buttons.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='standby-beep'; Test='test_ring_standby_beep.c'; Sources=@('ring_runtime.c', 'ring_link.c'); Defines=@() }
$ringJobs += @{ Name='preview'; Test='test_ring_preview.c'; Sources=$ringIntegration; Defines=@() }
$ringJobs += @{ Name='integration'; Test='test_ring_integration.c'; Sources=$ringIntegration; Defines=@() }
$ringJobs += @{ Name='geometry'; Test='test_ring_geometry.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='field-regressions'; Test='test_ring_field_regressions.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='entry-progress'; Test='test_ring_entry_progress.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='entry-assist'; Test='test_ring_entry_assist.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='entry-assist-safety'; Test='test_ring_entry_assist_safety.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='slow-progress'; Test='test_ring_slow_progress.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='seed-continuity'; Test='test_ring_seed_continuity.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='branch-follow'; Test='test_ring_branch_follow.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='yaw-convention'; Test='test_ring_yaw_convention.c'; Sources=@('ring.c', 'ring_runtime.c', 'ring_link.c'); Defines=@() }
$ringJobs += @{ Name='isolation'; Test='test_ring_isolation.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='disabled'; Test='test_ring_disabled.c'; Sources=$ringIntegration; Defines=@('-DRING_ENABLE=0') }
$ringJobs += @{ Name='uart-protocol'; Test='test_ring_uart_protocol.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='uart-fifo'; Test='test_ring_uart_fifo.c'; Sources=@('ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='diag-isolation'; Test='test_ring_diag_isolation.c'; Sources=@('ring.c', 'ring_link.c', 'ring_runtime.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
foreach ($ringJob in $ringJobs) { $ringJob.Defines += '-DRING_TRACK_MODE=0' }
$ringJobs += @{ Name='tangent-route'; Test='test_ring_tangent_route.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='tangent-topology'; Test='test_ring_tangent_topology.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='tangent-transport'; Test='test_ring_tangent_transport.c'; Sources=@('ring.c','ring_link.c','ring_runtime.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='observed-mouth'; Test='test_ring_observed_mouth.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='observed-geometry'; Test='test_ring_observed_geometry.c'; Sources=@('ring.c'); Defines=@() }
$ringJobs += @{ Name='near-pass'; Test='test_ring_near_pass.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='bottom-pass'; Test='test_ring_bottom_pass.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='far-handover'; Test='test_ring_far_handover.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='entry-follow'; Test='test_ring_entry_follow.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='clipped-portal'; Test='test_ring_clipped_portal.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='force-control'; Test='test_ring_force.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='ring-release'; Test='test_ring_release.c'; Sources=@('ring_vision.c','ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='normal-path'; Test='test_ring_normal_path.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='entry-budget'; Test='test_ring_entry75.c'; Sources=@(); Defines=@() }
foreach($ringJob in $ringJobs){$ringJob.Defines+='-DRING_TIMED_ENTRY=0'}
$ringJobs += @{ Name='timed-maneuver'; Test='test_ring_maneuver.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='timed-vision'; Test='test_ring_timed_vision.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='timed-integration'; Test='test_ring_timed_integration.c'; Sources=$ringIntegration; Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='entry-speed'; Test='test_ring_entry_speed.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='entry-uart-budget'; Test='test_ring_entry_uart_budget.c'; Sources=@('ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='reentry15s'; Test='test_ring_reentry.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='lap360-control-guard'; Test='test_ring_lap_guard.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='visual-exit-recovery'; Test='test_ring_visual_exit.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='forward-profile'; Test='test_ring_forward_profile.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v29-real-snapshots'; Test='test_ring_v29_regression.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v30-entry-handover'; Test='test_ring_v30_regression.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='yaw-bias-stability'; Test='test_ring_yaw_bias.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v31-forward-entry'; Test='test_ring_v31_motion.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v31-exit-direction'; Test='test_ring_v31_exit.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v32-real-paths'; Test='test_ring_v32_paths.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v32-visual-handover'; Test='test_ring_v32_handover.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v33-selected-exit'; Test='test_ring_v33_exit.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v33-control-exit'; Test='test_ring_v33_runtime.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v33-exit-uart'; Test='test_ring_v33_exit_uart.c'; Sources=@('ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='v33-exit-stale'; Test='test_ring_v33_exit_stale_audit.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v33-exit-recover'; Test='test_ring_v33_recover_audit.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v34-exit-beep'; Test='test_ring_v34_beep.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v34-exit-uart'; Test='test_ring_v34_exit_uart.c'; Sources=@('ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
$ringJobs += @{ Name='v34-current-outer'; Test='test_ring_v34_outer.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v34-control-exit'; Test='test_ring_v34_exit_control.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v34-vision-exit'; Test='test_ring_v34_exit_vision.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v34-publish-control'; Test='test_ring_v34_publish_integration.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v35-exit-reacquire'; Test='test_ring_v35_exit_reacquire.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v35-episode-control'; Test='test_ring_v35_episode_integration.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v35-opening'; Test='test_ring_v35_opening.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v35-uart-fair'; Test='test_ring_v35_uart_fair.c'; Sources=@('ring_link.c'); Defines=@('-I', (Join-Path $PSScriptRoot 'uart_stub')) }
# Preserve historical exit-policy regressions explicitly. New v36 cases below
# exercise the actual default400 gate; reference tests still cover old geometry.
foreach($ringJob in $ringJobs){
    if(-not ($ringJob.Defines -contains '-DRING_TRACK_MODE=0') -and
       -not ($ringJob.Defines -contains '-DRING_TIMED_ENTRY=0')){
        $ringJob.Defines+='-DRING_EXIT_NET_MIN_DEG=0'
    }
}
$ringJobs += @{ Name='v36-cropped-exit'; Test='test_ring_v36_corridor.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v36-runtime-observe-exit'; Test='test_ring_v36_runtime.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v36-corridor-negatives'; Test='test_ring_v36_corridor_negatives.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v36-net400-runtime'; Test='test_ring_v36_gate400_runtime.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v36-beep-patterns'; Test='test_ring_v36_beep.c'; Sources=@(); Defines=@() }
$ringJobs += @{ Name='v36-beep-runtime'; Test='test_ring_v36_beep_runtime.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
$ringJobs += @{ Name='v36-episode-integration'; Test='test_ring_v36_episode.c'; Sources=@('ring_runtime.c','ring_link.c'); Defines=@() }
if ($Only.Count -gt 0) { $ringJobs = @($ringJobs | Where-Object { $Only -contains $_.Name }) }
New-Item -ItemType Directory -Path $ringTemporary | Out-Null
try {
    foreach ($ringJob in $ringJobs) {
        Write-Output ("RUN: " + $ringJob.Name + "; model=" + $(if ($ringJob.Defines -contains '-DRING_TRACK_MODE=0') { 'legacy-frontal' } elseif($ringJob.Defines -contains '-DRING_TIMED_ENTRY=0') { 'tangent-reference' } elseif($ringJob.Defines -contains '-DRING_EXIT_NET_MIN_DEG=0') { 'timed-entry-historical-exit-reference' } else { 'timed-entry-default-net400' }))
        $ringExecutable = Join-Path $ringTemporary ($ringJob.Name + '.exe')
        $ringExecutables += $ringExecutable
        $ringArguments = @('-std=c99', '-Wall', '-Wextra', '-Werror', '-pedantic', '-DRING_HOST_TEST=1')
        $ringArguments += $ringJob.Defines
        $ringArguments += @('-I', $ringCode, (Join-Path $PSScriptRoot $ringJob.Test))
        foreach ($ringSource in $ringJob.Sources) { $ringArguments += Join-Path $ringCode $ringSource }
        $ringArguments += @('-lm', '-o', $ringExecutable)
        & $ringCompiler @ringArguments
        if ($LASTEXITCODE -ne 0) { throw "Ring compilation failed: $($ringJob.Name)" }
        & $ringExecutable
        if ($LASTEXITCODE -ne 0) { throw "Ring regression failed: $($ringJob.Name)" }
    }
    & node (Join-Path $PSScriptRoot 'verify_v29_isolation.cjs') $ringProject (Join-Path $PSScriptRoot 'v36_preserved_inputs.json')
    if ($LASTEXITCODE -ne 0) { throw 'Working original controller isolation check failed' }
} finally {
    # Only remove exact executables created inside this unique directory.
    foreach ($ringExecutable in $ringExecutables) {
        if (Test-Path -LiteralPath $ringExecutable) { Remove-Item -LiteralPath $ringExecutable }
    }
    Remove-Item -LiteralPath $ringTemporary
}
