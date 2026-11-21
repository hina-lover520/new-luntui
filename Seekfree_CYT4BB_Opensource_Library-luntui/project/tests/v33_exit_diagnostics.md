# v33 optional L exit diagnostics

Existing R/D/E/G/H/J/K/B/S/I field layouts and shared addresses remain unchanged. Pair and startup versions are 33.

L is optional evidence only. Its independent 32-byte mailbox is at `0x280011c0`, following J at `0x280011a0`; magic is `0x524c4721`. The existing shared SRAM reservation covers `0x28001000` through `0x28002fff`.

## Shared packet

| Word | Meaning |
|---:|---|
| 0 | Magic |
| 1 | Existing even seqlock sequence |
| 2 | Epoch |
| 3 | Current command sequence |
| 4 | Current vision frame sequence |
| 5 | Reject bits0..5; candidate bit16; latched bit17; confirmed bit18. All other bits zero. |
| 6 | Candidate frames low8; confirmation frames next8; signed lap in 0.1 degrees high16. Counts and lap saturate to representable range. |
| 7 | Signed forward progress in millirevolutions |

The receiver checks magic, seqlock, epoch, exact current command, reserved bits, and age <=300ms. L never owns control or refreshes command/watchdog age. A corrupt new packet cannot renew a preceding valid sample.

## UART

`L,seq,ms,epoch,cmd,frame,reject,candidate,latched,confirmed,candidate_frames,confirm_frames,lap10,forward_mrev,age,drop`

`reject` bit meanings: 1 lap not reached; 2 entry not confirmed; 4 exit geometry; 8 candidate continuity; 16 forward motion; 32 locked corridor lost. These are independent reasons and may combine.

L shares the existing one-record optional admission after R/K and E/J. Candidate/latch/confirmed changes are immediate on a new frame; scalar count/reject updates are limited to 200ms. Normal/COOLDOWN has no repeated L heartbeat; a terminal state transition can be emitted once. Vision publication only copies already computed scalars, adding no pixel scans or waits. Existing FIFO service stays at <=64 writes per call, twice per poll.

`#EXIT_WINDOW` reports evaluated integers: OBSERVE340, FIRST_MIN360, FIRST_MAX375, CONTINUE_YAW30, MS4000, CONFIRM3/90ms. The upper bound is evaluated from MIN+TOL without target floating-point printf.

## Verification

- UART protocol: PASS, 189201 packing and preserved-layout cases.
- FIFO suite: PASS for 16/64/128 depths, original R/D/E/G/H/J pressure, L15 bounds/cadence/new-frame behavior, drop0 at normal bandwidth.
- Production exit UART: 40 L records in 9.2s with R/K/J/E/H, 11424 bytes, drop0; NORMAL L silent.
- Production entry UART: 13301 bytes/9.2s, queue0, drop0; per-poll<=128 writes; constrained FIFO loss stays bounded/observable.
- Tangent transport: PASS23021; FIFO16 startup actually delivers the first complete R in146 polls/2352bytes. The fixture explicitly bounds startup to256poll/4096bytes and each poll<=16bytes, then retains its original50poll H event latency check. This distinguishes startup transfer from event starvation.
- New passive reader `audit_v33_topology_images.py`: L15 signed/bounds/column tests, matching J33 and wrong-pair rejection, inherited B/K/S/I/CRC/image reconstruction tests PASS. It never opens/writes a serial port.
- Same captured v32 input: 196650 bytes, 1845 diagnostic/image records; v32 and v33 decodes identical, errors0. Existing v32 reader file SHA256 remains `dffd57295c91c8cef66d2507a6f6a03a71e73ffed12038ff47e475a1cb78112d`.

L runtime reception/isolation tests are owned by the runtime change. Vision publication compiles against the agreed core exit fields. No device-side v33 capture or physical exit trial has been performed by this diagnostic work.
