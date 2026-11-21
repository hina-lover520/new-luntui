# Real binary image regression inputs

These eight files are exact 2,820-byte camera snapshots reconstructed from
`capture_20261112_200338_598_781e48_boot_001` and later 20:56/20:57 S/I records. The host receiver
verified the entire payload CRC32 before writing each source file. The
manifest records the immutable source name, frame, flags, CRC32 and SHA256.

The image is 188 by 120, row-major, one white-road bit per pixel, most
significant bit first. There is no production thresholding, cleanup,
rotation or hand editing in these fixtures. Tests check byte count and
CRC32 before unpacking.

Images 7 and 10 contain a visible local neck/side opening that the v12
near-edge global line failed to observe. These observations are deliberately
**not labelled as a true far mouth**. Image 10's neck is short and its upper
connection is interrupted. Images 6 and 9 include clipped outer boundaries.
All fixed images must fail to manufacture a new mouth sequence when
only yaw and wheel readings change.

File IDs 105, 111 and 112 preserve 20:56 capture image 1, and 20:57 capture
images 1 and 2 respectively. The manifest retains their complete original
boot names, so the reused on-car image IDs cannot alias across sessions.

The fixed-world synthetic maps in `test_ring_observed_geometry.c` supply
independent continuous motion/visibility cases; the actual snapshots cannot
serve as a continuous replay because intermediate frames were not captured.
