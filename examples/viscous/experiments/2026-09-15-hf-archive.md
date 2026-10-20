# 2026-09-15 — prepared Hugging Face archive

**Prepared and verified locally; not uploaded yet.** The operator authorized public visibility.
No Hugging Face login was available during preparation. After terminal sign-in, the upload
helper defaults to the logged-in personal account and `viscous-pick-place-2026-09-15`.

Archive: `/home/gavin/Documents/robot-experiments/hf-viscous-pick-place-2026-09-15/`.

## Expanded scope: full recordings, successes, and failures

The operator subsequently requested **everything, including the full continuous session
recordings**. The expanded bundle is approximately 25.6 GB and includes 40 AVI/MP4 files.
`full_sessions/` preserves all files from `2026-09-15-orange-ball/`, `reference-pick-place/`,
`teleop-demo-20260915-182851/`, and `social/`, excluding only Python bytecode caches.
This includes the latest failed attempts, complete videos, telemetry, timestamps, commands,
checkpoints, logs, helper scripts, review assets, and historical runtime markers. Copied
full-session files were checked byte for byte against the originals. Curated success exports
remain alongside the complete sources, so some evidence is intentionally duplicated.

The original `adaptive_run_01` AVIs are preserved as captured, without normal finalization
after the operator-requested controller termination. They have not been represented as
repaired or fully decoded. The curated success review videos remain finalized. The root
README distinguishes successes, failures, historical powered holds, and subsequent shutdown.
The archive remains an experiment archive, not a unified native LeRobot training dataset.

`expand_full_archive.py` records the expansion procedure. The updated upload helper checks
the complete checksum manifest before upload, refuses unrelated existing repositories, and
supports `--resume` for the same account, repository, and payload recorded in the local
`/home/gavin/Documents/robot-experiments/viscous_upload_state.json`. After upload it verifies
remote file names and sizes at a recorded Hub revision. That state file is outside the payload.
No repository has been created and no upload is claimed until authentication and remote
verification succeed.

## Earlier curated exports

- The original successful-attempt bundle was 62 checksummed files, approximately 1.08 GB.
  The bundle now also includes the changed-scene success and earlier retry checkpoints;
  the regenerated `SHA256SUMS` covers the expanded contents.
- The first successful agent attempt through its saved result: 3,186 telemetry samples,
  8,010 wrist frames, 1,599 external frames, baseline/result, and final images.
- Original compressed video packets remuxed to finalized AVI files; packet hashes checked
  against the source recording. Timestamp-aligned H.264 review copies were decoded and
  verified to contain the same frame counts.
- Original demonstration, native LeRobot task crop, alignment metadata, source, calibration
  snapshots, and reviews. The archive root is not itself a native LeRobot dataset.
- `adaptive_success/` freezes the operator-confirmed changed-scene pickup/deposit: about
  98 s, 977 telemetry rows, 2,451 wrist frames and 490 external frames, original compressed
  packets remuxed and hash-verified, and timestamp-aligned MP4 copies with verified frame counts.
  It includes commands, paired checkpoints, result and source provenance. This is the second
  overall success, after multiple misses, not a benchmark success rate.
- `adaptive_retry_checkpoints/` preserves local probe and failed-retry images/status/commands.
  Their full continuous videos remain in `reference-pick-place/adaptive_run_01/`; the bundle
  does not claim to contain every frame from all retries.

The original successful source controller was still recording when its interval was extracted;
its subsequent supported normal release is documented in the [changed-scene report](2026-09-15-adaptive-scene.md).
The replacement controller later ended without normal finalization; see the
[third-attempt shutdown addendum](2026-09-15-third-attempt.md).

## Upload commands

Sign in locally (do not paste a token into chat):

```bash
uv run --no-sync hf auth login
```

Upload the reviewed public archive after sign-in:

```bash
uv run --no-sync python /home/gavin/Documents/robot-experiments/upload_viscous_archive.py
```

Use `--dry-run` to verify and inspect the payload without creating or uploading anything.
If interrupted after this helper creates the repository, rerun with `--resume` (and the same
`--repo-id` if one was specified). The upload state validates the intended destination.

The helper refuses to overwrite an already-existing dataset. It accepts `--repo-id OWNER/NAME`
for an explicitly chosen destination. If an upload is interrupted after repository creation,
resume its upload rather than creating another copy. Record the
actual Hub URL and revision here after a successful upload; do not infer publication from
local preparation alone.
