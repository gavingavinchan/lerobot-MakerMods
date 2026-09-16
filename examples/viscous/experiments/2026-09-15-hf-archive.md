# 2026-09-15 — prepared Hugging Face archive

**Prepared and verified locally; not uploaded yet.** The operator authorized public visibility.
No Hugging Face login was available during preparation. After terminal sign-in, the upload
helper defaults to the logged-in personal account and `viscous-pick-place-2026-09-15`.

Archive: `/home/gavin/Documents/robot-experiments/hf-viscous-pick-place-2026-09-15/`.

- 62 checksummed files, approximately 1.08 GB, plus `SHA256SUMS`.
- The first successful agent attempt through its saved result: 3,186 telemetry samples,
  8,010 wrist frames, 1,599 external frames, baseline/result, and final images.
- Original compressed video packets remuxed to finalized AVI files; packet hashes checked
  against the source recording. Timestamp-aligned H.264 review copies were decoded and
  verified to contain the same frame counts.
- Original demonstration, native LeRobot task crop, alignment metadata, source, calibration
  snapshots, and reviews. The archive root is not itself a native LeRobot dataset.
- The upload bundle freezes the first successful attempt. Later changed-scene attempts remain
  separate in `reference-pick-place/adaptive_run_01/` and are not included in this bundle.

The source controller was still recording when the successful interval was extracted; it
was not stopped for export. Its subsequent supported normal release is documented in the
[changed-scene report](2026-09-15-adaptive-scene.md).

Sign in locally (do not paste a token into chat):

```bash
uv run --no-sync hf auth login
```

Upload the reviewed public archive after sign-in:

```bash
uv run --no-sync python /home/gavin/Documents/robot-experiments/upload_viscous_archive.py
```

The helper refuses to overwrite an already-existing dataset. It accepts `--repo-id OWNER/NAME`
for an explicitly chosen destination. If an upload is interrupted after repository creation,
inspect that repository and resume its upload rather than creating another copy. Record the
actual Hub URL and revision here after a successful upload; do not infer publication from
local preparation alone.
