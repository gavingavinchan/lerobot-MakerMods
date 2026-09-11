# Calibration backup

A copy of `~/.cache/huggingface/lerobot/calibration/` from the machine that commissioned the
Viscous Arm (unit 1) and the Maker arm, so the ids the teleop commands rely on survive a wiped
checkout or a wiped cache. The layout mirrors the cache exactly.

| File                                                  | Used by                                              |
| ----------------------------------------------------- | ---------------------------------------------------- |
| `robots/viscous_follower/viscous_01.json`             | `--robot.type=viscous_follower --robot.id=viscous_01` |
| `teleoperators/rebot_102_leader/star_viscous.json`    | `--teleop.type=rebot_102_leader_viscous --teleop.id=star_viscous` |
| `robots/maker_follower/maker_maker.json`              | `--robot.type=maker_follower --robot.id=maker_maker`  |
| `teleoperators/rebot_102_leader/maker_maker.json`     | `--teleop.type=rebot_102_leader_maker --teleop.id=maker_maker` |
| `*/so_*/video.json`                                   | SO-arm calibration, unrelated to the Viscous work     |

Restore (does not overwrite files that already exist):

```bash
cp -Rn examples/calibration/. ~/.cache/huggingface/lerobot/calibration/
```

**Never regenerate the follower file with `lerobot-calibrate`** -- on a commissioned arm that
re-zeros every motor at its current pose and destroys the calibration these files record. See
[Teleoperating the Viscous Arm](../../docs/source/viscous_teleop.mdx). If a file here and the
cache disagree, the cache is newer; copy it back here and commit it.
