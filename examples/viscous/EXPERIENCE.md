# Viscous Arm — accumulated operating experience

Read this before planning autonomous motion. Update it after each reviewed experiment so
future sessions can use previous evidence. This is an explicit record of experience; saving
an episode alone does not establish an improved controller. Current evidence concerns unit
`viscous_01`, its existing calibration, the dry tabletop, and the recorded camera setup.

## Evidence so far

| Experiment | Outcome | What it establishes |
| --- | --- | --- |
| [2026-09-15 follower-only trial](experiments/2026-09-15-orange-ball.md) | No grasp | Gripper direction, small visual corrections, and heating during slow motion/long holds |
| [2026-09-15 reference-guided trial](experiments/2026-09-15-reference-pick-place.md) | One supervised pickup and deposit | A demonstrated joint trajectory plus local visual corrections worked in this scene |
| [2026-09-15 changed-scene trial](experiments/2026-09-15-adaptive-scene.md) | Initial retries failed; subsequent pickup and deposit succeeded, confirmed by operator | Wrist ball size exposed insufficient approach distance; coordinated forward correction and closure before lift worked in this changed scene |
| [2026-09-15 third requested attempt](experiments/2026-09-15-third-attempt.md) | Two empty closures including the continuation; no pickup or deposit; operator requested shutdown | Wrist-image matching alone did not establish capture; checkpoint-only thermal monitoring was inadequate |

Two successful attempts across the original and changed scenes are not a measured success rate. No verified URDF, Cartesian reach
model, or generally reliable autonomous policy was established by these trials.

## Lessons to apply

1. **Start from demonstrated motion when the scene matches.** The successful trial used the
   operator's approach, grasp, transfer, and retreat as a joint-space reference. The earlier
   small-jog trial never established a complete grasp. This supports using the demonstration
   as a starting point; it does not prove the approaches have different reliability across
   repeated trials. Compare current object positions, camera framing, calibration, and actual
   arm pose before reusing a reference.
   When the operator asks for adaptation, preserve the changed scene as the test. Do not
   substitute resetting objects to the demonstration. A controller offset cap is a software
   bound, not proof that the arm cannot reach the new object position.
2. **Use wrist images for alignment and external images for task outcome.** At raw reference
   time 11.6 s, total offsets of +1.5° pan / +1° shoulder lift worsened alignment; total offsets
   of −1.5° pan / +2.5° shoulder lift improved it and preceded success. These are offsets from
   that reference, not absolute poses or a universal correction. Two joints changed together:
   their separate effects and any degrees-to-distance relationship remain unmeasured. Image
   centering alone does not establish depth, clearance, or a secure grasp.
   **Compare apparent ball size as well as position.** The operator's wrist-camera size cue
   resolved the changed-scene miss: failed retries showed a ball about 120 px wide versus
   153 px immediately before the demonstrated grasp (640×480 images). Coordinated negative
   shoulder-lift / positive wrist-flex corrections increased apparent width to about 145 px;
   closure then secured it. Do not keep raising the elbow merely to lower the ball in the
   image: earlier isolated raises reduced its apparent size. These are local observations
   with the same ball/camera, not calibrated distance or a universal 153 px threshold.
   In the third requested attempt, the ball was clipped at the bottom of the wrist image
   (170x89 px) and remained behind the fingers. Do not equate clipped width with the
   full-ball reference or close without resolving that geometry in the external view.
   That closure was empty; local corrections did not resolve it.
   After an operator reset, a full 145x141 px ball at (295, 352) also preceded an empty
   closure with wrist-yaw offset -9 degrees. Image size/position matching is therefore
   insufficient by itself across changed gripper orientations. The operator stopped
   further attempts; see the third-attempt report for the hold and subsequent shutdown.
   Successful offsets at reference 12.5 s were pan +16.5°, shoulder −10°, elbow +6°, wrist
   flex +10°; see the report for the approach and distinct deposit corrections. Do not reuse
   the pickup offsets blindly for placement into a moved foam opening.
3. **More-negative gripper angles open the fingers on this calibrated unit.** The earlier
   opening-and-return check tracked within 1°. Confirm actual object capture visually;
   commanding closure does not prove capture. Do not transfer absolute gripper angles to
   another calibration or after a coupling shift.
   In the changed-scene trial, the operator observed fingers starting below the ball and
   rising before securing it. Elbow adjustments happened with the gripper open; closure
   followed but captured nothing. Once positioned around the ball, hold height and close,
   check visual retention and measured gripper response, then lift. At the completed grasp
   sample, successful closure showed −79.77°/2.42 Nm against a −72.84° target; failed closure
   showed −72.85°/0.04 Nm. This contrast is supporting evidence, not a calibrated detector.
   The successful changed-scene closure measured −80.37°/1.69 Nm against −75.49° commanded.
   A +3° shoulder correction then lifted with the ball retained; the subsequent transfer
   preserved its wrist-image position while the external view confirmed movement to the foam.
   **After a miss, reopen and correct locally.** The operator explicitly rejects returning to
   the beginning after each failed closure. Unload when needed for temperature, not merely
   because one grasp failed. Successful `scale_05` made its corrections in one approach.
4. **Minimize loaded waiting, not just movement speed.** In the successful run, about 101 s
   passed at the pregrasp reference phase, with only about 2.4 s spent applying corrections.
   The elbow later peaked at 80 °C. Plan the sequence and visual checkpoints before extending
   the arm; carry out lengthy analysis while it is in a verified resting state. Existing heat
   warnings require return to rest. Continuing this run past 60 °C and 75 °C is a behavior to
   correct, even though placement succeeded. See the [thermal guidance](../../docs/source/viscous_teleop.mdx#thermal-duty).
   The later changed-scene success still peaked at 68 °C across a roughly 98 s recorded
   interval. Pickup success did not resolve loaded waiting or thermal recovery. Prepare
   deposit alignment and release decisions before lifting to avoid another long loaded pause.
   The third requested attempt peaked at 68 C despite a cooling interval. Later checkpoint
   temperatures rose from 51 C to 67 C. Temperature checks between conversational tool
   calls did not enforce timely unloading; an active monitor and validated return path
   remain necessary before repeating this procedure.
5. **Judge progress from measured state and images.** Peak sampled errors against sent
   commands were about 9.2° at the shoulder lift and 10.0° at the elbow. These differences do
   not by themselves indicate poor control: table contact can constrain motion, and the
   operator explicitly accepts tabletop contact as potentially useful for gripper alignment
   (2026-09-15). Do not classify ordinary tabletop contact as task failure or automatically
   increase gains to eliminate contact-related error. The cause of these particular peaks
   has not been established. Interpret error alongside images, task progress, temperature,
   and torque. Interpolating targets slowly does not bound actual motion or holding load.
   Controller phase names and
   `segment complete` are bookkeeping, not evidence that the physical task succeeded.
6. **Treat placement, return, and torque release as distinct outcomes.** The foam moved
   during contact: this was deposit into its final opening, not demonstrated precision against
   a fixed target. The run returned folded but remained powered at the saved completion
   snapshot. A powered folded pose differed from its initial unpowered rest. Do not infer
   current hardware state from an old report or release torque merely because replay ended.

## Reference material and limits

- Latest shutdown: after stopping the third attempt, the operator explicitly requested
  immediate torque release while supporting the arm. The `adaptive_run_01` controller was
  terminated; watchdog torque release is expected but was not independently measured.
  That controller is no longer live. See the third-attempt report's shutdown addendum.
- Demonstration: `/home/gavin/Documents/robot-experiments/teleop-demo-20260915-182851/`.
  `task_crop/episode/` contains 7.91 s of task motion and initial retreat. Full `samples.jsonl`
  also contains the later return motion. The experimental controller interpolated its
  **sent actions** using raw demonstration time 8.2–17.9 s, not crop-relative time.
- Successful controller archive:
  `/home/gavin/Documents/robot-experiments/reference-pick-place/run_02/`.
  Its README/result describe historical completion state; fresh controller status must be
  checked before any new controller starts. Do not launch the archived controller as a
  ready-to-use recipe: known completion and parking defects are tracked in [TODO.md](TODO.md).
- Wrist and external video provide about 25 and 5 distinct frames/s respectively. Exporting
  at 60 Hz repeats frames; it does not improve camera feedback rate. Use timestamp sidecars
  for alignment. A moved camera invalidates assumptions tied to its earlier framing.
- The [changed-scene review](experiments/2026-09-15-adaptive-scene.md) records local +3° elbow,
  wrist-flex, and shoulder-lift probes, plus an operator-assisted normal controller handoff.
  The replacement `reference_controller.py` fixes clipped-target completion and permits
  cumulative pan offsets up to 30° in <=3° increments. It remains experimental; the camera
  loop, parking, and thermal limitations are not resolved by that change.
- Changed-scene success: `adaptive_run_01/scale_05/` under `reference-pick-place/` contains
  commands, paired checkpoint images, telemetry snapshots, and operator-confirmed result.
  A frozen `adaptive_success/` interval in the public-upload bundle preserves 977 telemetry
  rows, 2,451 wrist frames and 490 external frames, with packet-verified AVI and aligned MP4.
  Earlier failed retries are retained as evidence; publication still requires local HF login.

## Reduce time between seeing and acting

The reviewed run's camera timestamp intervals were approximately 40 ms at the wrist and
200 ms externally, with maxima of 47 ms and 201 ms through the saved result. The controller
captured continuously, refreshed latest JPEGs at most every 200 ms, and ran its command loop
at a nominal 60 Hz. The tens-of-seconds gaps between commands were not gaps in camera capture.
These timestamps measure host capture completion, not sensor exposure or full inference
latency. The archive does not separate model reasoning, tool round trips, and other waiting.

For subsequent operation, prepare the demonstrated sequence and decisions before motion;
retrieve both fresh views plus telemetry together; make each checkpoint answer a specific
question such as alignment, capture, or deposit. Avoid repeating broad analysis at each step.
After controller fixes and validation, use continuous execution of appropriate demonstrated
segments with monitoring instead of an unconditional conversational pause at every waypoint.
For frequent visual corrections, a future local image-feedback loop could handle the measured
adjustments while the agent handles task planning and exceptions. That loop is not implemented
or validated. Measure capture-to-observation, decision, command, and motion-completion times
separately before attributing delay to camera hardware or claiming a speed improvement.

## How to accumulate the next lesson

For each authorized experiment, preserve an episode report containing:

- Date, task, robot/calibration/configuration, camera arrangement, starting pose, and scene.
- Reference and exact phase/timestamp; requested command, clipped/sent command, measured
  response, and paired before/after images. Record failed corrections too.
- Separate observed pickup, lift, placement, return, and disconnect outcomes; include object
  or fixture movement and any operator intervention.
- Temperatures, torques, tracking errors, and time spent holding under load.
- A short supported lesson, its applicable conditions, confidence/repetitions, and what remains
  untested. Keep proposed experiments distinct from demonstrated behavior.

Promote repeated findings into this guide, and revise them when evidence contradicts them.
The next useful measurements are local joint-to-image responses near the demonstrated
pregrasp pose, followed by supervised repeatability checks after controller fixes. These are
proposals for future authorized sessions, not completed measurements. Compare success count,
corrections, elapsed time, loaded hold time, and peak temperature to establish improvement.
