# CLAUDE.md

Behavioral rules cut common LLM coding mistake. Merge with project instructions as needed.

**Tradeoff:** Rules bias caution over speed. Trivial task, use judgment.

## 0. This Repo

FTC team BioBuzz 2627, season DECODE. Fork of the Pedro Pathing Quickstart on FTC SDK 11.1.0.

| Path | What | Editable |
|---|---|---|
| `TeamCode/src/main/java/.../teamcode/` | Our code | Yes |
| `TeamCode/.../teamcode/pedroPathing/Constants.java` | Tuned follower values | Only with tuning data |
| `TeamCode/.../teamcode/pedroPathing/Tuning.java` | Pedro tuning OpModes | Rarely — upstream file |
| `TeamCode/.../teamcode/examples/` | Reference examples | Copy out, don't edit in place |
| `FtcRobotController/` | Vendored SDK | **Never** |
| `build.dependencies.gradle` | Vendor deps | Ask before bumping |

## 1. Think Before Coding

**No assume. No hide confusion. Surface tradeoffs.**

Before build:
- State assumptions. Uncertain, ask.
- Multiple readings exist, show all — no silent pick.
- Simpler way exist, say. Push back when warranted.
- Unclear, stop. Name confusion. Ask.
- Big project + bug fix: run test before + after.

FTC-specific things you cannot know — ask, never guess:
- Hardware config names (`hardwareMap.get(..., "left_front")`). Must match the Driver Station config exactly. A guessed name is a crash on init, not a compile error.
- Motor directions, encoder ports, gear ratios, ticks-per-rev, servo ranges. Physical facts.
- Which alliance / start position an auto targets.
- Field coordinates. Don't invent poses.

## 2. Simplicity First

**Least code that solve problem. Nothing speculative.**

- No feature past ask.
- No abstraction for single-use code.
- No "flexibility"/"configurability" not requested.
- No error handling for impossible case.
- Write 200 lines, could be 50, rewrite.

Ask self: "Senior engineer call this overcomplicated?" Yes, simplify.

FTC corollary: one OpMode, one job. No subsystem framework, command scheduler, or
hardware abstraction layer unless asked. A TeleOp that reads sticks and sets powers is
a TeleOp that reads sticks and sets powers.

## 3. Surgical Changes

**Touch only what must. Clean only own mess.**

Edit existing code:
- No "improve" nearby code, comment, format.
- No refactor thing not broken.
- Match existing style, even if you differ.
- Notice unrelated dead code, mention — no delete.

Changes make orphans:
- Remove imports/variables/functions YOUR change made unused.
- No remove pre-existing dead code unless asked.

Test: every changed line trace direct to user request.

FTC corollary: tuned numbers are data, not code. PIDF gains, `FollowerConstants`,
feedforward, servo positions, shooter RPM — never "clean up", round, or re-derive them.
They came off a real robot. Change one only when asked, and say which one changed.

## 4. Goal-Driven Execution

**Define success criteria. Loop till verified.**

Turn task into verifiable goal:
- "Add validation" → "Write tests for invalid input, then pass them"
- "Fix the bug" → "Write test that reproduce, then pass"
- "Refactor X" → "Tests pass before + after"

Multi-step task, state brief plan:
```
1. [Step] → verify: [check]
2. [Step] → verify: [check]
3. [Step] → verify: [check]
```

Strong criteria let you loop alone. Weak criteria ("make it work") need constant clarify.

**FTC has no unit tests and often no compiler in the loop.** Say which rung you reached:

1. `./gradlew :TeamCode:compileDebugJavaWithJavac` passed. Strongest available.
2. No Android SDK in this environment → compile not run. Say so. Then verify every
   API call against real code in this repo (`FtcRobotController/.../samples/` for SDK,
   `pedroPathing/Tuning.java` for Pedro) and say which file you matched against.
3. Neither possible → say the code is unverified and name what the human must check.

Never call code "working" when it only ever got read. Never call it "tested" when it
never ran on a robot.

Robot-verifiable goals, stated up front:
- "Add field centric" → "Push stick away from driver, robot moves away from driver at any heading"
- "Fix the shooter" → "Telemetry shows measured RPM within 50 of target before the feeder fires"
- "Tune the follower" → "Run Tuning OpMode X, record value, put it in Constants.java"

## 5. Safety Rails

Non-negotiable. Violating these breaks a robot or fails an inspection.

- `OpMode.loop()` never blocks. No `sleep()`, no `while` waiting on a sensor. Use a
  state machine plus `ElapsedTime`.
- `LinearOpMode` loops gate on `opModeIsActive()`.
- Every loop iteration that follows a path calls `follower.update()` exactly once.
- Set motor direction and `ZeroPowerBehavior` in `init()`, not mid-match.
- Units: Pedro poses are inches and **radians**. FTC IMU is whatever `AngleUnit` you
  pass. Convert at the boundary, state the unit in the variable name or a comment.
- New OpMode goes in `TeamCode`, has `@TeleOp`/`@Autonomous` with a `name` and `group`,
  and never lands in `FtcRobotController`.

## 6. Self-Improvement

- Find error. Log it + reason why happen.

---

**Rules working if:** fewer needless diff changes, fewer rewrites from overcomplication,
clarify questions come before build not after mistake, and no OpMode ever crashes on init
because a hardware name was guessed.
