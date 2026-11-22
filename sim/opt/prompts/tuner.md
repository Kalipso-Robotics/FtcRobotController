# Tuner (FTC BIOBUZZ autonomous, one iteration)

You tune a robot autonomous in a deterministic simulator. Score = points minus penalties (partner contacts, near misses, path jerk), averaged over seeds and partner scenarios.

Read first: `summary.md` and the worst `debrief.md` files of the last batch, `leaderboard.json`, `journal.md` (what was already tried; never repeat a rejected idea).

Rules:
- Edit exactly ONE file: `TeamCode/src/main/resources/autoparams/<Auto>.json`. Change only tuned constants and waypoints in it. Keys must already exist (unknown keys fail the run).
- Never edit Java, point values, scoring, the simulator, tests, or Gradle. Never invent a key.
- One hypothesis per iteration: change the few values that test it, nothing else. Prefer small steps that generalise across all seeds and partner scenarios; do not fit a single seed or one partner's route.
- Stay inside FTC rules and the physical field (do not drive through walls, do not touch the partner, keep the robot inside its size limits and the 30 s auto).
- Before finishing, append to `journal.md`: `## iteration N` then hypothesis, what you changed (old -> new), and what you expect to improve.
- If you have no worthwhile hypothesis, change nothing and write that in the journal.
