#!/usr/bin/env bash
# Autonomous tuning loop. Never pushes. Commits only on branch auto-opt/<date>.
# usage: sim/opt/loop.sh [--dry-run]
# env: AUTO (default RedDecideAuto) ITERATIONS (200) SEEDS (3) JOBS (4) EPS (0.5) TOL (2) TARGET (none) HOURS (8)
#      STUB=1 stub tuner + critic (no claude)   SKIP_TESTS=1 skip the node/gradle test step   PATIENCE (15)
# state (git-ignored, under build/): TeamCode/build/auto-opt/{leaderboard.json,journal.md,loop.log,iter-N/}
set -euo pipefail

if [[ "${1:-}" != "--dry-run" && -z "${LOOP_CAFFEINATED:-}" ]] && command -v caffeinate >/dev/null; then
  LOOP_CAFFEINATED=1 exec caffeinate -dims "$0" "$@"
fi

DRY=0; [[ "${1:-}" == "--dry-run" ]] && DRY=1
ROOT="$(cd "$(dirname "$0")/../.." && pwd)"; cd "$ROOT"
AUTO="${AUTO:-RedDecideAuto}"; ITERATIONS="${ITERATIONS:-200}"; SEEDS="${SEEDS:-3}"; JOBS="${JOBS:-4}"
EPS="${EPS:-0.5}"; TOL="${TOL:-2}"; TARGET="${TARGET:-}"; HOURS="${HOURS:-8}"; PATIENCE="${PATIENCE:-15}"; STUB="${STUB:-0}"
STATE="$ROOT/TeamCode/build/auto-opt"; LB="$STATE/leaderboard.json"; JOURNAL="$STATE/journal.md"
PF_REL="TeamCode/src/main/resources/autoparams/$AUTO.json"; PF="$ROOT/$PF_REL"
BRANCH="auto-opt/$(date +%F)"; OPT="$ROOT/sim/opt"
export JAVA_HOME="${JAVA_HOME:-/Applications/Android Studio.app/Contents/jbr/Contents/Home}"
[[ -d "$ROOT/sim/.node/bin" ]] && PATH="$ROOT/sim/.node/bin:$PATH"

if (( DRY )); then
  cat <<PLAN
loop.sh dry run (nothing executed)
  auto $AUTO  params $PF_REL  branch $BRANCH  state $STATE
  limits: $ITERATIONS iterations, $HOURS h, $PATIENCE without improvement${TARGET:+, target mean $TARGET}
  batch: --seeds $SEEDS --jobs $JOBS   gate: mean > best + $EPS and worst >= bestWorst - $TOL
  tuner: $([[ $STUB == 1 ]] && echo "stub-tuner.mjs" || echo "claude -p (Read Grep Glob Edit) prompts/tuner.md")   critic: $([[ $STUB == 1 ]] && echo "stub (ACCEPT)" || echo "claude -p prompts/critic.md")
  tests: $([[ "${SKIP_TESTS:-0}" == 1 ]] && echo skipped || echo "node --test in sim/app sim/physics sim/opt; gradle sim tests")
  per iteration: tuner -> diff check (only autoparams/*.json) -> batch -> gate -> critic -> tests -> commit + leaderboard, else git checkout
  working tree: $([[ -z "$(git status --porcelain)" ]] && echo clean || echo "DIRTY (a real run would refuse)")
PLAN
  exit 0
fi

say() { printf '[%s] %s\n' "$(date +%T)" "$*" | tee -a "$STATE/loop.log"; }
revert() { git checkout -- "$ROOT/TeamCode/src/main/resources/autoparams" 2>/dev/null || true; }
trap 'revert; say "stopped; uncommitted tuner edits reverted"' EXIT

[[ -z "$(git status --porcelain)" ]] || { echo "dirty working tree: commit or stash first" >&2; trap - EXIT; exit 1; }
[[ -f "$PF" ]] || { echo "missing $PF_REL" >&2; trap - EXIT; exit 1; }
mkdir -p "$STATE"; touch "$JOURNAL"
git rev-parse --verify -q "$BRANCH" >/dev/null && git checkout -q "$BRANCH" || git checkout -q -b "$BRANCH"

batch() { node "$OPT/batch.mjs" --auto "$AUTO" --params "$PF" --seeds "$SEEDS" --jobs "$JOBS" --out "$1" >"$1.log" 2>&1; }
record() { # record <scores.json> <iteration> <commit>
  node -e 'const fs=require("fs");const[,,s,l,i,c]=process.argv;const a=JSON.parse(fs.readFileSync(s)).aggregate;
    const d=fs.existsSync(l)?JSON.parse(fs.readFileSync(l)):{history:[]};
    d.best={mean:a.mean,worst:a.worst,iteration:+i,commit:c};d.history.push(d.best);fs.writeFileSync(l,JSON.stringify(d,null,2)+"\n")' "$1" "$LB" "$2" "$3"
}
tests() {
  (( ${SKIP_TESTS:-0} )) && return 0
  for d in sim/app sim/physics sim/opt; do (cd "$ROOT/$d" && node --test >/dev/null 2>&1) || { say "tests failed in $d"; return 1; }; done
  ./gradlew :TeamCode:testDebugUnitTest --tests "*kalipsorobotics.sim.*" --offline -q >/dev/null 2>&1 || { say "gradle sim tests failed"; return 1; }
}

if [[ ! -f "$LB" ]]; then
  say "baseline batch"; batch "$STATE/iter-0" || true
  [[ -f "$STATE/iter-0/scores.json" ]] || { say "baseline produced no scores.json, see $STATE/iter-0.log"; exit 1; }
  node "$OPT/gate.mjs" "$STATE/iter-0/scores.json" - >/dev/null || { say "baseline had errored runs"; exit 1; }
  record "$STATE/iter-0/scores.json" 0 "$(git rev-parse --short HEAD)"
fi

stale=0
for ((i = 1; i <= ITERATIONS; i++)); do
  (( SECONDS < HOURS * 3600 )) || { say "time limit ($HOURS h)"; break; }
  (( stale < PATIENCE )) || { say "no improvement in $PATIENCE iterations"; break; }
  if [[ -n "$TARGET" ]] && node -e 'process.exit(JSON.parse(require("fs").readFileSync(process.argv[1])).best.mean>=+process.argv[2]?0:1)' "$LB" "$TARGET"; then say "target $TARGET reached"; break; fi
  OUT="$STATE/iter-$i"; say "== iteration $i/$ITERATIONS (stale $stale)"

  # 1. tuner
  if [[ $STUB == 1 ]]; then node "$OPT/stub-tuner.mjs" "$i" "$PF"; echo "## iteration $i: stub nudge" >>"$JOURNAL"
  else
    claude -p "$(cat "$OPT/prompts/tuner.md")

Iteration $i. Params file: $PF. Last batch: $STATE/iter-$((i - 1))/summary.md (and per-run debrief.md beside it; earlier iterations in $STATE). Leaderboard: $LB. Journal (append here): $JOURNAL." \
      --allowedTools Read Grep Glob Edit >"$STATE/tuner-$i.log" 2>&1 || say "tuner exited nonzero"
  fi

  # 2. diff check: only autoparams/*.json may change
  bad="$(git status --porcelain | awk '{print $2}' | grep -vE '^TeamCode/src/main/resources/autoparams/[^/]+\.json$' || true)"
  if [[ -n "$bad" ]]; then say "tuner touched forbidden files, reverting: $bad"; git checkout -- . ; git clean -fdq -- $bad; stale=$((stale + 1)); continue; fi
  if [[ -z "$(git status --porcelain)" ]]; then say "tuner changed nothing"; stale=$((stale + 1)); continue; fi

  # 3. batch + 4. gate
  if ! batch "$OUT" || ! node "$OPT/gate.mjs" "$OUT/scores.json" "$LB" "$EPS" "$TOL" | tee -a "$STATE/loop.log"; then
    say "rejected by gate"; revert; stale=$((stale + 1)); continue
  fi

  # 5. critic (veto only)
  if [[ $STUB == 1 ]]; then verdict="$(node "$OPT/stub-tuner.mjs" --critic | tail -1)"
  else
    verdict="$(claude -p "$(cat "$OPT/prompts/critic.md")

Diff:
$(git diff)

Scores: $OUT/scores.json  Summary: $OUT/summary.md  Journal: $JOURNAL" --allowedTools Read Grep Glob 2>&1 | tee "$STATE/critic-$i.log" | tail -1)"
  fi
  [[ "$verdict" == "VERDICT: ACCEPT" ]] || { say "critic: ${verdict:-no verdict}"; revert; stale=$((stale + 1)); continue; }

  # 6. tests, 7. accept
  tests || { revert; stale=$((stale + 1)); continue; }
  git add -- "$PF_REL"
  git commit -q -m "auto-opt: $AUTO iteration $i" -m "Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>"
  record "$OUT/scores.json" "$i" "$(git rev-parse --short HEAD)"; stale=0
  say "ACCEPTED iteration $i: $(node -e 'const b=JSON.parse(require("fs").readFileSync(process.argv[1])).best;console.log("mean",b.mean,"worst",b.worst)' "$LB")"
done
say "done"
