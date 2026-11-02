---
name: ship-to-dev
description: >
  Ship the work on the current branch end to end: run the gates (ruff, the
  compose check, the suite in the dev image), commit, push, open a PR to dev,
  watch CI, merge it, then delete the branch locally and on the remote. User
  invoked (/ship-to-dev); invoking it is the go-ahead to merge into dev.
metadata:
  origin: project
---

# Ship to dev

The loop this repo's day-to-day work runs: a change on a branch ends up
merged into `dev`, and the branch is deleted locally and on the remote.
Rebuilding the `syncai_backend` container is not part of it. This skill
is the order the steps run in, with the commands that actually work on
this machine (no ROS, no ruff, no pytest on the host — every gate runs in
a container or through `uvx`).

**Invoking it is the authorization to merge into `dev`.** Nothing else in
it asks. It still *stops and reports* rather than pushing through on any of
the stop conditions below — a red gate, a red check, a branch that is not
an ancestor of `origin/dev` — because those are the cases where merging
would land something nobody looked at.

## 0. Where you start

- On a `<type>/<kebab>` branch cut from `dev` (`feat`/`feature`, `fix`/
  `bugfix`, `refactor`, `docs` are the prefixes merged so far). On `dev`
  itself with uncommitted work: cut the branch first, named for the change.
  Never on `main`: it is the default branch for GitHub's sake and holds
  only the first commit; everything lives on `dev`.
- `git status` shows only this change. `.vscode/` and
  `.claude/settings.local.json` are the user's; leave them out of the
  commits and say so.

## 1. Gates

CI (`.github/workflows/ci.yml`) is two jobs; run the same three things here
first, because the pytest job alone takes ten minutes of runner time
(it builds the dev image and clones `syncai_common` every run).

**ruff, pinned to CI's version** (the host has no ruff; `uvx` is in
`~/.local/bin`; the host `python3` is 3.15 and must not be trusted for
anything in this repo):

```bash
uvx ruff@0.16.8 check .          # the version ci.yml pins; bump both together
docker compose config --quiet    # a malformed compose file, without starting anything
```

**The suite, in the dev image.** Build the image once
(`docker build --target dev -t syncai-backend-dev .`); it only needs
rebuilding on a `requirements.txt` change. The source is bind-mounted.
`syncai_common` is cloned and built into a workspace kept at
`~/.cache/syncai-backend/ros2_ws` so that minute is paid once, not per run
(`rm -rf` it to start over — e.g. after `interface.repos` moves):

```bash
WS=~/.cache/syncai-backend/ros2_ws; mkdir -p "$WS"
docker run --rm -v "$WS":/ros2_ws -v "$PWD":/ros2_ws/src/syncai_backend \
    syncai-backend-dev bash -lc '
  set -eo pipefail
  cd /ros2_ws
  [ -d src/syncai_common ] || vcs import < src/syncai_backend/interface.repos
  [ -f install/setup.bash ] || colcon-build --packages-select syncai_common >/dev/null
  source install/setup.bash
  cd src/syncai_backend
  python3 -m pytest test/ -q -ra'
```

Expected: ~1400 passed, 1 skipped (`test_copyright`, on purpose). Read the
result with these in mind:

- **Collection errors mean `syncai_common` is missing**, not broken tests
  — the gateway and router files import it plainly. Check the workspace.
- The ament linters (`test_flake8`, `test_pep257`) are part of the suite,
  and CI runs them: a D202 in a file the change touched has failed a PR
  whose author only ran the map tests. Never run a subset as the gate.
- The image runs as uid 1000 on purpose; root would make the
  `ini_not_writable` test fail against working code.
- A change to `requirements.txt`, `setup.py` or `package.xml` also needs
  `docker build --target runtime .`: CI's test job ends by building that
  stage, and a missing entry point or `data_files` breaks there while the
  dev stage stays green.

A failure in a test the change does not touch is not automatically a flake:
re-run it alone, and if it still fails, `git stash -u`, run it on clean
`dev`, `git stash pop`. Only "fails on dev too" or "passes alone" may be
reported as unrelated — with that evidence in the PR body.

**Stop** on a failure the change caused.

## 2. Commits

Conventional Commits, one concern each, bottom-up — `helpers/`, `tasks/` →
`repositories/`, `gateways/` → `services/` → `interfaces/rest/`,
`temporal/` → `main.py` → docs — so the diff reads in dependency order.
Subjects are imperative with the scope in parens
(`fix(maps): skip the reload when the map was switched away from`). The
body says *why*. End every message with the attribution line the session
gives.

`README.md` is the long-form reference and records *why* things are the
way they are: a change in behaviour updates its section and, when it
alters what CLAUDE.md describes, CLAUDE.md too — in a `docs:` commit of
their own, or folded into the change with the body saying so.

## 3. Push and PR

```bash
git push -u origin <branch>
gh pr create --base dev --head <branch> --title "<first commit's subject>" --body-file <scratchpad>/pr.md
```

`--body-file`, never an inline `--body`: a heredoc keeps backticks, `$`
and newlines intact. The body, for a reviewer who was not there: **Why**,
**What**, **Tests** (what ran, what passed, any unrelated failure and the
evidence it is unrelated), and **Not verified** when true — on the robot,
against a live Temporal server, on Humble's rclpy, with the real nav
stack. Most changes here have something in that last section; say it
rather than imply the container run proved it. End it with the session's
PR attribution line.

## 4. CI

```bash
gh pr checks <n> --watch --interval 30 >/dev/null 2>&1; gh pr checks <n>
```

Run it in the background and wait for the notification; do not poll. Two
checks: `ruff + compose` (about a minute) and `pytest (ROS 2 Humble)`
(ten minutes or so: dev image, `syncai_common` clone and build, the suite,
then the runtime image off the same cache).

- **All pass:** go on.
- **Any fail:** read the log first (`gh run view --job <id> --log-failed`).
  A ruff failure that passed locally means the pinned versions drifted
  apart — fix the pin, not the code. A pytest failure that passed locally
  is usually the missing-`syncai_common` collection error (the clone in CI
  needs `interface.repos` to point at a live branch) or a linter the
  local run skipped. Fix and push if it is the change; **stop and report**
  otherwise.
- A check that reads *cancelled* after a force-push is the `concurrency`
  group doing its job, not a failure: the new run is the one to watch.

There is no review workflow in this repo; nothing to report as skipped.

## 5. Merge and clean up

```bash
gh pr merge <n> --merge
git switch dev && git pull --ff-only origin dev
git fetch --prune origin
git merge-base --is-ancestor <branch> origin/dev   # stop if this fails
git push origin --delete <branch>                  # skip if already gone
git branch -d <branch>                             # -d, never -D
```

`--merge`, matching the merge commits `dev` already has.

## 6. Report

Short, to the operator: the PR link and that it merged (with the merge
commit), the branch deleted locally and on the remote, the gate and CI
results, and anything that could not be verified.
