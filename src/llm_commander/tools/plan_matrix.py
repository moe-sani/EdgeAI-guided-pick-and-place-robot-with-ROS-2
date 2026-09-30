"""Regression check for the planning prompt: run after any change to llm.SYSTEM_PROMPT or the model.

Usage (GenieX server running, workspace sourced):
  python3 src/llm_commander/tools/plan_matrix.py [MODEL] [URL]
"""
import sys
import time

from llm_commander.llm import LLMPlanner, PlanError

MODEL = sys.argv[1] if len(sys.argv) > 1 else 'ai-hub-models/Qwen3-VL-4B-Instruct'
URL = sys.argv[2] if len(sys.argv) > 2 else 'http://127.0.0.1:18181/v1/chat/completions'
OBJECTS = ['BlueCube', 'GreenCube', 'RedCube']
DESTS = ['BlueTray', 'GreenTray', 'Box']

# (held object, command, expected plan as "action arg" strings, or None = must not move the robot)
CASES = [
    (None, 'put it in the blue tray', None),
    (None, 'drop it in the box', None),
    (None, 'Pick up the red cube', ['pick RedCube']),
    (None, 'grab the blue one', ['pick BlueCube']),
    (None, 'Pick up the red cube and place it in the blue tray.', ['pick RedCube', 'place BlueTray']),
    (None, 'put the green one in the green tray, then move the blue cube to the box',
     ['pick GreenCube', 'place GreenTray', 'pick BlueCube', 'place Box']),
    (None, 'Put all cubes in the box', 'all-to-box'),
    (None, 'pick up the yellow cube', None),
    (None, "what's the weather like?", None),
    (None, 'go home', ['home']),
    ('RedCube', 'put it in the blue tray', ['place BlueTray']),
    ('RedCube', 'drop it in the box', ['place Box']),
    ('RedCube', 'go home', ['home']),
    ('RedCube', 'pick up the green cube', None),
    ('RedCube', 'put the red cube in the green tray', ['place GreenTray']),
]


def norm(plan):
    out = []
    for s in plan:
        if s['action'] == 'pick_and_place':  # equivalent to pick + place
            out += [f"pick {s['object']}", f"place {s['destination']}"]
        else:
            out.append(' '.join(str(v) for v in s.values()))
    return out


def main():
    p = LLMPlanner(URL, MODEL, 300)
    ok = 0
    for held, cmd, want in CASES:
        t = time.time()
        try:
            _, plan, _ = p.plan(cmd, OBJECTS, DESTS, held)
            got = norm(plan)
        except PlanError as e:
            got = []
            err = str(e)[:80]
        else:
            err = ''
        if want == 'all-to-box':
            good = sorted(got) == sorted([f'pick {o}' for o in OBJECTS] + ['place Box'] * 3)
        else:
            good = got == (want or [])
        ok += good
        print(f"{'PASS' if good else 'FAIL'} {time.time() - t:4.1f}s held={held!s:8} {cmd[:48]!r:50} -> "
              f"{got or err or '(no motion)'}", flush=True)
    print(f'\n{ok}/{len(CASES)} passed with {MODEL}')
    sys.exit(0 if ok == len(CASES) else 1)


if __name__ == '__main__':
    main()
