"""Turn a natural-language command into a validated action plan using a local OpenAI-compatible LLM."""
import json
import re
import urllib.error
import urllib.request

SYSTEM_PROMPT = """You control a robot arm in a simulation. Convert the user's request into a JSON plan.

Objects the robot can pick: {objects}
Destinations: {destinations}
The robot is currently holding: {holding}

Allowed actions:
- {{"action": "pick", "object": <object>}}   (grasp the object, lift it and keep holding it)
- {{"action": "place", "destination": <destination>}}   (put the held object into the destination)
- {{"action": "pick_and_place", "object": <object>, "destination": <destination>}}
- {{"action": "home"}}   (move the robot to its home position)

Respond with ONLY a JSON object, no other text:
{{"reply": "<one short sentence to the user>", "plan": [<actions in order>]}}

Rules:
- Use only the exact object and destination names listed above.
- NEVER choose a destination the user did not ask for. If the user only says to pick up / grab / lift
  an object, use "pick".
- "place" only works while holding something; "it" usually means the held object.
- If the user says "it" (or similar) while holding nothing and names no object, return an empty plan
  and ask which object they mean. Never use "home" unless the user asks to go home / reset / rest.
- The robot can hold only one object at a time.
- If the request mentions something that does not exist or is not a robot task, return an empty plan and explain in "reply".
- "all cubes" means every object listed above."""


class PlanError(Exception):
    pass


def _norm(name):
    return re.sub(r'[^a-z0-9]', '', str(name).lower())


def _resolve(name, candidates):
    lookup = {_norm(c): c for c in candidates}
    return lookup.get(_norm(name))


def _extract_json(text):
    text = re.sub(r'<think>.*?</think>', '', text, flags=re.S).strip()
    text = re.sub(r'^```(?:json)?\s*|\s*```$', '', text).strip()
    try:
        return json.loads(text)
    except json.JSONDecodeError:
        start, end = text.find('{'), text.rfind('}')
        if start != -1 and end > start:
            return json.loads(text[start:end + 1])
        raise


class LLMPlanner:
    def __init__(self, url, model, timeout_s=120):
        self.url, self.model, self.timeout_s = url, model, timeout_s

    def _chat(self, system, user):
        body = {
            'model': self.model,
            'temperature': 0,
            'max_tokens': 400,
            'messages': [{'role': 'system', 'content': system}, {'role': 'user', 'content': user}],
        }
        req = urllib.request.Request(self.url, json.dumps(body).encode(), {'Content-Type': 'application/json'})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                data = json.load(resp)
        except urllib.error.URLError as e:
            raise PlanError(f'Cannot reach the LLM server at {self.url} ({e}). Is `geniex serve` running?')
        return data['choices'][0]['message']['content']

    def plan(self, command, objects, destinations, holding=None):
        """Return (reply, plan, raw_text). ``plan`` only contains validated actions."""
        system = SYSTEM_PROMPT.format(objects=', '.join(objects) or '(none)',
                                      destinations=', '.join(destinations) or '(none)',
                                      holding=holding or 'nothing')
        raw = self._chat(system, command)
        try:
            data = _extract_json(raw)
        except (json.JSONDecodeError, ValueError):
            raise PlanError(f'LLM did not return valid JSON: {raw!r}')
        if not isinstance(data, dict) or not isinstance(data.get('plan', []), list):
            raise PlanError(f'LLM returned an unexpected structure: {raw!r}')

        def obj_of(step):
            obj = _resolve(step.get('object'), objects)
            if obj is None:
                raise PlanError(f"Unknown object {step.get('object')!r} (known: {', '.join(objects)})")
            return obj

        def dest_of(step):
            dest = _resolve(step.get('destination'), destinations)
            if dest is None:
                raise PlanError(f"Unknown destination {step.get('destination')!r} (known: {', '.join(destinations)})")
            return dest

        # Walk the plan tracking what the gripper holds, so impossible sequences are rejected up front.
        plan, held = [], holding
        for step in data.get('plan', []):
            if isinstance(step, str):  # small models sometimes emit "home" instead of {"action": "home"}
                step = {'action': step}
            action = _norm(step.get('action', '')) if isinstance(step, dict) else ''
            if action == 'home':
                plan.append({'action': 'home'})
            elif action == 'pick':
                obj = obj_of(step)
                if held:
                    raise PlanError(f'Cannot pick {obj}: already holding {held}. Place it first.')
                plan.append({'action': 'pick', 'object': obj})
                held = obj
            elif action == 'place':
                if not held:
                    raise PlanError('Cannot place: the robot is not holding anything.')
                plan.append({'action': 'place', 'destination': dest_of(step)})
                held = None
            elif action == 'pickandplace':
                obj, dest = obj_of(step), dest_of(step)
                if held and held != obj:
                    raise PlanError(f'Cannot pick {obj}: already holding {held}. Place it first.')
                plan.append({'action': 'pick_and_place', 'object': obj, 'destination': dest})
                held = None
            else:
                raise PlanError(f'Unsupported action in plan: {step!r}')
        return str(data.get('reply', '')), plan, raw
