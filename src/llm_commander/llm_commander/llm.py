"""Turn a natural-language command into a validated action plan using a local OpenAI-compatible LLM."""
import json
import re
import urllib.error
import urllib.request

SYSTEM_PROMPT = """You control a robot arm in a simulation. Convert the user's request into a JSON plan.

Objects the robot can pick: {objects}
Destinations: {destinations}

Allowed actions:
- {{"action": "pick_and_place", "object": <object>, "destination": <destination>}}
- {{"action": "home"}}   (move the robot to its home position)

Respond with ONLY a JSON object, no other text:
{{"reply": "<one short sentence to the user>", "plan": [<actions in order>]}}

Rules:
- Use only the exact object and destination names listed above.
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

    def plan(self, command, objects, destinations):
        """Return (reply, plan, raw_text). ``plan`` only contains validated actions."""
        system = SYSTEM_PROMPT.format(objects=', '.join(objects) or '(none)',
                                      destinations=', '.join(destinations) or '(none)')
        raw = self._chat(system, command)
        try:
            data = _extract_json(raw)
        except (json.JSONDecodeError, ValueError):
            raise PlanError(f'LLM did not return valid JSON: {raw!r}')
        if not isinstance(data, dict) or not isinstance(data.get('plan', []), list):
            raise PlanError(f'LLM returned an unexpected structure: {raw!r}')

        plan = []
        for step in data.get('plan', []):
            if isinstance(step, str):  # small models sometimes emit "home" instead of {"action": "home"}
                step = {'action': step}
            action = _norm(step.get('action', '')) if isinstance(step, dict) else ''
            if action == 'home':
                plan.append({'action': 'home'})
            elif action == 'pickandplace':
                obj = _resolve(step.get('object'), objects)
                dest = _resolve(step.get('destination'), destinations)
                if obj is None:
                    raise PlanError(f"Unknown object {step.get('object')!r} (known: {', '.join(objects)})")
                if dest is None:
                    raise PlanError(f"Unknown destination {step.get('destination')!r} (known: {', '.join(destinations)})")
                plan.append({'action': 'pick_and_place', 'object': obj, 'destination': dest})
            else:
                raise PlanError(f'Unsupported action in plan: {step!r}')
        return str(data.get('reply', '')), plan, raw
