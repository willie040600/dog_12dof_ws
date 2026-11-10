"""Gemini through the Interactions API, as the model of agent.Agent (spec 2026-11-10, 2 and 5.4).

Plain REST with requests: one endpoint, and the system Python that ROS uses cannot pip-install the SDK.
The request and reply shapes are the ones recorded from the real API in test/data/.
"""
import json
import time

import requests

from robot_dog_agent.agent import Call, Reply

LONGEST_WAIT = 120  # s; a Retry-After beyond this (a used-up daily quota) is reported instead of waited for


class GeminiError(Exception):
    """A request that failed for good; the message is for the user."""


def build_request(model, system_instruction, tools, text, results, previous_id):
    """The request body for a user text or, with results = [(Call, result dict), ...], for tool results."""
    if results is None:
        content = text
    else:
        content = [{'type': 'function_result', 'name': call.name, 'call_id': call.id,
                    'result': [{'type': 'text', 'text': json.dumps(result, ensure_ascii=False)}]}
                   for call, result in results]
    request = {'model': model, 'input': content, 'system_instruction': system_instruction, 'tools': tools}
    if previous_id:
        request['previous_interaction_id'] = previous_id
    return request


def parse_reply(body):
    """(interaction id, Reply) of a response body."""
    texts, calls = [], []
    for step in body.get('steps') or []:
        if step.get('type') == 'model_output':
            texts += [part.get('text', '') for part in step.get('content') or [] if part.get('type') == 'text']
        elif step.get('type') == 'function_call':
            calls.append(Call(step.get('id'), step.get('name'), step.get('arguments') or {}))
    return body.get('id'), Reply(''.join(texts), calls)


def error_message(response):
    """The server's own words for a failed response; error bodies come as an object or a one-element list."""
    try:
        body = response.json()
    except ValueError:
        return (response.text or '').strip()[:200]
    if isinstance(body, list) and body:
        body = body[0]
    error = body.get('error') if isinstance(body, dict) else None
    return str(error.get('message', error)) if isinstance(error, dict) else json.dumps(body)[:200]


class GeminiClient:
    URL = 'https://generativelanguage.googleapis.com/v1beta/interactions'
    RETRY_WAITS = (10, 20, 40)  # s, for 429, 5xx and connection errors without a Retry-After
    TIMEOUT = 60                # s per request

    def __init__(self, api_key, model, system_instruction, tools, post=requests.post, sleep=time.sleep, say=print):
        self.api_key, self.model, self.system_instruction, self.tools = api_key, model, system_instruction, tools
        self.post, self.sleep, self.say = post, sleep, say
        self.previous_id = None

    def reset(self):
        """Forget the conversation: the next send starts a new one."""
        self.previous_id = None

    def send(self, text=None, results=None):
        response = self._post(text, results, self.previous_id)
        if response.status_code == 400 and self.previous_id and results is None:
            # the server no longer has the conversation (kept for one day on the free tier)
            self.say('之前的對話已過期，重新開始')
            self.previous_id = None
            response = self._post(text, results, None)
        if response.status_code != 200:
            message = error_message(response)
            hint = 'API key 無效，請檢查 GEMINI_API_KEY。' if 'API key' in message else ''
            raise GeminiError('%sGemini 拒絕了請求（HTTP %d）：%s' % (hint, response.status_code, message))
        self.previous_id, reply = parse_reply(response.json())
        return reply

    def _post(self, text, results, previous_id):
        """One request, retried while the server is busy or out of reach. Returns the last response."""
        request = build_request(self.model, self.system_instruction, self.tools, text, results, previous_id)
        headers = {'x-goog-api-key': self.api_key, 'Content-Type': 'application/json'}
        for wait in self.RETRY_WAITS + (None,):
            try:
                response = self.post(self.URL, headers=headers, json=request, timeout=self.TIMEOUT)
            except (requests.ConnectionError, requests.Timeout) as e:
                problem, limited = '連不上 Gemini（%s）' % type(e).__name__, False
            else:
                if response.status_code != 429 and response.status_code < 500:
                    return response
                limited = response.status_code == 429
                problem = 'HTTP %d：%s' % (response.status_code, error_message(response))
                try:
                    wait = wait and float(response.headers.get('Retry-After') or wait)
                except ValueError:
                    pass
                if wait and wait > LONGEST_WAIT:
                    raise GeminiError('Gemini 的額度用完了，要等 %.0f 秒才能再試（%s）' % (wait, problem))
            if wait is None:
                raise GeminiError('Gemini 沒有回應，已重試 %d 次（%s）' % (len(self.RETRY_WAITS), problem))
            self.say('%s，等 %.0f 秒再試（%s）' % ('被限流' if limited else '伺服器沒有回應', wait, problem))
            self.sleep(wait)
