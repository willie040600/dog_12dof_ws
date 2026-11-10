import json
import os

import pytest
import requests

from robot_dog_agent import gemini, tools
from robot_dog_agent.agent import Call
from robot_dog_agent.gemini import GeminiClient, GeminiError

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')
MODEL, INSTRUCTION = 'gemini-3.6-flash', tools.system_instruction()


def load(name):
    """A reply recorded from the real API: {request, http_status, retry_after, body}."""
    with open(os.path.join(DATA, name + '.json')) as f:
        return json.load(f)


class Response:
    def __init__(self, status_code, body=None, text=None, retry_after=None):
        self.status_code, self._body = status_code, body
        self.text = text if text is not None else json.dumps(body)
        self.headers = {'Retry-After': retry_after} if retry_after else {}

    def json(self):
        if self._body is None:
            raise ValueError('not JSON')
        return self._body


def recorded(name):
    rec = load(name)
    return Response(rec['http_status'], rec['body'], retry_after=rec['retry_after'])


class FakePost:
    """Stands in for requests.post: records every call and answers with the given responses in turn."""

    def __init__(self, responses):
        self.responses, self.calls = list(responses), []

    def __call__(self, url, headers=None, json=None, timeout=None):
        self.calls.append({'url': url, 'headers': headers, 'json': json, 'timeout': timeout})
        response = self.responses.pop(0)
        if isinstance(response, BaseException):
            raise response
        return response


def client(responses, key='k'):
    post, sleeps, said = FakePost(responses), [], []
    c = GeminiClient(key, MODEL, INSTRUCTION, tools.DECLARATIONS, post=post, sleep=sleeps.append, say=said.append)
    return c, post, sleeps, said


def test_build_request_matches_what_the_api_accepted():
    assert gemini.build_request(MODEL, INSTRUCTION, tools.DECLARATIONS, '用一句話打招呼', None, None) == \
        load('text')['request']
    assert gemini.build_request(MODEL, INSTRUCTION, tools.DECLARATIONS, '去廚房', None, None) == load('call')['request']
    call = gemini.parse_reply(load('call')['body'])[1].calls[0]
    result = {'status': 'reached', 'x': 2.5, 'y': -0.6, 'yaw_deg': -90, 'seconds': 20.0}
    assert gemini.build_request(MODEL, INSTRUCTION, tools.DECLARATIONS, None, [(call, result)],
                                load('call')['body']['id']) == load('after_result')['request']


def test_parse_recorded_replies():
    iid, reply = gemini.parse_reply(load('text')['body'])
    assert iid == load('text')['body']['id'] and reply.text.strip() and reply.calls == []
    iid, reply = gemini.parse_reply(load('call')['body'])
    assert reply.calls == [Call(load('call')['body']['steps'][1]['id'], 'go_to_place', {'place': 'kitchen'})]
    assert reply.text == ''
    assert gemini.parse_reply(load('after_result')['body'])[1].text.strip()
    names = [(c.name, c.args) for c in gemini.parse_reply(load('two_calls')['body'])[1].calls]
    assert names == [('go_to_place', {'place': 'kitchen'}), ('go_to_place', {'place': 'start'})]


def test_parse_a_body_without_steps():
    assert gemini.parse_reply({'id': 'x', 'status': 'failed'})[1] == gemini.Reply('', [])


def test_key_goes_in_the_header_only():
    c, post, _, _ = client([recorded('text')], key='secret-key')
    assert c.send(text='哈囉').text.strip()
    call = post.calls[0]
    assert call['url'] == GeminiClient.URL and 'secret-key' not in call['url'] and call['timeout'] == 60
    assert call['headers']['x-goog-api-key'] == 'secret-key' and 'secret-key' not in json.dumps(call['json'])


def test_conversation_is_chained_and_reset():
    c, post, _, _ = client([recorded('call'), recorded('after_result'), recorded('text')])
    reply = c.send(text='去廚房')
    c.send(results=[(reply.calls[0], {'status': 'reached'})])
    c.reset()
    c.send(text='哈囉')
    assert 'previous_interaction_id' not in post.calls[0]['json']
    assert post.calls[1]['json']['previous_interaction_id'] == load('call')['body']['id']
    assert post.calls[1]['json']['input'][0]['call_id'] == reply.calls[0].id
    assert 'previous_interaction_id' not in post.calls[2]['json']


def test_retries_after_429_and_5xx():
    c, post, sleeps, said = client([Response(429, {'error': {'message': 'quota'}}, retry_after='7'),
                                    Response(503, {'error': {'message': 'busy'}}), recorded('text')])
    assert c.send(text='哈囉').text.strip()
    assert sleeps == [7, 20] and len(post.calls) == 3
    assert len(said) == 2 and '被限流' in said[0] and '伺服器' in said[1]


def test_gives_up_after_three_retries():
    c, post, sleeps, _ = client([Response(429, {'error': {'message': 'quota'}})] * 4)
    with pytest.raises(GeminiError, match='quota'):
        c.send(text='哈囉')
    assert sleeps == [10, 20, 40] and len(post.calls) == 4


def test_a_long_retry_after_is_not_waited_for():
    c, post, sleeps, _ = client([Response(429, {'error': {'message': 'daily quota'}}, retry_after='3600')])
    with pytest.raises(GeminiError, match='約 1.0 小時'):
        c.send(text='哈囉')
    assert sleeps == [] and len(post.calls) == 1


def test_waiting_times_are_readable():
    assert [gemini.duration(s) for s in (45, 170, 3599, 3600, 85169)] == \
        ['45 秒', '約 3 分鐘', '約 60 分鐘', '約 1.0 小時', '約 23.7 小時']


def test_connection_errors_are_retried():
    c, _, sleeps, said = client([requests.ConnectionError('no route'), requests.Timeout('slow'), recorded('text')])
    assert c.send(text='哈囉').text.strip()
    assert sleeps == [10, 20] and len(said) == 2


def test_bad_key_is_not_retried_and_not_leaked():
    c, post, sleeps, _ = client([recorded('error_key')], key='secret-key')
    with pytest.raises(GeminiError) as e:
        c.send(text='哈囉')
    assert len(post.calls) == 1 and sleeps == []
    assert 'secret-key' not in str(e.value) and 'GEMINI_API_KEY' in str(e.value)


def test_expired_conversation_starts_over():
    c, post, _, said = client([recorded('text'), recorded('error_previous_id'), recorded('text')])
    c.send(text='哈囉')
    assert c.send(text='再一次').text.strip()
    assert 'previous_interaction_id' in post.calls[1]['json'] and 'previous_interaction_id' not in post.calls[2]['json']
    assert post.calls[2]['json']['input'] == '再一次' and said == ['之前的對話已過期，重新開始']


def test_a_bad_request_without_a_conversation_is_an_error():
    c, post, _, said = client([recorded('error_previous_id')])
    with pytest.raises(GeminiError, match='invalid argument'):
        c.send(text='哈囉')
    assert len(post.calls) == 1 and said == []


def test_tool_results_are_not_resent_into_a_new_conversation():
    # results answer calls of the lost conversation: starting over with them makes no sense
    c, post, _, _ = client([recorded('call'), recorded('error_previous_id')])
    reply = c.send(text='去廚房')
    with pytest.raises(GeminiError):
        c.send(results=[(reply.calls[0], {'status': 'reached'})])
    assert len(post.calls) == 2


def test_non_json_error_body():
    c, _, sleeps, _ = client([Response(502, text='<html>Bad Gateway</html>')] * 3 + [recorded('text')])
    assert c.send(text='哈囉').text.strip() and sleeps == [10, 20, 40]


# --- final review ---

def test_a_reply_cut_off_half_way_is_retried():
    c, _, sleeps, _ = client([requests.exceptions.ChunkedEncodingError('cut'), recorded('text')])
    assert c.send(text='哈囉').text.strip() and sleeps == [10]


def test_other_request_errors_do_not_leak_the_key():
    # requests refuses a header value with leading whitespace and quotes the whole value in its message
    error = requests.exceptions.InvalidHeader("Invalid leading whitespace in header value: ' secret-key'")
    c, post, sleeps, _ = client([error], key=' secret-key')
    with pytest.raises(GeminiError) as e:
        c.send(text='哈囉')
    assert 'secret-key' not in str(e.value) and 'InvalidHeader' in str(e.value)
    assert sleeps == [] and len(post.calls) == 1


def test_a_200_that_is_not_json_is_an_error():
    c, _, _, _ = client([Response(200, text='<html>proxy login</html>')])
    with pytest.raises(GeminiError, match='看不懂'):
        c.send(text='哈囉')
