import pytest

from robot_dog_agent import agent
from robot_dog_agent.agent import Agent, Call, Reply

KITCHEN, START = Call('c1', 'go_to_place', {'place': 'kitchen'}), Call('c2', 'go_to_place', {'place': 'start'})
WHERE = Call('c0', 'get_pose', {})
REACHED, FAILED = {'status': 'reached'}, {'status': 'failed', 'reason': 'Nav2 status FAILED'}
SKIPPED = {'status': 'skipped', 'reason': 'an earlier step did not succeed'}


class ScriptedLLM:
    """Answers send() with the given replies in turn; an exception in the list is raised instead."""

    def __init__(self, replies):
        self.replies, self.sent, self.resets = list(replies), [], 0

    def send(self, text=None, results=None):
        self.sent.append((text, results))
        reply = self.replies.pop(0)
        if isinstance(reply, BaseException):
            raise reply
        return reply

    def reset(self):
        self.resets += 1


class RecordingToolbox:
    def __init__(self, results_by_name=None, error=None):
        self.results, self.calls, self.error = results_by_name or {}, [], error

    def call(self, name, args):
        self.calls.append((name, args))
        if self.error:
            raise self.error
        return self.results.get(args.get('place', name), REACHED)


def test_text_only():
    llm, box = ScriptedLLM([Reply('你好', [])]), RecordingToolbox()
    assert Agent(llm, box).ask('哈囉') == '你好'
    assert llm.sent == [('哈囉', None)] and box.calls == []


def test_one_call_then_text():
    llm, box = ScriptedLLM([Reply('', [KITCHEN]), Reply('到廚房了', [])]), RecordingToolbox()
    assert Agent(llm, box).ask('去廚房') == '到廚房了'
    assert llm.sent == [('去廚房', None), (None, [(KITCHEN, REACHED)])]
    assert box.calls == [('go_to_place', {'place': 'kitchen'})]


def test_two_calls_in_one_reply_run_in_order():
    llm, box = ScriptedLLM([Reply('', [KITCHEN, START]), Reply('回來了', [])]), RecordingToolbox()
    assert Agent(llm, box).ask('先去廚房，再回起點') == '回來了'
    assert box.calls == [('go_to_place', {'place': 'kitchen'}), ('go_to_place', {'place': 'start'})]
    assert llm.sent[1] == (None, [(KITCHEN, REACHED), (START, REACHED)])


def test_later_calls_are_skipped_after_a_failure():
    llm, box = ScriptedLLM([Reply('', [KITCHEN, START]), Reply('廚房去不了', [])]), RecordingToolbox({'kitchen': FAILED})
    assert Agent(llm, box).ask('先去廚房，再回起點') == '廚房去不了'
    assert box.calls == [('go_to_place', {'place': 'kitchen'})]
    assert llm.sent[1] == (None, [(KITCHEN, FAILED), (START, SKIPPED)])


def test_get_pose_ok_does_not_skip_the_next_call():
    ok = {'status': 'ok', 'x': 0.0, 'y': 0.0}
    llm, box = ScriptedLLM([Reply('', [WHERE, KITCHEN]), Reply('好', [])]), RecordingToolbox({'get_pose': ok})
    Agent(llm, box).ask('看一下位置再去廚房')
    assert llm.sent[1] == (None, [(WHERE, ok), (KITCHEN, REACHED)])


def test_text_with_calls():
    llm, box = ScriptedLLM([Reply('我現在過去', [KITCHEN]), Reply('到了', [])]), RecordingToolbox()
    assert Agent(llm, box).ask('去廚房') == '到了'
    assert box.calls == [('go_to_place', {'place': 'kitchen'})]


def test_empty_reply():
    assert Agent(ScriptedLLM([Reply('', [])]), RecordingToolbox()).ask('嗯') == agent.EMPTY_REPLY
    assert Agent(ScriptedLLM([Reply('  \n', [])]), RecordingToolbox()).ask('嗯') == agent.EMPTY_REPLY


def test_too_many_rounds():
    llm, box = ScriptedLLM([Reply('', [KITCHEN])] * 8), RecordingToolbox()
    assert Agent(llm, box).ask('一直走') == agent.TOO_MANY_ROUNDS
    assert len(llm.sent) == 8 and len(box.calls) == 7 and llm.resets == 1


def test_on_call_sees_every_result():
    seen = []
    llm, box = ScriptedLLM([Reply('', [KITCHEN, START]), Reply('不行', [])]), RecordingToolbox({'kitchen': FAILED})
    Agent(llm, box, on_call=lambda call, result: seen.append((call, result))).ask('先去廚房，再回起點')
    assert seen == [(KITCHEN, FAILED), (START, SKIPPED)]


def test_conversation_is_reset_after_an_exception():
    llm = ScriptedLLM([Reply('', [KITCHEN]), RuntimeError('rate limited'), Reply('好', [])])
    a = Agent(llm, RecordingToolbox())
    with pytest.raises(RuntimeError, match='rate limited'):
        a.ask('去廚房')
    assert llm.resets == 1
    assert a.ask('哈囉') == '好'


def test_conversation_is_reset_after_ctrl_c_in_a_tool():
    llm = ScriptedLLM([Reply('', [KITCHEN])])
    with pytest.raises(KeyboardInterrupt):
        Agent(llm, RecordingToolbox(error=KeyboardInterrupt())).ask('去廚房')
    assert llm.resets == 1


def test_an_on_call_that_raises_ends_the_command_and_resets():
    class Closing(BaseException):
        pass

    def on_call(call, result):
        raise Closing()

    llm = ScriptedLLM([Reply('', [KITCHEN, START])])
    box = RecordingToolbox()
    with pytest.raises(Closing):
        Agent(llm, box, on_call=on_call).ask('先去廚房，再回起點')
    assert box.calls == [('go_to_place', {'place': 'kitchen'})] and len(llm.sent) == 1 and llm.resets == 1
