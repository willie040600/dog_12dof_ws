from robot_dog_agent import chat
from robot_dog_agent.agent import Agent, Call
from robot_dog_agent.gemini import GeminiError
from robot_dog_agent.robot import FakeRobot


class FakeAgent:
    """Answers ask() with the given strings in turn; an exception in the list is raised instead."""

    def __init__(self, answers=()):
        self.answers, self.asked = list(answers), []

    def ask(self, text):
        self.asked.append(text)
        answer = self.answers.pop(0) if self.answers else '好'
        if isinstance(answer, BaseException):
            raise answer
        return answer


def run(lines, answers=()):
    """run_chat fed with `lines` (an exception in the list is raised by read); returns (agent, what was said)."""
    agent, said, lines = FakeAgent(answers), [], list(lines)

    def read(prompt):
        line = lines.pop(0)
        if isinstance(line, BaseException):
            raise line
        return line

    chat.run_chat(agent, read=read, say=said.append)
    return agent, said


def test_blank_lines_are_not_sent():
    agent, _ = run(['', '   ', '去廚房', 'exit'])
    assert agent.asked == ['去廚房']


def test_reply_is_printed():
    _, said = run(['去廚房', 'exit'], ['到了'])
    assert said == ['狗> 到了']


def test_exit_words_and_end_of_input():
    for last in ('exit', 'quit', ' EXIT ', EOFError(), KeyboardInterrupt()):
        agent, _ = run(['哈囉', last, '不該被讀到'])
        assert agent.asked == ['哈囉'], last


def test_errors_do_not_end_the_chat():
    agent, said = run(['一', '二', 'exit'], [GeminiError('被限流'), '好'])
    assert agent.asked == ['一', '二'] and said == ['[錯誤] 被限流', '狗> 好']


def test_ctrl_c_during_a_command():
    agent, said = run(['一', '二', 'exit'], [KeyboardInterrupt(), '好'])
    assert agent.asked == ['一', '二'] and said == ['（已取消）', '狗> 好']


def test_main_without_a_key(monkeypatch, capsys):
    monkeypatch.delenv('GEMINI_API_KEY', raising=False)
    assert chat.main(['--no-robot']) == 2
    out = capsys.readouterr().out
    assert 'GEMINI_API_KEY' in out and 'aistudio.google.com' in out and '.bashrc' in out


def test_default_model_has_a_usable_free_quota():
    # gemini-3.6-flash allows 20 requests per day on the free tier (measured 2026-11-10): eight commands
    assert chat.DEFAULT_MODEL == 'gemini-3.5-flash-lite'


def test_build_agent_with_the_fake_robot():
    said = []
    agent = chat.build_agent(FakeRobot(), 'k', 'gemini-3.6-flash', say=said.append)
    assert isinstance(agent, Agent) and agent.llm.api_key == 'k' and agent.llm.model == 'gemini-3.6-flash'
    assert agent.toolbox.call('get_pose', {})['status'] == 'ok'
    agent.toolbox.on_progress(1.234)
    agent.toolbox.on_progress(0.0)      # Nav2 has no path (yet): nothing worth showing
    assert said == ['  剩 1.2 m']


def test_tool_calls_are_shown():
    said = []
    agent = chat.build_agent(FakeRobot(), 'k', 'm', say=said.append)
    agent.on_call(Call('c1', 'go_to_place', {'place': 'kitchen'}), {'status': 'blocked', 'reason': 'on an obstacle'})
    assert said == ['  [工具] go_to_place {"place": "kitchen"} → blocked（on an obstacle）']
