"""The tool-call loop between the user, the model and the tools (spec 2026-11-10, 5.2). Plain Python.

The model is anything with
    send(text=None, results=None) -> Reply; results = [(Call, result dict), ...] answers the calls of the last Reply
    reset(); forgets the conversation
gemini.GeminiClient is the real one.
"""
from collections import namedtuple

Call = namedtuple('Call', 'id name args')
Reply = namedtuple('Reply', 'text calls')  # text '' when there is none; calls is a list of Call

TOO_MANY_ROUNDS = '這個指令用了太多步驟，我先停下來。'
EMPTY_REPLY = '（模型沒有回覆，請再說一次。）'
SKIPPED = {'status': 'skipped', 'reason': 'an earlier step did not succeed'}


class Agent:
    MAX_ROUNDS = 8  # sends to the model per command

    def __init__(self, llm, toolbox, on_call=None):
        self.llm, self.toolbox, self.on_call = llm, toolbox, on_call

    def ask(self, text):
        """The model's answer to one command, after running the tool calls it asks for."""
        try:
            reply, sends = self.llm.send(text=text), 1
            while reply.calls and sends < self.MAX_ROUNDS:
                reply, sends = self.llm.send(results=self._run(reply.calls)), sends + 1
        except BaseException:
            self.llm.reset()  # the conversation on the server may end in a call without a result
            raise
        if reply.calls:
            self.llm.reset()
            return TOO_MANY_ROUNDS
        return reply.text if reply.text.strip() else EMPTY_REPLY

    def _run(self, calls):
        """Run the calls in order; after one that did not succeed the rest are skipped."""
        results, failed = [], False
        for call in calls:
            result = dict(SKIPPED) if failed else self.toolbox.call(call.name, call.args)
            failed = failed or result.get('status') not in ('ok', 'reached')
            if self.on_call:
                self.on_call(call, result)
            results.append((call, result))
        return results
