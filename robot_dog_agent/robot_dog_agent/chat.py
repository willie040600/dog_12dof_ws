"""Talk to the robot dog in the terminal (spec 2026-11-10, 5.3).

    ros2 run robot_dog_agent chat [--model gemini-3.5-flash-lite] [--no-robot]
Needs GEMINI_API_KEY, and Isaac Sim with nav.launch.py running unless --no-robot (a stand-in robot that arrives
at once: no ROS, for trying the conversation). Ctrl+C during a navigation cancels it; exit ends the chat.
"""
import argparse
import functools
import json
import os
import sys

from robot_dog_agent import places, tools
from robot_dog_agent.agent import Agent
from robot_dog_agent.gemini import GeminiClient, GeminiError
from robot_dog_agent.robot import FakeRobot

DEFAULT_MODEL = 'gemini-3.5-flash-lite'  # gemini-3.6-flash: only 20 requests per day on the free tier
NO_KEY = """找不到環境變數 GEMINI_API_KEY。
1. 到 https://aistudio.google.com/ 申請 API key（免費額度就夠用）。
2. 在 ~/.bashrc 加一行：export GEMINI_API_KEY=你的key
3. 開一個新的終端機，再執行一次。"""
BAD_KEY = """環境變數 GEMINI_API_KEY 的格式不對：裡面有空白、引號或非英文的字元。
請檢查 ~/.bashrc 的那一行，應該是：export GEMINI_API_KEY=你的key（等號兩邊沒有空白，key 不加引號以外的東西）。"""


def usable_key(raw):
    """The API key without surrounding whitespace, or None when it cannot be one (never echoed anywhere)."""
    key = (raw or '').strip()
    if not key or not all(33 <= ord(c) <= 126 and c not in '"\'' for c in key):  # printable ASCII, no quotes
        return None
    return key


def build_agent(robot, api_key, model, say=print, on_call=None):
    """An Agent on Gemini that drives `robot`; progress and tool calls are reported through say."""
    def show_call(call, result):
        reason = '（%s）' % result['reason'] if result.get('reason') else ''
        say('  [工具] %s %s → %s%s' % (call.name, json.dumps(call.args, ensure_ascii=False), result.get('status'),
                                      reason))
        if on_call:
            on_call(call, result)

    def show_progress(distance):
        if distance > 0.0:  # 0: Nav2 has no path (yet)
            say('  剩 %.1f m' % distance)

    toolbox = tools.Toolbox(robot, places.load_body_truth(), on_progress=show_progress)
    llm = GeminiClient(api_key, model, tools.system_instruction(), tools.DECLARATIONS, say=say)
    return Agent(llm, toolbox, on_call=show_call)


def run_chat(agent, read=input, say=print):
    """Read commands until exit / end of input and print the agent's answers."""
    while True:
        try:
            line = read('你> ').strip()
        except (EOFError, KeyboardInterrupt):
            return
        if line.lower() in ('exit', 'quit'):
            return
        if not line:
            continue
        try:
            say('狗> %s' % agent.ask(line))
        except GeminiError as e:
            say('[錯誤] %s' % e)
        except KeyboardInterrupt:
            say('（已取消）')
        except Exception as e:  # keep the chat alive; only the class name, a message could quote the key
            say('[錯誤] 程式內部錯誤（%s），這一句沒有完成。' % type(e).__name__)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--model', default=DEFAULT_MODEL, help='Gemini model id')
    parser.add_argument('--no-robot', action='store_true', help='a stand-in robot: no ROS, no simulator')
    args, _ = parser.parse_known_args(argv)
    api_key = usable_key(os.environ.get('GEMINI_API_KEY'))
    if not api_key:
        print(BAD_KEY if os.environ.get('GEMINI_API_KEY', '').strip() else NO_KEY)
        return 2
    if args.no_robot:
        robot = FakeRobot()
    else:
        from robot_dog_agent.nav2_robot import Nav2Robot  # needs rclpy
        print('等 Nav2 啟動…（要先開好 Isaac Sim 和 nav.launch.py；Ctrl+C 取消）', flush=True)
        try:
            robot = Nav2Robot()
        except RuntimeError as e:
            print('[錯誤] Nav2 啟動了，但機器狗還沒有定位（%s）。請確認 nav.launch.py 正常執行。' % e)
            return 1
        except KeyboardInterrupt:
            print('（已取消）')
            return 130
    try:
        say = functools.partial(print, flush=True)  # progress must show at once, also through a pipe
        say('可以去的地方：' + '、'.join('%s（%s）' % (p.name, p.id) for p in places.PLACES))
        say('Ctrl+C 可以中斷導航；輸入 exit 結束。')
        run_chat(build_agent(robot, api_key, args.model, say=say), say=say)
    finally:
        if not args.no_robot:
            robot.close()
    return 0


if __name__ == '__main__':
    sys.exit(main())
