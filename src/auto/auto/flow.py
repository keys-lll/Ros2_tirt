"""
The delivery mission: three trips per round, driven by operator input.

Navigation, input, output and sleeping are injected, so the flow is testable
without ROS and a camera can replace the keyboard later.
"""

FRONT_TABLES = (1, 4)
BACK_TABLES = (2, 3, 5)
QUIT = ('q', 'quit')


class MissionAborted(Exception):
    """The operator quit, or input ended."""


def allowed_second_tables(first):
    """Round two must draw from the other zone than round one."""
    return BACK_TABLES if first in FRONT_TABLES else FRONT_TABLES


class Mission:
    def __init__(self, settings, navigate, ask, say, sleep):
        self.settings = settings
        self.navigate = navigate
        self.ask = ask
        self.say = say
        self.sleep = sleep

    def ask_line(self, prompt):
        """Ask the operator; q (or end of input) aborts the mission."""
        try:
            answer = self.ask(prompt).strip()
        except EOFError:
            raise MissionAborted('輸入結束')
        if answer.lower() in QUIT:
            raise MissionAborted('操作手結束')
        return answer

    def ask_table(self, prompt, allowed=(1, 2, 3, 4, 5), allow_blank=False):
        """Ask for a table number until it is one of `allowed`; None if blank is allowed."""
        while True:
            answer = self.ask_line(prompt)
            if not answer and allow_blank:
                return None
            if answer.isdecimal() and 1 <= int(answer) <= 5:
                if int(answer) in allowed:
                    return int(answer)
                self.say(f'第二回合要換區，只能抽 {"、".join(map(str, allowed))} 桌')
            else:
                self.say('請輸入 1~5 的桌號')

    def run(self):
        """Round one at any table, then optionally round two from the other zone."""
        first = self.ask_table('第一回合抽到幾桌（1~5，q 結束）> ')
        self.run_round(first)
        allowed = allowed_second_tables(first)
        second = self.ask_table(
            f'第二回合抽到幾桌（只能 {"、".join(map(str, allowed))}；直接按 Enter 結束）> ',
            allowed=allowed, allow_blank=True)
        if second is not None:
            self.run_round(second)

    def go(self, name):
        """Navigate to `name`; after a failure the operator retries, skips or quits."""
        while True:
            result = self.navigate(name)
            self.say(result.text)
            if result.ok:
                return
            while True:
                choice = self.ask_line(
                    '這一步沒成功。[r] 重試  [s] 跳過（我已手動把車放到位）  [q] 結束 > ').lower()
                if choice == 's':
                    return
                if choice == 'r':
                    break
                self.say('請輸入 r、s 或 q')

    def hold(self):
        self.say(f'靜止 {self.settings.stop_seconds:g} 秒…')
        self.sleep(self.settings.stop_seconds)

    def run_round(self, table):
        """Trip 1 seat the guest, trip 2 deliver the set meal, trip 3 deliver the extra order."""
        s = self.settings
        spot = s.tables[table - 1]

        self.say(f'===== 第一趟 帶位到 {table} 號桌 =====')
        self.ask_line(f'車子在 {s.start}。按 Enter 出發帶位到 {table} 號桌（q 結束）> ')
        self.go(spot)
        self.hold()
        order = self.ask_line('請裁判給點餐卡，輸入點餐內容（可直接按 Enter 略過）> ')
        self.go(s.prep)

        self.say(f'===== 第二趟 送固定餐到 {table} 號桌 =====')
        self.ask_line(f'請把餐點放上托盤（點餐：{order or "未輸入"}），按 Enter 出發送到 {table} 號桌 > ')
        self.go(spot)
        self.hold()
        extra = self.ask_line('請裁判給加點卡，輸入加點項目（可樂 / 橘子果汁 / 白開水；Enter 略過）> ')
        self.go(s.prep)

        self.say(f'===== 第三趟 送加點餐到 {table} 號桌 =====')
        self.ask_line(f'請放上加點餐點（{extra or "未輸入"}），按 Enter 出發送到 {table} 號桌 > ')
        self.go(spot)
        self.hold()
        self.ask_line(f'請裁判給「不須加點」卡，按 Enter 後回 {s.start} > ')
        self.go(s.start)
        self.say(f'{table} 號桌這個回合完成')
