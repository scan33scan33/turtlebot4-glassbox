#!/usr/bin/env python3
"""ToyScript — a tiny DSL for high-level robot tasks (toy cleaning, fetch, etc.).

Primitives (provided by a Robot binding):
    GO_TO(x, y)            navigate the base to (x, y)            -> "arrived"/"blocked"
    PUSH_TO(x, y)          drive into/through, pushing ahead      -> "pushed"/"stalled"
    FIND(name)             locate a detected object               -> {x,y} or NONE
    WAIT(secs)             dwell in place                          -> "done"
    SCAN_FOR(name[, deg])  step-and-stare sweep for an object      -> {x,y} or NONE
    PUSH_AWAY(x, y[, step]) push the ball at (x,y) away            -> "pushed"/"stalled"
    PUSH_THROUGH([dist])   straight push until stall               -> "done"/"pinned"
    PUSH_TO_WALL([name])   lidar-goal + camera-steer wall push     -> "pinned"/...
    PUSH_TO_GOAL([x, y])   push the ball to a human-set goal       -> "at goal"/...
                           (no args = the goal marked in the web UI)
    FOLLOW([name][, m][, fast])
                           follow at a standoff (move, not push)   -> "done"/"not-found"
                           With NO arguments the target, the standoff and `fast`
                           come from whatever the spoken phrase resolved to (the
                           navigator's _follow_params), which is why a single
                           programs/follow.toy covers a person, a dog and a ball.
                           Explicit arguments override that. `fast` TRUE picks the
                           lidar + velocity predictor, and only applies to small
                           targets (it replaces the old FOLLOW_FAST primitive).
Built-ins (pure):
    POINT(x, y)            make a {x,y} value
    PRINT expr             log a line

Control flow:
    SET name = expr
    IF expr THEN ... [ELSE ...] END
    WHILE expr DO ... END

Values: numbers, strings, NONE, TRUE/FALSE, points ({x,y}). Operators:
    + - * /   == != < > <= >=   AND OR NOT   and member access  p.x / p.y

Run a file:   python3 toyscript.py program.toy        (uses a MockRobot)
Embed:        Interpreter(robot).run(source)
"""
import sys, re, math, time

# ── Tokenizer ──────────────────────────────────────────────────────────────────
_TOK = re.compile(r"""
    \s*(?:
      (\#[^\n]*)                      # comment
    | (\n)                            # newline (statement separator)
    | (-?\d+\.\d+|-?\d+)              # number
    | "([^"]*)"                       # string
    | (==|!=|<=|>=|[<>])              # comparison
    | ([-+*/=().,])                   # punctuation / arithmetic
    | ([A-Za-z_][A-Za-z0-9_]*)        # identifier / keyword
    )
""", re.VERBOSE)

KEYWORDS = {"SET","IF","THEN","ELSE","END","WHILE","DO","PRINT","AND","OR","NOT",
            "NONE","TRUE","FALSE"}

def tokenize(src):
    toks, pos = [], 0
    while pos < len(src):
        m = _TOK.match(src, pos)
        if not m or m.end() == pos:
            if src[pos:].strip() == "": break
            raise SyntaxError(f"bad token near: {src[pos:pos+20]!r}")
        pos = m.end()
        comment, nl, num, string, cmp_, punct, ident = m.groups()
        if comment is not None: continue
        if nl is not None: toks.append(("NL", "\n")); continue
        if num is not None: toks.append(("NUM", float(num)))
        elif string is not None: toks.append(("STR", string))
        elif cmp_ is not None: toks.append(("OP", cmp_))
        elif punct is not None: toks.append(("OP", punct))
        elif ident is not None:
            U = ident.upper()
            toks.append((U, U) if U in KEYWORDS else ("ID", ident))
    toks.append(("EOF", None))
    return toks


# ── Parser (recursive descent → AST tuples) ─────────────────────────────────────
class Parser:
    def __init__(self, toks): self.t = toks; self.i = 0
    def peek(self): return self.t[self.i]
    def kind(self): return self.t[self.i][0]
    def next(self): tok = self.t[self.i]; self.i += 1; return tok
    def eat(self, kind):
        if self.kind() != kind:
            raise SyntaxError(f"expected {kind}, got {self.peek()}")
        return self.next()
    def skip_nl(self):
        while self.kind() == "NL": self.next()

    def parse(self):
        body = self.block(("EOF",))
        self.eat("EOF"); return body

    def block(self, terminators):
        stmts = []
        while True:
            self.skip_nl()
            if self.kind() in terminators or self.kind() == "EOF": break
            stmts.append(self.statement())
        return stmts

    def statement(self):
        k = self.kind()
        if k == "SET":
            self.next(); name = self.eat("ID")[1]; self.eat_op("=")
            return ("set", name, self.expr())
        if k == "PRINT":
            self.next(); return ("print", self.expr())
        if k == "IF":
            self.next(); cond = self.expr(); self.eat("THEN")
            then = self.block(("ELSE", "END")); els = []
            if self.kind() == "ELSE": self.next(); els = self.block(("END",))
            self.eat("END"); return ("if", cond, then, els)
        if k == "WHILE":
            self.next(); cond = self.expr(); self.eat("DO")
            body = self.block(("END",)); self.eat("END")
            return ("while", cond, body)
        return ("expr", self.expr())          # bare call, e.g. GO_TO(...)

    def eat_op(self, op):
        if self.peek() != ("OP", op): raise SyntaxError(f"expected '{op}', got {self.peek()}")
        self.next()

    # expression precedence: OR < AND < NOT < compare < add < mul < unary < atom
    def expr(self): return self.or_()
    def or_(self):
        n = self.and_()
        while self.kind() == "OR": self.next(); n = ("or", n, self.and_())
        return n
    def and_(self):
        n = self.not_()
        while self.kind() == "AND": self.next(); n = ("and", n, self.not_())
        return n
    def not_(self):
        if self.kind() == "NOT": self.next(); return ("not", self.not_())
        return self.cmp_()
    def cmp_(self):
        n = self.add()
        while self.peek()[0] == "OP" and self.peek()[1] in ("==","!=","<",">","<=",">="):
            op = self.next()[1]; n = ("cmp", op, n, self.add())
        return n
    def add(self):
        n = self.mul()
        while self.peek() in (("OP","+"),("OP","-")):
            op = self.next()[1]; n = ("bin", op, n, self.mul())
        return n
    def mul(self):
        n = self.unary()
        while self.peek() in (("OP","*"),("OP","/")):
            op = self.next()[1]; n = ("bin", op, n, self.unary())
        return n
    def unary(self):
        if self.peek() == ("OP","-"): self.next(); return ("neg", self.unary())
        return self.postfix()
    def postfix(self):
        n = self.atom()
        while self.peek() == ("OP","."):            # member access  p.x
            self.next(); n = ("member", n, self.eat("ID")[1])
        return n
    def atom(self):
        k, v = self.peek()
        if k == "NUM": self.next(); return ("num", v)
        if k == "STR": self.next(); return ("str", v)
        if k == "NONE": self.next(); return ("none",)
        if k == "TRUE": self.next(); return ("bool", True)
        if k == "FALSE": self.next(); return ("bool", False)
        if k == "OP" and v == "(":
            self.next(); e = self.expr(); self.eat_op(")"); return e
        if k == "ID":
            self.next()
            if self.peek() == ("OP","("):           # function call
                self.next(); args = []
                if self.peek() != ("OP",")"):
                    args.append(self.expr())
                    while self.peek() == ("OP",","): self.next(); args.append(self.expr())
                self.eat_op(")"); return ("call", v, args)
            return ("var", v)
        raise SyntaxError(f"unexpected {self.peek()}")


# ── Interpreter ─────────────────────────────────────────────────────────────────
class StopProgram(Exception): pass

class Interpreter:
    def __init__(self, robot, log=print, max_steps=100000, on_call=None):
        self.robot = robot; self.log = log; self.on_call = on_call
        self.env = {}; self.steps = 0; self.max_steps = max_steps
        self.stopped = False

    def run(self, source):
        ast = Parser(tokenize(source)).parse()
        self.exec_block(ast)
        return self.env

    def stop(self): self.stopped = True

    def exec_block(self, stmts):
        for s in stmts: self.exec_stmt(s)

    def exec_stmt(self, s):
        if self.stopped: raise StopProgram()
        self.steps += 1
        if self.steps > self.max_steps: raise RuntimeError("max steps exceeded (runaway loop?)")
        kind = s[0]
        if kind == "set":
            self.env[s[1]] = self.eval(s[2])
        elif kind == "print":
            self.log(self._fmt(self.eval(s[1])))
        elif kind == "expr":
            self.eval(s[1])
        elif kind == "if":
            if self._truthy(self.eval(s[1])): self.exec_block(s[2])
            else: self.exec_block(s[3])
        elif kind == "while":
            while self._truthy(self.eval(s[1])):
                self.exec_block(s[2])
                if self.stopped: raise StopProgram()

    def eval(self, e):
        k = e[0]
        if k == "num": return e[1]
        if k == "str": return e[1]
        if k == "none": return None
        if k == "bool": return e[1]
        if k == "var":
            if e[1] not in self.env: raise NameError(f"undefined variable '{e[1]}'")
            return self.env[e[1]]
        if k == "member":
            obj = self.eval(e[1])
            if isinstance(obj, dict) and e[2] in obj: return obj[e[2]]
            raise TypeError(f"cannot read .{e[2]} of {obj!r}")
        if k == "neg": return -self._num(self.eval(e[1]))
        if k == "not": return not self._truthy(self.eval(e[1]))
        if k == "and":
            return self.eval(e[2]) if self._truthy(self.eval(e[1])) else self.eval(e[1])
        if k == "or":
            l = self.eval(e[1]); return l if self._truthy(l) else self.eval(e[2])
        if k == "cmp": return self._cmp(e[1], self.eval(e[2]), self.eval(e[3]))
        if k == "bin":
            a, b = self._num(self.eval(e[2])), self._num(self.eval(e[3]))
            return {"+":a+b,"-":a-b,"*":a*b,"/":a/b if b else float('inf')}[e[1]]
        if k == "call": return self._call(e[1], [self.eval(a) for a in e[2]])
        raise RuntimeError(f"bad expr {e}")

    def _call(self, name, args):
        N = name.upper()
        if N == "POINT":
            return {"x": self._num(args[0]), "y": self._num(args[1])}
        if   N == "GO_TO":    a = [self._num(args[0]), self._num(args[1])];                       fn = self.robot.go_to
        elif N == "PUSH_TO":  a = [self._num(args[0]), self._num(args[1])];                       fn = self.robot.push_to
        elif N == "FIND":     a = [str(args[0])];                                                 fn = self.robot.find
        elif N == "WAIT":     a = [self._num(args[0])];                                           fn = self.robot.wait
        elif N == "SCAN_FOR": a = [str(args[0]), self._num(args[1]) if len(args) > 1 else 360.0]; fn = self.robot.scan_for
        elif N == "PUSH_AWAY":a = [self._num(args[0]), self._num(args[1]), self._num(args[2]) if len(args) > 2 else 0.40]; fn = self.robot.push_away
        elif N == "PUSH_THROUGH": a = [self._num(args[0]) if len(args) > 0 else 0.70];                fn = self.robot.push_through
        elif N == "PUSH_TO_WALL": a = [str(args[0]) if args else "ball"];                              fn = self.robot.push_to_wall
        elif N == "PUSH_TO_GOAL": a = [self._num(args[0]), self._num(args[1])] if len(args) >= 2 else []; fn = self.robot.push_to_goal
        # FOLLOW with no arguments is the normal case: the program defers target,
        # standoff and predictor to what the spoken phrase resolved to, so one
        # follow.toy covers every target. Explicit arguments override that, so a
        # hand-written .toy can still pin a value. FOLLOW(name, m, TRUE) replaces
        # the old FOLLOW_FAST primitive.
        elif N == "FOLLOW":       a = ([str(args[0])] if len(args) > 0 else []) \
                                    + ([self._num(args[1])] if len(args) > 1 else []) \
                                    + ([self._truthy(args[2])] if len(args) > 2 else []); fn = self.robot.follow
        elif N == "EXPLORE":      a = [self._num(args[0]) if args else 900];                            fn = self.robot.explore_open
        else:
            raise NameError(f"unknown primitive '{name}'")
        label = N + "(" + ", ".join(("%.2f" % x if isinstance(x, (int, float)) else str(x)) for x in a) + ")"
        if self.on_call: self.on_call(label, None, False)
        res = fn(*a)
        if self.on_call: self.on_call(label, res, True)
        return res

    # helpers
    def _cmp(self, op, a, b):
        if op == "==": return a == b
        if op == "!=": return a != b
        a, b = self._num(a), self._num(b)
        return {"<":a<b,">":a>b,"<=":a<=b,">=":a>=b}[op]
    def _truthy(self, v):
        if v is None or v is False: return False
        if v is True: return True
        if isinstance(v, (int, float)): return v != 0
        return True
    def _num(self, v):
        if isinstance(v, bool) or not isinstance(v, (int, float)):
            raise TypeError(f"expected a number, got {v!r}")
        return v
    def _fmt(self, v):
        if v is None: return "NONE"
        if isinstance(v, dict): return f"({v.get('x')}, {v.get('y')})"
        return str(v)


# ── Mock robot for offline testing ──────────────────────────────────────────────
class MockRobot:
    """Simulates the world: robot pose + a couple of toys it can 'see' and push."""
    def __init__(self, log=print):
        self.log = log; self.x = 0.0; self.y = 0.0; self.yaw = 0.0
        self.toys = {"teddy_bear": {"x": 1.5, "y": 0.8},
                     "ball": {"x": 1.2, "y": 0.3}}          # name -> world pos
        self.ball_goal = None    # the human-set goal PUSH_TO_GOAL drives the ball to
    def go_to(self, x, y):
        self.log(f"  [robot] GO_TO ({x:.2f},{y:.2f})"); self.x, self.y = x, y; return "arrived"
    def push_to(self, x, y):
        # push the nearest toy a step toward (x,y)
        self.log(f"  [robot] PUSH_TO ({x:.2f},{y:.2f})")
        for t in self.toys.values():
            d = math.hypot(x-t["x"], y-t["y"])
            if d > 0.4:
                t["x"] += (x-t["x"])/d*0.5; t["y"] += (y-t["y"])/d*0.5
                self.x, self.y = t["x"], t["y"]
        return "pushed"
    def wait(self, secs):
        self.log(f"  [robot] WAIT {secs:.1f}s"); return "done"
    def scan_for(self, name, max_deg=360.0):
        self.log(f"  [robot] SCAN_FOR {name!r}"); return self.find(name)
    def push_away(self, bx, by, step=0.40):
        self.log(f"  [robot] PUSH_AWAY ball=({bx:.2f},{by:.2f}) step={step:.2f}"); return "pushed"
    def push_through(self, dist=0.70):
        self.log(f"  [robot] PUSH_THROUGH {dist:.2f}m"); return "pinned"
    def push_to_wall(self, name="ball", max_time=70):
        self.log(f"  [robot] PUSH_TO_WALL {name}"); return "pinned"
    def push_to_goal(self, gx=None, gy=None, name="ball", max_time=240.0, max_rounds=14):
        """Simulate the real loop: line up BEHIND the ball on the goal<->ball line,
        shove it toward the goal, re-measure, repeat until it arrives."""
        goal = (gx, gy) if (gx is not None and gy is not None) else self.ball_goal
        if goal is None:
            self.log("  [robot] PUSH_TO_GOAL: no goal marked"); return "no-goal"
        t = self.toys.get(name)
        if t is None:
            self.log(f"  [robot] PUSH_TO_GOAL: no {name} in view"); return "lost"
        for k in range(int(max_rounds)):
            d = math.hypot(goal[0]-t["x"], goal[1]-t["y"])
            if d <= 0.30:
                self.log(f"  [robot] PUSH_TO_GOAL: ball at goal (d={d:.2f}m)"); return "at goal"
            ux, uy = (goal[0]-t["x"])/d, (goal[1]-t["y"])/d
            sx, sy = t["x"] - ux*0.55, t["y"] - uy*0.55      # stance behind the ball
            self.log(f"  [robot] round {k+1}: stance ({sx:.2f},{sy:.2f}) -> shove "
                     f"({d:.2f}m to goal)")
            self.x, self.y = sx, sy
            step = min(d - 0.10, 0.6)                        # the ball rolls on
            t["x"] += ux*step; t["y"] += uy*step
            self.x, self.y = t["x"] - ux*0.55, t["y"] - uy*0.55
        self.log("  [robot] PUSH_TO_GOAL: timed out"); return "timeout"
    def follow(self, name=None, standoff=None, fast=None, max_time=900):
        """Mirror NavRobot.follow's resolution with no ROS state to read: no
        argument means the everyday default (a person at 1 m), a small target
        means a closer standoff."""
        name = name or "person"
        if standoff is None:
            standoff = 0.7 if "ball" in name.lower() else 1.0
        fast = bool(fast)
        self.log(f"  [robot] FOLLOW {name} standoff={standoff:.1f}m"
                 + (" (predict + lidar-track)" if fast else ""))
        return "done"
    def explore_open(self, max_time=900, reach=2.0):
        self.log(f"  [robot] EXPLORE max_time={max_time}"); return "done"
    def find(self, name):
        t = self.toys.get(name)
        self.log(f"  [robot] FIND {name!r} -> {t}")
        return dict(t) if t else None


if __name__ == "__main__":
    if len(sys.argv) > 1:
        with open(sys.argv[1]) as fh:
            src = fh.read()
    else:
        src = sys.stdin.read()
    interp = Interpreter(MockRobot())
    try:
        interp.run(src)
        print("\n[program finished] vars:", {k: interp._fmt(v) for k, v in interp.env.items()})
    except StopProgram:
        print("\n[program stopped]")
