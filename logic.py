#!/usr/bin/env python3
"""logic - a Bus Pirate `logic` style logic analyzer UX for sigrok-pico.

Live scrolling waveforms in the terminal, driven straight from the
raspberrypi-pico firmware protocol over USB CDC.  No PulseView, no sigrok,
no third-party python modules - just stdlib and a serial port.

  logic                       live view, auto sample rate, Ctrl-C to stop
  logic -i                    board / clock / pin-map / envelope info
  logic -c GP0,GP1,GP3,GP4    pins to watch (GPn, Dn or bare index; gaps ok)
  logic -f 10e6               fix the sample rate in Hz (default: auto)
  logic -n 0.5                target window per shot, seconds
  logic nav                   grab one long shot, then pan with the arrows
  logic --once                print one static graph and exit (scriptable)
  logic --shots 10            N shots then exit
  logic -T GP1=f              hold the view until GP1 falls (freeze on edge)
  logic -0 . -1 '#'           low / high glyphs

The firmware wire format is decoded natively: the optimized 4-channel RLE
stream (<=4 channels, all inside GP0..GP3, no analog) and the general
7-bits-per-byte slice stream with run-length suppression.  Rate and depth are
kept inside what the hardware was measured to survive, so the tool cannot
strand the board half-streaming (which is how a Pico gets "wedged").
"""

import os
import re
import sys
import time
import termios
import tty
import select
import argparse

# ---------------------------------------------------------------- limits ---
# Measured on RP2350 @150MHz with the 2025-refactor firmware:
#   <=4 channels (D4 stream): single shot fits in RAM to ~950k samples,
#                             real-time streaming sustains to ~30Msps.
#   >=5 channels (slice)    : single shot fits in RAM to ~238k samples,
#                             real-time streaming sustains to ~8Msps.
# Stay a little inside those numbers - exceeding them makes the firmware
# answer '!' and (if a capture is then killed) the USB stack can lock up.
INBUF = dict(D4=900_000, SLICE=200_000)
STREAM = dict(D4=30_000_000, SLICE=8_000_000)
MIN_RATE = 5_000
AUTO_START = 1_000_000
SPR = 10            # samples per signal cycle the auto rate aims for
AUTO_CYCLES = 12    # cycles of the fastest active pin across the screen
MINSAMP = 256       # depth floor - below this the buckets stop meaning much
QUIET_SAMPLES = 8000        # cheap depth used while hunting for slow signals


def err(*a):
    print(*a, file=sys.stderr)


def hms(v):
    for suf, div in (("G", 1e9), ("M", 1e6), ("k", 1e3)):
        if v >= div:
            return "%.4g%s" % (v / div, suf)
    return "%.3g" % v


def dur(v):
    if v < 1e-6:
        return "%.0f ns" % (v * 1e9)
    if v < 1e-3:
        return "%.2f us" % (v * 1e6)
    if v < 1:
        return "%.2f ms" % (v * 1e3)
    return "%.2f s" % v


# ------------------------------------------------------------ the device ---
class Pico:
    def __init__(self, dev):
        self.dev = dev
        self.mask_ok = False        # do we know the firmware's channel mask?
        self.fd = os.open(dev, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        a = termios.tcgetattr(self.fd)
        a[0] = a[1] = a[3] = 0
        a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL
        a[4] = a[5] = termios.B115200
        a[6][termios.VMIN] = 0
        a[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, a)
        self.names = {}

    # -- byte level ------------------------------------------------------
    def read(self, t=0.05):
        if not select.select([self.fd], [], [], t)[0]:
            return b""
        try:
            return os.read(self.fd, 65536)
        except OSError:
            return b""

    def drain(self, t=0.3):
        n = 0
        while True:
            d = self.read(t)
            if not d:
                break
            n += len(d)
        return n

    # -- command level ---------------------------------------------------
    def cmd(self, s, t=1.5):
        """Send a command line, wait for the '*' ack."""
        os.write(self.fd, (s + "\n").encode())
        end, got = time.time() + t, ""
        while time.time() < end:
            got += self.read(0.02).decode("latin1")
            if "*" in got:
                return got
        raise IOError("no ack for %r (got %r)" % (s, got[:40]))

    def ask(self, s, t=0.6, gap=0.10):
        """Send a query, read the reply (queries answer without '*')."""
        self.drain(0.05)
        os.write(self.fd, (s + "\n").encode())
        end, got = time.time() + t, ""
        while time.time() < end:
            d = self.read(0.05)
            if d:
                got += d.decode("latin1")
                end = min(end, time.time() + gap)
            elif got:
                break
        return got

    def settle(self, maxt=2.5, quiet=0.25):
        """Abort a running stream and wait for the link to go quiet.

        CDC bytes from an abandoned shot keep arriving; reading identify()
        through them is what makes the board look unresponsive.
        """
        try:
            os.write(self.fd, b"+")
            time.sleep(0.03)
            os.write(self.fd, b"*")
        except OSError:
            pass
        t0 = last = time.time()
        while time.time() - last < quiet and time.time() - t0 < maxt:
            if self.read(0.05):
                last = time.time()
        self.drain(0.05)

    # -- facts -----------------------------------------------------------
    n_digital = 26

    def identify(self):
        self.settle()
        self.mask_ok = False        # somebody else may own the board's state
        m = re.search(r"SRPICO,A(\d\d)(.)D(\d\d),(\d\d)", self.ask("i"))
        if not m:
            raise IOError("bad identify reply")
        self.n_digital = int(m.group(3))
        return dict(n_analog=int(m.group(1)), a_size=int(m.group(2)),
                    n_digital=int(m.group(3)), proto=int(m.group(4)))

    def clock(self):
        """Local firmware extension: 'k' -> CLK<khz>x<maxHz>x<cyc/sample>."""
        m = re.search(r"CLK(\d+)x(\d+)x(\d+)", self.ask("k"))
        if not m:
            return None
        return dict(clk_sys=int(m.group(1)) * 1000, max_rate=int(m.group(2)),
                    cyc=int(m.group(3)))

    def pin(self, idx):
        """Channel index -> pin name, taken from the device ('nD<n')."""
        if idx not in self.names:
            m = re.search(r"GP(\d+)", self.ask("nD%d" % idx, gap=0.04))
            self.names[idx] = "GP" + m.group(1) if m else "D%d" % (idx + 2)
        return self.names[idx]

    def pin_index(self, name, n_digital):
        for i in range(n_digital):
            if self.pin(i) == name:
                return i
        raise SystemExit("no channel on %s" % name)

    # -- acquisition -----------------------------------------------------
    @staticmethod
    def mode_of(chans, analog=False):
        mask = 0
        for c in chans:
            mask |= 1 << c
        return "D4" if (mask & ~0xF) == 0 and not analog else "SLICE"

    def capture(self, chans, rate, nsamp, analog=False, tmo=None):
        """One fixed-depth shot -> (runs, mode, end_reason).

        runs is [(value, count), ...]: decoded values with their run lengths,
        which is what makes long windows cheap to render.  Unlike the sigrok
        driver, a *gapped* channel set is fine - the PIO always samples from
        GP0 up, the gaps are simply not reported back to us.
        """
        chans = sorted(set(chans))
        if not chans:
            raise ValueError("no channels")
        mode = self.mode_of(chans, analog)
        if tmo is None:
            tmo = max(2.0, nsamp / float(rate) + 5.0)
        mask = 0
        for c in chans:
            mask |= 1 << c
        nib = mode == "D4"
        groups = [g for g in range(0, 32, 7) if (mask >> g) & 0x7F]
        bps = 1 if nib else len(groups)

        self.settle()
        # CAUTION: the firmware's '*' reset does not clear d_mask - only a cold
        # boot does.  Anything enabled by an earlier capture (ours or another
        # host's) is still sampling, which silently changes pin_count and the
        # wire format.  So state every channel explicitly, every shot.
        for i in range(self.n_digital):
            self.cmd("D%d%d" % (1 if i in chans else 0, i))
        self.cmd("L%d" % nsamp)
        self.cmd("R%d" % rate)
        self.drain(0.04)                      # pre-arm drain, as the driver does

        os.write(self.fd, b"F\n")             # fixed depth, untriggered
        runs, buf, rle, prev, seen, end = [], b"", 0, 0, 0, None
        have = False

        def emit(v, n):
            if n <= 0:
                return
            if runs and runs[-1][0] == v:
                runs[-1] = (v, runs[-1][1] + n)
            else:
                runs.append((v, n))

        t_last = time.time()
        while end is None:
            if time.time() - t_last > tmo:
                end = "timeout"
                break
            chunk = self.read(0.2)
            if not chunk:
                continue
            t_last = time.time()
            buf += chunk
            i = 0
            n = len(buf)
            while i < n:
                b = buf[i]
                if b == 0x24:                             # '$' end of stream
                    end, i = "done", i + 1
                    break
                if b == 0x21:                             # '!' device abort
                    end, i = "abort", i + 1
                    break
                if nib:
                    if 0x30 <= b <= 0x7F:                 # RLE-only, x8
                        rle += (b - 47) * 8
                        i += 1
                    elif b >= 0x80:                       # value + inline 0..7
                        rle += (b & 0x70) >> 4
                        if rle and have:
                            emit(prev, rle)
                        rle = 0
                        prev = b & 0x0F
                        have = True
                        emit(prev, 1)
                        seen += 1
                        i += 1
                    else:                                 # trailing byte count
                        i += 1
                else:
                    if b < 0x30:                          # control / count
                        i += 1
                    elif b < 0x80:                        # RLE of last slice
                        rle += (b - 47) if b <= 79 else (b - 78) * 32
                        i += 1
                    elif i + bps > n:
                        break                             # partial slice
                    else:
                        word, j = 0, i
                        for g in groups:
                            word |= (buf[j] & 0x7F) << g
                            j += 1
                        if rle and have:
                            emit(prev, rle)
                        rle = 0
                        prev = word
                        have = True
                        emit(word, 1)
                        seen += 1
                        i = j
                if seen > nsamp + 4096:
                    end = "overrun"
                    break
            if end is not None:
                break
            buf = buf[i:]
        if rle and have:
            emit(prev, rle)
        # A clean '$' end needs no long good-bye; an abort might still be
        # spilling bytes, so be patient in that case.
        if end == "done":
            try:
                os.write(self.fd, b"*")
            except OSError:
                pass
            self.drain(0.02)
            self.mask_ok = True          # nobody else has touched the board
        else:
            self.mask_ok = False
            self.settle(quiet=0.25)
        return runs, mode, end


# ------------------------------------------------------------ analysis -----
class Shot:
    """Decoded capture: runs plus everything derived from them."""

    def __init__(self, runs, rate, chans, mode, end=None):
        self.runs = runs
        self.rate = float(rate)
        self.chans = sorted(chans)
        self.mode = mode
        self.end = end
        self.n = sum(c for _, c in runs)
        # D4 packs the enabled pins into a nibble (bit k = GPk); the slice
        # stream returns a word whose bit position equals the channel index.
        self.pos = [k if mode == "D4" else c for k, c in enumerate(self.chans)]

    def channel_edges(self, k):
        """[(sample_index, new_bit), ...] for one channel."""
        p = self.pos[k]
        out, idx, prev = [], 0, None
        for v, c in self.runs:
            bit = (v >> p) & 1
            if prev is None or bit != prev:
                out.append((idx, bit))
                prev = bit
            idx += c
        return out

    def stats(self, k):
        p = self.pos[k]
        e = self.channel_edges(k)
        hi = 0
        idx = 0
        for v, c in self.runs:
            if (v >> p) & 1:
                hi += c
            idx += c
        st = dict(edges=max(0, len(e) - 1), duty=100.0 * hi / max(self.n, 1),
                  level=(e[0][1] if e else 0), f=None, period=None,
                  jitter=None, minp=None, clean=False, thin=False)
        if len(e) < 2:
            return st
        # a cycle is two consecutive edges; work in whole cycles so that a rate
        # where the high and low halves differ by a sample still reads right
        cyc = [e[i + 2][0] - e[i][0] for i in range(len(e) - 2)]
        gaps = [e[i + 1][0] - e[i][0] for i in range(len(e) - 1)]
        inner = gaps[1:-1] if len(gaps) > 4 else gaps   # first/last are window seams
        st["minp"] = min(inner) / self.rate
        st["favg"] = st["edges"] / 2.0 / self.n * self.rate
        if cyc:
            med = sorted(cyc)[len(cyc) // 2]
            mean = sum(cyc) / float(len(cyc))
            sd = (sum((c - mean) ** 2 for c in cyc) / len(cyc)) ** 0.5
            st["period"] = med
            st["f"] = self.rate / med if med else None
            st["jitter"] = sd / mean if mean else 1.0
            st["clean"] = st["jitter"] < 0.08 and len(cyc) > 3
            st["thin"] = med < 6            # less than ~3 samples per half cycle
        return st

    def verdict(self, st):
        if not st["edges"]:
            return "steady %s" % ("HIGH" if st["level"] else "LOW")
        f = st.get("f") or st.get("favg")
        txt = "square %s" % hms(f) + "Hz"
        if st["clean"]:
            txt += "  (+-%.1f%%)" % (st["jitter"] * 100)
        else:
            txt = "%d edges, ~%sHz avg" % (st["edges"], hms(f))
        txt += "  min %s" % dur(st["minp"]) if st["minp"] else ""
        if st["thin"]:
            txt += "  (raise -f: %d samples/cycle)" % st["period"]
        return txt

    def columns(self, cols, start=0, count=None):
        """Level/edge grid: `count` samples from `start` spread on `cols`."""
        lvl = [bytearray(len(self.chans)) for _ in range(cols)]
        edge = [bytearray(len(self.chans)) for _ in range(cols)]
        if not self.n:
            return lvl, edge
        count = self.n if count is None else max(1, min(count, self.n))
        start = max(0, min(start, self.n - count))
        span = count / float(cols)                  # samples per column
        stop = float(start + count)
        prev = None
        s0 = 0.0
        for v, c in self.runs:
            end = s0 + c
            if end > start and s0 < stop:
                c0 = min(int((max(s0, start) - start) / span), cols - 1)
                c1 = min(int((min(end, stop) - 1 - start) / span), cols - 1)
                for col in range(max(c0, 0), c1 + 1):
                    for k in range(len(self.chans)):
                        lvl[col][k] = (v >> self.pos[k]) & 1
                # an edge belongs to the column where the run starts, not to
                # every column that run then covers
                if prev is not None:
                    for k in range(len(self.chans)):
                        if ((v >> self.pos[k]) & 1) != ((prev >> self.pos[k]) & 1):
                            edge[c0][k] = min(255, edge[c0][k] + 1)
            prev = v
            s0 = end
        return lvl, edge


# ------------------------------------------------------------- rendering ---
def draw(pico, shot, cols=None, off=0, zoom=None, lo="_", hi="\u203e", eg="|",
         dense="\u2592", busy="\u2588", plain=False):
    if cols is None:
        try:
            cols = os.get_terminal_size().columns - 2
        except OSError:
            cols = 100
        cols = max(40, min(cols, 220))
    count = shot.n if zoom is None else zoom * cols
    lvl, edge = shot.columns(cols, off, count)
    lines = []
    head = (" logic  f=%s  win=%s  n=%d  %dch fmt=%s"
            % (hms(shot.rate) + "Hz", dur(count / shot.rate), shot.n,
               len(shot.chans), shot.mode))
    if zoom is not None:
        head += "  %sx  @%s" % (hms(zoom), dur(off / shot.rate))
    lines.append(head)
    for k, c in enumerate(shot.chans):
        row = []
        for col in range(cols):
            n = edge[col][k]
            if n == 0:
                row.append(hi if lvl[col][k] else lo)
            elif n == 1:
                row.append(eg)
            else:
                row.append(dense if n < 6 else busy)
        st = shot.stats(k)
        lines.append(" %-5s %s  %5.1f%%  %s" %
                     (pico.pin(c), "".join(row), st["duty"], shot.verdict(st)))
    if plain:
        lines.append(" %d samples @ %s = %s window, %d columns, stream %s"
                     % (shot.n, hms(shot.rate) + "Hz",
                        dur(shot.n / shot.rate), cols, shot.end))
    txt = "\n".join(lines)
    return txt


# ------------------------------------------------------------------ main ---
def parse_pins(spec, pico, n_digital):
    if not spec:
        return list(range(min(4, n_digital)))
    want = []
    for tok in re.split(r"[,\s]+", spec.strip()):
        if not tok:
            continue
        t = tok.upper()
        if t.startswith("GP"):
            want.append(pico.pin_index("GP" + t[2:], n_digital))
        elif t.startswith("D"):
            want.append(int(t[1:]) - 2)
        else:
            want.append(int(t))
    return sorted(set(want))


def rate_cap(clk):
    """Highest rate we ever ask for.

    Every shot is kept inside the on-chip capture buffer (see INBUF), and while
    the sample sits in RAM the PIO never waits for USB - so the only real
    ceiling is clk_sys.  Shots deeper than the buffer would need the firmware
    to encode and stream in real time, which is exactly where it starts
    answering '!'; that regime is avoided by construction.
    """
    return clk["max_rate"] if clk else 120_000_000


def grab(pico, chans, rate, nsamp):
    runs, mode, end = pico.capture(chans, rate, nsamp)
    shot = Shot(runs, rate, chans, mode, end)
    if not shot.n:
        err("  nothing came back (stream %s)" % end)
    elif end not in ("done", None):
        err("  stream %s after %d samples - lower -f or -n" % (end, shot.n))
    return shot


def main():
    ap = argparse.ArgumentParser(
        prog="logic", description=__doc__.split("\n\n")[1],
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-d", default="/dev/ttyACM0", help="serial device")
    ap.add_argument("-i", dest="info", action="store_true",
                    help="show board info and exit")
    ap.add_argument("-f", type=float, default=None, help="sample frequency Hz")
    ap.add_argument("-o", dest="oversample", type=int, default=1,
                    help="oversample: multiply -f")
    ap.add_argument("-c", default=None, help="pins, e.g. GP0,GP1,GP3")
    ap.add_argument("-n", dest="window", type=float, default=None,
                    help="window per shot in seconds (default: auto)")
    ap.add_argument("--cycles", type=int, default=AUTO_CYCLES,
                    help="cycles to fit on screen in auto mode")
    ap.add_argument("-T", default=None, help="freeze until GPn=r|f|1|0")
    ap.add_argument("-0", dest="lo", default="_", help="glyph for low")
    ap.add_argument("-1", dest="hi", default="\u203e", help="glyph for high")
    ap.add_argument("-2", dest="dense", default="\u2592",
                    help="glyph for a column with several edges")
    ap.add_argument("-3", dest="busy", default="\u2588",
                    help="glyph for a column that is mostly edges")
    ap.add_argument("--cols", type=int, default=None)
    ap.add_argument("--once", action="store_true", help="one graph, then exit")
    ap.add_argument("--shots", type=int, default=0, help="N shots, then exit")
    ap.add_argument("verb", nargs="?", default="start",
                    choices=["start", "stop", "nav", "hide", "show"])
    a = ap.parse_args()

    if a.verb == "stop":
        try:
            Pico(a.d).settle()
            print("device stream aborted")
        except OSError as e:
            err("%s: %s" % (a.d, e))
            return 1
        return 0

    try:
        pico = Pico(a.d)
        idn = pico.identify()
    except (OSError, IOError) as e:
        err("%s: %s" % (a.d, e))
        err("  BOOTSEL mode? held by another process? needs a replug?")
        return 2
    clk = pico.clock()

    if a.info:
        print("sigrok-pico  %s" % a.d)
        print("  channels : %d digital, %d analog (%dB each), protocol v%d"
              % (idn["n_digital"], idn["n_analog"], idn["a_size"], idn["proto"]))
        if clk:
            print("  clock    : clk_sys %sHz, %d cycle/sample -> ceiling %sHz"
                  % (hms(clk["clk_sys"]), clk["cyc"], hms(clk["max_rate"])))
            print("             exact divisors: %s"
                  % "  ".join(hms(clk["clk_sys"] // d)
                              for d in (1, 2, 3, 4, 5, 6, 10, 15, 20, 30)))
        for m, lim in (("D4", INBUF["D4"]), ("SLICE", INBUF["SLICE"])):
            print("  %-8s: deepest single shot %s samples "
                  "(deeper needs real-time streaming: <= %sHz)"
                  % (m, hms(lim), hms(STREAM[m])))
        print("  pin map  : " + " ".join("%s=D%d" % (pico.pin(i), i + 2)
                                        for i in range(min(8, idn["n_digital"]))))
        return 0

    if a.verb in ("hide", "show"):
        print("%s: this view draws a graph per capture; use `logic stop` to "
              "abort a running stream" % a.verb)
        return 0

    chans = parse_pins(a.c, pico, idn["n_digital"])
    for c in chans:
        if c < 0 or c >= idn["n_digital"]:
            raise SystemExit("channel %d out of range" % c)
    mode = Pico.mode_of(chans)

    trig = None
    if a.T:
        nm, _, ed = a.T.partition("=")
        trig = (parse_pins(nm, pico, idn["n_digital"])[0], (ed or "r")[0])

    rate = int(a.f * a.oversample) if a.f else AUTO_START
    rate = max(MIN_RATE, rate)
    win = a.window if a.window else 0.2        # auto starts curious, then widens
    nshot = a.shots
    tty = sys.stdout.isatty() and not a.once
    frozen = None
    first = True
    try:
        while True:
            cap = rate_cap(clk)
            rate = int(max(MIN_RATE, min(rate, cap)))
            nsamp = int(min(max(rate * win, MINSAMP), INBUF[mode]))
            shot = grab(pico, chans, rate, nsamp)
            if not shot.n:
                return 1
            if trig:
                tk, te = trig
                k = chans.index(tk) if tk in chans else None
                e = shot.channel_edges(k) if k is not None else []
                hit = any((te == "r" and b == 1) or (te == "f" and b == 0) or
                          (te == "1" and b == 1) or (te == "0" and b == 0)
                          for _, b in e[1:])
                if not hit:
                    if not tty:
                        err("no %s edge in %d samples @ %sHz"
                            % (te, shot.n, hms(rate)))
                        return 1
                    sys.stdout.write("  waiting for -T %s  (%s @ %sHz)   \r"
                                     % (a.T, dur(shot.n / rate), hms(rate)))
                    sys.stdout.flush()
                    continue

            # ---- auto capture speed: put ~`--cycles` cycles across the
            # screen at ~SPR samples per cycle, and go hunting with a wider
            # window whenever nothing is moving at all.
            if a.f is None or a.window is None:
                act = [shot.stats(k) for k in range(len(chans))]
                fast = None
                for x in act:
                    if not x["edges"]:
                        continue
                    fx = x["f"] or x.get("favg")
                    if fx and (fast is None or fx > fast):
                        fast = fx
                if fast:
                    if a.window is None:
                        win = min(60.0, max(1e-4, a.cycles / float(fast)))
                    if a.f is None:
                        r = SPR * fast                    # resolution
                        r = max(r, MINSAMP / win)         # depth for buckets
                        r = min(r, cap, INBUF[mode] / win)  # fits in RAM
                        rate = int(max(MIN_RATE, r))
                elif a.window is None and not a.f:
                    win = min(60.0, win * 3.0)
                    rate = int(max(MIN_RATE, min(cap, QUIET_SAMPLES / win)))

            if a.verb == "nav" and frozen is None:
                frozen = shot
                off = 0
                out = draw(pico, frozen, cols=a.cols, lo=a.lo, hi=a.hi,
                           dense=a.dense, busy=a.busy, plain=True)
            else:
                out = draw(pico, shot, cols=a.cols, lo=a.lo, hi=a.hi,
                           dense=a.dense, busy=a.busy, plain=not tty)
            if tty:
                os.write(1, b"\033[H\033[J" if first else b"\033[H")
                first = False
            print(out, flush=True)
            if a.once:
                return 0
            if nshot:
                nshot -= 1
                if not nshot:
                    return 0
            if a.verb == "nav":
                if nav_keys(pico, frozen, chans, rate, a):
                    frozen = None
                else:
                    return 0
    except KeyboardInterrupt:
        print()
        return 0
    finally:
        try:
            pico.settle(1.0)
        except Exception:
            pass


def nav_keys(pico, shot, chans, rate, a):
    """Arrow-key panning/zoom for `logic nav`.  True = recapture."""
    fd = sys.stdin.fileno()
    raw = sys.stdin.isatty()
    old = termios.tcgetattr(fd) if raw else None
    zoom = None
    try:
        if raw:
            tty.setraw(fd)
        cols = a.cols or max(40, min(os.get_terminal_size().columns - 2, 220))
        off = 0
        err("  left/right pan, up zoom in, down zoom out, space recapture, x exit")
        while True:
            try:
                c = os.read(fd, 1)
            except OSError:
                return False
            if c in (b"q", b"x", b"\x03", b""):
                return False
            if c == b"\x1b":
                # Escape sequences can arrive one byte at a time over a serial
                # or pty link, so read the pieces instead of assuming a burst.
                c2 = os.read(fd, 1)
                if not c2:
                    return False
                c3 = b""
                # the final byte can lag behind by a whole character time on a
                # slow link, so give it a few hundred ms before giving up
                if select.select([fd], [], [], 0.3)[0]:
                    c3 = os.read(fd, 1)
                key = (c2 + c3)[-1:]
                z = zoom or max(1, shot.n // cols)
                step = max(1, z // 8)
                if key == b"D":
                    off = max(0, off - step)
                elif key == b"C":
                    off = min(max(0, shot.n - z), off + step)
                elif key == b"A":              # zoom in: fewer samples/column
                    zoom = max(2, z // 2)
                elif key == b"B":              # zoom out
                    zoom = min(shot.n, z * 2)
                elif key in (b"H", b""):       # bare Esc / Home = go to start
                    off = 0
                elif key == b"F":
                    off = max(0, shot.n - z)
            elif c == b" ":
                return True
            os.write(1, b"\033[H\033[J")
            print(draw(pico, shot, cols, off, zoom, a.lo, a.hi,
                       dense=a.dense, busy=a.busy))
    finally:
        if old is not None:
            termios.tcsetattr(fd, termios.TCSANOW, old)


if __name__ == "__main__":
    sys.exit(main())
