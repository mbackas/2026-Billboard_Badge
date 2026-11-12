"""Unofficial calm partial refresh for the badge's B1 e-paper panel: about 0.6 s a frame.

Follows the findings of badge os (github.com/yigitabi5444/supercon26-badge-os).
Use it from an asyncio task, without gc.collect() between frames (it takes ~0.35 s):
    panel = PartialEPD()
    await panel.start()        # waits for the stock driver, then cleans to white
    panel.clear()              # fast; then draw into panel.fb: 960 x 672, 1 = black
    await panel.show()         # sends what changed, one ~0.6 s pull
    await panel.stop()         # always: powers the panel down and frees the bus
"""

import array
import asyncio as aio
import framebuf
import time

import micropython
from micropython import const

from lib import epd as _epd

DRIVE_FRAMES = 10    # drive frames per change, of the 20 a pull runs (1-19)
SESSION_PULLS = 60   # pulls per session; badge os proved 60
TEMPERATURE = 0x84   # 0x50 + 2 * degrees C: 26 C, what both community drivers send

# Waits in the session-open sequence, in ms: reset high/low/high, after 0x05 7D,
# after 0x05 00, load strobe high, load strobe low, and after the last strobe
# (0xB0 must come after it). QUICK is Pervasive Displays' sequence for its
# newer large panels; badge os found it works on this one.
# SAFE is what PD documents for this panel, used if a quick session ever fails
# to take the waveform table (then a pull plays the panel's own ~33 s one).
_QUICK = (2, 4, 20, 50, 1, 2, 10, 10)
_SAFE = (20, 200, 200, 200, 10, 100, 100, 500)
_SLOW_PULL_MS = 2_500     # longer than this: the table didn't take
_PULL_TIMEOUT_MS = 60_000 # never send anything mid-refresh, even a 33 s one

_W = const(960)
_H = const(672)
_HALF = const(60)          # bytes per row that each driver chip gets
_PLANE = const(80640)      # bytes per full plane
_CHIP_PLANE = const(40320) # bytes per plane that each driver chip gets

_LEFT = const(1)    # left chip, columns 0-479 (CS1, the master)
_RIGHT = const(2)   # right chip, columns 480-959 (CS2)
_BOTH = const(3)

_DUW_ALL = b"\x00\x3b\x00\x00\x9f\x02"   # RAM window: 60 bytes x 672 rows
_DRFW_ALL = b"\x00\x3b\x00\xa9"          # refresh window: whole panel. Never narrower:
                                         # a shorter y range copies rows down the panel

# The B1 panel takes every byte with its bits in reverse order.
_REV = bytearray(256)
for _i in range(256):
    _b = (_i & 0xF0) >> 4 | (_i & 0x0F) << 4
    _b = (_b & 0xCC) >> 2 | (_b & 0x33) << 2
    _REV[_i] = (_b & 0xAA) >> 1 | (_b & 0x55) << 1


def partial_table(drive: int = DRIVE_FRAMES) -> bytearray:
    """The 400-byte transition table for register 0xB0."""
    # 5 tables (old << 1 | new, then VCOM) x 10 groups x 8 bytes: level, TPa, TPb, SRab, TPc,
    # TPd, SRcd, RP. A level byte has 2 bits per phase, a in bits 7-6: 0 ground, 1 VDH, 2 VDL.
    # SRab = 0 skips phases a and b, so a group is one frame of c and one of d: 20 frames, the
    # fewest the chips accept. No frame count is 0: that hangs the chips until a reset.
    # Tables 1 and 2 are equal and opposite, so every change is undone by its reverse.
    if not 1 <= drive <= 19:
        raise ValueError("drive must be 1-19: the last frame has to be ground")
    t = bytearray(400)
    for k in range(50):
        t[8 * k:8 * k + 8] = b"\x00\x01\x01\x00\x01\x01\x01\x01"
    for f in range(drive):
        shift = 2 if f % 2 == 0 else 0      # phase c, then phase d
        t[(10 + f // 2) * 8] |= 1 << shift  # table 1, white -> black: VDH
        t[(20 + f // 2) * 8] |= 2 << shift  # table 2, black -> white: VDL
    return t


@micropython.viper
def _diff(a, b, off: int, box) -> int:
    """Byte box where planes a and b differ in one chip's half (bytes off to
    off+59 of each row). Writes box = [x0, x1, y0, y1] (x in that half's bytes,
    y in rows) and returns 1, or returns 0 if that half is the same."""
    wa = ptr32(a)
    wb = ptr32(b)
    ba = ptr8(a)
    bb = ptr8(b)
    out = ptr16(box)
    x0 = 60
    x1 = -1
    y0 = -1
    y1 = -1
    w0 = off >> 2   # a row is 30 words; a half is 15 of them
    for y in range(672):
        base = y * 30 + w0
        hit = 0
        for w in range(15):
            if wa[base + w] != wb[base + w]:
                hit = 1
                i = (base + w) << 2
                for k in range(4):
                    if ba[i + k] != bb[i + k]:
                        x = (w << 2) + k
                        if x < x0:
                            x0 = x
                        if x > x1:
                            x1 = x
        if hit:
            if y0 < 0:
                y0 = y
            y1 = y
    if x1 < 0:
        return 0
    out[0] = x0
    out[1] = x1
    out[2] = y0
    out[3] = y1
    return 1


@micropython.viper
def _gather(dst, src, box, rev) -> int:
    """Box [x0, x1, y0, y1, off] of one chip's half of plane src into dst: the
    rows in screen order, every byte bit-reversed. Returns the byte count."""
    d = ptr8(dst)
    s = ptr8(src)
    b = ptr16(box)
    t = ptr8(rev)
    x0 = int(b[0])
    x1 = int(b[1])
    y = int(b[2])
    y1 = int(b[3])
    off = int(b[4])
    n = 0
    while y <= y1:
        i = y * 120 + off + x0
        end = y * 120 + off + x1
        while i <= end:
            d[n] = t[s[i]]
            n += 1
            i += 1
        y += 1
    return n


@micropython.viper
def _fill(dst, value: int, n: int):
    d = ptr8(dst)
    for i in range(n):
        d[i] = value


@micropython.viper
def _zero(dst, words: int):
    """Clear a buffer a word at a time (framebuf's fill goes pixel by pixel)."""
    d = ptr32(dst)
    for i in range(words):
        d[i] = 0


@micropython.viper
def _any(src, n: int) -> int:
    s = ptr8(src)
    for i in range(n):
        if s[i]:
            return 1
    return 0


class PanelError(Exception):
    pass


class PartialEPD:
    width = _W
    height = _H

    def __init__(self) -> None:
        self._dc = _epd.EPD_DC_PIN
        self._rst = _epd.EPD_RST_PIN
        self._cs1 = _epd.EPD_CS1_PIN
        self._cs2 = _epd.EPD_CS2_PIN
        self._busy = _epd.EPD_BUSY1_PIN   # the right chip's BUSY2 stays low in a session
        self._spi = None
        self._locked = False
        self._awake = False      # chips out of reset: they need the soft shutdown
        self._partial = False    # a partial session is open
        self.pulls = 0           # pulls in the open partial session
        self.buf = None          # the bytes behind fb
        self.fb = None           # draw here: 960 x 672 MONO_HLSB, 1 = black
        self.last_send_ms = 0    # last show(): from the call to the refresh start
        self.last_pull_ms = 0    # last show(): the refresh itself
        self._shown = None       # what the film shows
        self._stage = None       # one chip's bytes on their way out
        self._waits = _QUICK
        self._lut = partial_table()
        self._temp = bytes((TEMPERATURE,))
        self._idx = bytearray(1)
        self._two = bytearray(2)
        self._duw = bytearray(6)
        self._ptr = bytearray(3)
        self._now = (array.array("H", (0, 0, 0, 0)), array.array("H", (0, 0, 0, 0)))
        self._prev = (array.array("H", (0, 0, 0, 0)), array.array("H", (0, 0, 0, 0)))
        self._prev_hit = [0, 0]
        self._win = array.array("H", (0, 0, 0, 0, 0))

    # ---- bus ---------------------------------------------------------------

    def _cmd(self, index, data, cogs=_BOTH):
        """Index byte with DC low, then the data with DC high, to the chips in `cogs`."""
        dc, cs1, cs2, spi = self._dc, self._cs1, self._cs2, self._spi
        left = 0 if cogs & _LEFT else 1
        right = 0 if cogs & _RIGHT else 1
        self._idx[0] = index
        dc(0)
        cs1(left)
        cs2(right)
        spi.write(self._idx)
        cs1(1)
        cs2(1)
        dc(1)
        cs1(left)
        cs2(right)
        spi.write(data)
        cs1(1)
        cs2(1)

    async def _ready(self, what, timeout_ms):
        """Wait for BUSY to go high. Returns the ms waited."""
        busy = self._busy
        t0 = time.ticks_ms()
        while not busy():
            if time.ticks_diff(time.ticks_ms(), t0) > timeout_ms:
                raise PanelError(f"panel stuck busy: {what}")
            await aio.sleep_ms(1)
        return time.ticks_diff(time.ticks_ms(), t0)

    # ---- session -----------------------------------------------------------

    async def _reset(self):
        rst, w = self._rst, self._waits
        rst(1)
        await aio.sleep_ms(w[0])
        rst(0)
        await aio.sleep_ms(w[1])
        rst(1)
        await aio.sleep_ms(w[2])
        self._cs1(1)
        self._cs2(1)
        await aio.sleep_ms(5)
        self._awake = True
        self._cmd(0x70, b"\x71\x36")   # B1 booster setting: without it a refresh draws ~2 A

    async def _start(self, lut):
        """Register sequence, the waveform table into 0xB0, DC/DC up."""
        cmd, w = self._cmd, self._waits
        cmd(0x05, b"\x7d")
        await aio.sleep_ms(w[3])
        cmd(0x05, b"\x00")
        await aio.sleep_ms(w[4])
        cmd(0xD8, b"\x80")                # master/slave sync
        cmd(0xD6, b"\x00")
        cmd(0xA7, b"\x10")
        await aio.sleep_ms(w[5])
        cmd(0xA7, b"\x00")
        await aio.sleep_ms(w[6])
        cmd(0x03, b"\x00\x11")            # oscillator, as the stock driver sends it
        for cog in (_LEFT, _RIGHT):
            cmd(0x44, b"\x00", cog)
            cmd(0x45, b"\x80", cog)
            cmd(0xA7, b"\x10", cog)
            await aio.sleep_ms(w[5])
            cmd(0xA7, b"\x00", cog)
            await aio.sleep_ms(w[6])
            cmd(0x44, b"\x06", cog)
            cmd(0x45, self._temp, cog)
            cmd(0xA7, b"\x10", cog)
            await aio.sleep_ms(w[5])
            cmd(0xA7, b"\x00", cog)       # loads the panel's own waveform...
            await aio.sleep_ms(w[6])
        await aio.sleep_ms(w[7] - w[6])
        cmd(0x60, b"\x25")                # TCON
        cmd(0x61, b"\x01", _LEFT)         # gate scan direction, master only
        cmd(0x02, b"\x00")
        cmd(0xB0, lut)                    # ...which this replaces: once per session
        await self._dcdc_on()

    async def _dcdc_on(self):
        """DC/DC soft start, the stock driver's steps."""
        cmd, v2 = self._cmd, self._two
        cmd(0x51, b"\x50\x01")
        for v in range(1, 5):
            cmd(0x09, b"\x1f")
            v2[0], v2[1] = 0x50, v
            cmd(0x51, v2)
            cmd(0x09, b"\x9f")
            await aio.sleep_ms(2)
        for v in range(1, 11):
            cmd(0x09, b"\x1f")
            v2[0], v2[1] = 0x0A, v
            cmd(0x51, v2)
            cmd(0x09, b"\x9f")
            await aio.sleep_ms(2)
        for v in range(3, 11):
            cmd(0x09, b"\x7f")
            v2[0], v2[1] = 0x0A, v
            cmd(0x51, v2)
            cmd(0x09, b"\xff")
            await aio.sleep_ms(2)
        for v in range(9, 1, -1):
            cmd(0x09, b"\x7f")
            v2[0], v2[1] = v, 0x0A
            cmd(0x51, v2)
            cmd(0x09, b"\xff")
            await aio.sleep_ms(2)
        cmd(0x09, b"\xff")
        await aio.sleep_ms(100)
        await self._ready("DC/DC start", 10_000)   # never trigger into a ramping booster

    async def _shutdown(self):
        """DC/DC soft shutdown, the stock driver's steps."""
        if not self._awake:
            return
        self._partial = False
        try:
            await self._ready("before shutdown", _PULL_TIMEOUT_MS)
        except PanelError as ex:
            print("epd_partial:", ex, "- shutting down anyway")
        cmd = self._cmd
        cmd(0x09, b"\x7f")
        cmd(0x05, b"\x7d")
        cmd(0x09, b"\x00")
        await aio.sleep_ms(200)
        self._awake = False
        try:
            await self._ready("shutdown", 2_000)
        except PanelError as ex:
            print("epd_partial:", ex)
        self._dc(0)
        self._cs1(0)
        self._rst(0)
        self._cs1(1)

    def _load(self, index, src=None, value=0):
        """A whole plane into both chips' RAM `index` (0x10/0x11): src, or every byte = value."""
        stage, win, cmd = self._stage, self._win, self._cmd
        cmd(0x13, _DUW_ALL)
        cmd(0x90, _DRFW_ALL)
        for cog, off in ((_LEFT, 0), (_RIGHT, _HALF)):
            if src is None:
                _fill(stage, value, _CHIP_PLANE)
            else:
                win[0], win[1], win[2], win[3], win[4] = 0, _HALF - 1, 0, _H - 1, off
                _gather(stage, src, win, _REV)
            cmd(0x12, b"\x3b\x00\x14", cog)   # RAM pointer: row 0, x counting down from 59
            cmd(index, stage, cog)

    def _window(self, cog, index, src):
        """Box self._win of one chip's half of plane src into its RAM `index`."""
        win, duw, ptr = self._win, self._duw, self._ptr
        x0, x1, y0, y1 = win[0], win[1], win[2], win[3]
        # The chip's RAM x runs down from 59 while screen x runs up.
        duw[0], duw[1] = 59 - x1, 59 - x0
        duw[2], duw[3], duw[4], duw[5] = y0 & 0xFF, y0 >> 8, y1 & 0xFF, y1 >> 8
        ptr[0], ptr[1], ptr[2] = 59 - x0, y0 & 0xFF, 0x14 | y0 >> 8
        n = _gather(self._stage, src, win, _REV)
        self._cmd(0x13, duw, cog)
        self._cmd(0x12, ptr, cog)
        self._cmd(index, memoryview(self._stage)[:n], cog)
        return n

    async def _pull(self):
        """One refresh on what's in RAM. Returns its ms."""
        self._cmd(0x15, b"\x3c")
        await aio.sleep_ms(5)
        ms = 5 + await self._ready("refresh", _PULL_TIMEOUT_MS)
        if ms > _SLOW_PULL_MS and self._waits is _QUICK:
            # The panel played its own waveform: the quick session didn't take the table.
            print(f"epd_partial: a {ms} ms refresh; switching to the slow session waits")
            self._waits = _SAFE
        return ms

    async def _open(self):
        """A partial session: the transition table, both RAMs = the film."""
        await self._reset()
        await self._start(self._lut)
        self._load(0x10, self._shown)
        self._load(0x11, self._shown)
        self._prev_hit[0] = self._prev_hit[1] = 0
        self.pulls = 0
        self._partial = True

    # ---- public ------------------------------------------------------------

    async def start(self, clean_red: bool = True) -> None:
        """Take the panel over from the stock driver, clean it to white and open a session."""
        d = _epd.display
        if not d._reverse_fb_bits:
            raise PanelError("partial refresh is only known to work on the B1 panel")
        while d._framebuffer_ready.is_set() or not d.is_done_updating():
            await aio.sleep_ms(50)
        forget = getattr(d, "forget_panel", None)
        if forget is not None:  # the stock driver must not skip its next refresh
            forget()
        wipe = True
        if clean_red and _any(d._framebuffer_red, _PLANE):   # nothing fast removes red
            print("epd_partial: red on the panel, running a normal refresh first")
            d.clear()
            d.update()
            await aio.sleep_ms(200)
            while d._framebuffer_ready.is_set() or not d.is_done_updating():
                await aio.sleep_ms(100)
            wipe = False    # the stock refresh left the film clean and white
        lock = getattr(d, "lock", None)
        if lock is not None:
            await lock.acquire()
            self._locked = True
        try:
            if self._rst():   # chips awake, maybe still refreshing: never reset them mid-refresh
                await self._ready("idle", 60_000)
            self._spi = _epd.display.spi   # borrow the display's own bus (hardware SPI(1))
            if self.buf is None:
                self.buf = bytearray(_PLANE)
                self.fb = framebuf.FrameBuffer(self.buf, _W, _H, framebuf.MONO_HLSB)
                self._shown = bytearray(_PLANE)
                self._stage = bytearray(_CHIP_PLANE)
            self.clear()
            _zero(self._shown, _PLANE // 4)
            if wipe:
                await self.wipe()
            await self._open()
        except BaseException:
            await self.stop()
            raise

    async def wipe(self) -> None:
        """Flash black then white (~3 s) to take ghosting off; fb is kept, the session closed."""
        await self._shutdown()
        from data.rendered_luts import lutb_base   # the badge team's short black/white table
        if len(lutb_base) != 400:
            raise PanelError("lutb_base must be 400 bytes")
        await self._reset()
        self._load(0x10, None, 0xFF)
        self._load(0x11, None, 0x00)
        await self._start(bytes(lutb_base))
        await self._pull()
        self._load(0x10, None, 0x00)   # both: after the pull 0x11 holds the black frame,
        self._load(0x11, None, 0x00)   # which would pick the red table
        await self._pull()
        await self._shutdown()
        _zero(self._shown, _PLANE // 4)

    def clear(self) -> None:
        """fb all white, fast."""
        _zero(self.buf, _PLANE // 4)

    async def sleep(self) -> None:
        """Power the panel down while the picture sits still; the next show() reopens (~0.5 s)."""
        await self._shutdown()

    async def restart(self, wipe: bool = False) -> None:
        """Close the session and open a fresh one (~0.5 s, ~3.5 s with a wipe), best in a pause."""
        if wipe:
            await self.wipe()
        else:
            await self._shutdown()
        await self._open()

    async def show(self) -> int:
        """Show fb with one calm partial pull; returns the ms it took, or 0 if nothing changed."""
        t0 = time.ticks_ms()
        buf, shown, now, prev, win = self.buf, self._shown, self._now, self._prev, self._win
        hit = (_diff(buf, shown, 0, now[0]), _diff(buf, shown, _HALF, now[1]))
        if not (hit[0] or hit[1]):
            return 0
        if not self._partial or self.pulls >= SESSION_PULLS:
            await self.restart()
        for c in (0, 1):
            # Resend this frame's box and the previous one's: the chip may have
            # swapped or copied its RAMs after the last pull (badge os saw a
            # swap), and either way both RAMs are right again after this.
            n, p = now[c], prev[c]
            if hit[c] and self._prev_hit[c]:
                win[0], win[1] = min(n[0], p[0]), max(n[1], p[1])
                win[2], win[3] = min(n[2], p[2]), max(n[3], p[3])
            elif hit[c]:
                win[0], win[1], win[2], win[3] = n[0], n[1], n[2], n[3]
            elif self._prev_hit[c]:
                win[0], win[1], win[2], win[3] = p[0], p[1], p[2], p[3]
            else:
                continue
            win[4] = c * _HALF
            cog = _LEFT if c == 0 else _RIGHT
            self._window(cog, 0x10, buf)     # new frame
            self._window(cog, 0x11, shown)   # what the film shows
        for c in (0, 1):
            if hit[c]:
                p, n = prev[c], now[c]
                p[0], p[1], p[2], p[3] = n[0], n[1], n[2], n[3]
            self._prev_hit[c] = hit[c]
        self._cmd(0x13, _DUW_ALL)   # RAM window back to the whole panel before the pull
        shown[:] = buf
        self.pulls += 1
        self.last_send_ms = time.ticks_diff(time.ticks_ms(), t0)
        self.last_pull_ms = await self._pull()
        return time.ticks_diff(time.ticks_ms(), t0)

    async def stop(self) -> None:
        """Power the panel down and hand the panel back to the stock driver."""
        try:
            await self._shutdown()
        finally:
            self._partial = False
            self._spi = None
            # The film now holds a partial-refresh picture with light ghosts, so
            # epd_grayscale should clean it before a gray update.
            _epd.display._panel_monochrome = False
            if self._locked:
                self._locked = False
                _epd.display.lock.release()
