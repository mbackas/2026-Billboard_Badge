"""E-Paper Display Driver for Billboard Badge

Ported from Wiring in https://github.com/PervasiveDisplays/EPD_Driver_GU_large/blob/e408154652802a82e520cc25ba04adeb347d0528/src/EPD_Driver.cpp

--COPYRIGHT--
Brief The drivers and functions of development board
Copyright (c) 2012-2021 Pervasive Displays Inc. All rights reserved.
 Authors: Pervasive Displays Inc.
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions
 are met:
 1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
 2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in
    the documentation and/or other materials provided with the
    distribution.
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

import asyncio as aio
import btree
import framebuf
import gc
from machine import Pin, SPI, SoftSPI
import micropython
from micropython import const
import time

import lvgl

from lib.large_png import show_png
from lib.led_bar import leds

EPD_CS1_PIN = Pin(13, mode=Pin.OUT)
EPD_CS2_PIN = Pin(3, mode=Pin.OUT)
EPD_BUSY1_PIN = Pin(12, mode=Pin.IN)
EPD_BUSY2_PIN = Pin(8, mode=Pin.IN)
EPD_DC_PIN = Pin(10, mode=Pin.OUT)  # Data High Command Low
EPD_RST_PIN = Pin(11, mode=Pin.OUT)
EPD_SCL_PIN = Pin(21, mode=Pin.OUT)
EPD_SDA_PIN = Pin(47, mode=Pin.OUT)
EPD_SPI_MISO_PIN = Pin(45, mode=Pin.IN)  # Not connected, signal not needed
EPD_PWR_EN_PIN = Pin(48, mode=Pin.OUT, value=False)


pdi_cp = 0x9608
pdi_size = 0x96  # 9.69"
display_width = const(960)
half_width = const(display_width // 2)
display_height = const(672)
# Frame-buffer size = width * height / 8 * depth, uint32_t
# image_data_size is really only half the panel pixels, of one color
half_frame_one_color_size = const(display_height * display_width // (8 * 2))
# The width of rows in the full-frame and half-frame framebuffers
# Counting bytes, not pixels, since each byte is 8 packed pixels
display_width_bytes = const(display_width // 8)
display_half_width_bytes = const(display_width_bytes // 2)


# Two displays use different bit orders
def reverse_byte(byte: int) -> int:
    byte = (byte & 0xF0) >> 4 | (byte & 0x0F) << 4
    byte = (byte & 0xCC) >> 2 | (byte & 0x33) << 2
    byte = (byte & 0xAA) >> 1 | (byte & 0x55) << 1
    return byte


# Pre-computed 256-byte LUT for instant bit reversal
_REVERSE_LUT = bytearray(256)
for _i in range(256):
    _REVERSE_LUT[_i] = reverse_byte(_i)


@micropython.viper
def _viper_lv_flush(
    src: ptr8,
    fb_black: ptr8,
    fb_red: ptr8,
    x1: int,
    y1: int,
    w: int,
    h: int,
    stride: int,
):
    width = 960
    for row in range(h):
        base = (y1 + row) * width + x1
        row_off = row * stride
        for col in range(w):
            i = row_off + (col << 1)
            c = src[i] | (src[i + 1] << 8)  # RGB565, little-endian
            idx = base + col
            byte_idx = idx >> 3
            mask = 1 << (7 - (idx & 7))
            clear = mask ^ 0xFF
            bits = c & 0x8410
            if bits == 0x8000:
                fb_red[byte_idx] |= mask
                fb_black[byte_idx] &= clear
            elif bits == 0:
                fb_black[byte_idx] |= mask
                fb_red[byte_idx] &= clear
            else:
                fb_black[byte_idx] &= clear
                fb_red[byte_idx] &= clear


@micropython.viper
def _fast_split_and_reverse(src_fb: ptr8, dst_left: ptr8, dst_right: ptr8, lut: ptr8):
    for r in range(672):
        src_row = r * 120
        dst_row = r * 60
        for c in range(60):
            dst_left[dst_row + c] = lut[src_fb[src_row + c]]
            dst_right[dst_row + c] = lut[src_fb[src_row + 60 + c]]


def _buf(data):
    """SPI can send bytes-like data as is; only lists need converting (no 40 KB copies)."""
    return data if isinstance(data, (bytes, bytearray, memoryview)) else bytes(data)


@micropython.viper
def _reverse_copy(src: ptr8, dst: ptr8, lut: ptr8, n: int):
    for i in range(n):
        dst[i] = lut[src[i]]


# Start-up waits in ms: before reset, reset high, reset low, reset high again,
# after 0x05 7D, after 0x05 00, waveform-load strobe high, strobe low.
# STOCK is Pervasive Displays' reference driver. QUICK is lib/epd_partial.py's,
# which badge os found works on the B1 panel; the Refresh Lab (2026-11-12) showed
# identical pictures with it and 1.65 s less per refresh.
_STOCK_WAITS = (200, 20, 200, 200, 200, 10, 100, 100)
_QUICK_WAITS = (0, 2, 4, 20, 50, 1, 2, 10)


@micropython.viper
def _fast_split_no_reverse(src_fb: ptr8, dst_left: ptr8, dst_right: ptr8):
    for r in range(672):
        src_row = r * 120
        dst_row = r * 60
        for c in range(60):
            dst_left[dst_row + c] = src_fb[src_row + c]
            dst_right[dst_row + c] = src_fb[src_row + 60 + c]


class EPaperDisplay:
    width = display_width
    height = display_height

    def __init__(self) -> None:
        print("Initializing E-Paper Display (Fast Hardware SPI + Viper)")
        EPD_DC_PIN.value(True)
        EPD_RST_PIN.value(True)
        EPD_CS1_PIN.value(True)
        EPD_CS2_PIN.value(True)
        try:
            self.spi = SPI(
                1,
                baudrate=20_000_000,
                sck=EPD_SCL_PIN,
                mosi=EPD_SDA_PIN,
                miso=EPD_SPI_MISO_PIN,
            )
        except Exception:
            self.spi = SoftSPI(
                baudrate=8_000_000,
                sck=EPD_SCL_PIN,
                mosi=EPD_SDA_PIN,
                miso=EPD_SPI_MISO_PIN,
            )
            self.spi.init()

        try:
            with open("/data/hwconf", "rb") as f:
                hwconf = btree.open(f)
                display_pn = hwconf.get(b"display_pn", b"B1")
                if display_pn == b"84":
                    self._reverse_fb_bits = False
                else:
                    self._reverse_fb_bits = True
        except OSError:
            self._reverse_fb_bits = True

        # The framebuffers are set and display should be updated.
        self._framebuffer_ready = aio.Event()
        # Panel capability stays stable while the worker sends prepared frames.
        self._supports_grayscale = self._reverse_fb_bits
        self._requested_grayscale = False
        self._requested_mixed = False
        self._render_grayscale = False
        self._render_mixed = False
        # E-paper keeps its image across reboot; its initial pigments are unknown.
        self._panel_monochrome = False
        # Red may be on the panel. Only the panel's own (full) waveform removes red,
        # so gray updates clean up first only while this is True.
        self._panel_has_red = True
        # Start-up waits: quick on the B1 panel (see _QUICK_WAITS), stock otherwise.
        self._waits = _QUICK_WAITS if self._reverse_fb_bits else _STOCK_WAITS
        # The last frame run() put on the panel, so an identical one can be skipped.
        # _last_mode is (grayscale, mixed), or None when the panel's contents are
        # unknown: before the first refresh, after a failed one, or after another
        # driver (lib/epd_partial.py) drew on it. Those call forget_panel().
        self._last_mode = None
        self._last_planes = None
        self._rev_buf = None  # scratch plane for _send_images' bit reversal
        self._png_blink = False
        self._refreshing_hw = False

        """Each byte is a mini-row of 8 bits for pixels of one color (1 sets the color, 0 is white)
        Bytes are in rows across the screen, as seen on the badge
        0 ------------------------> 59 [60 bytes per row]
        60 ----------------------> 119
        ...                        ...
        [672 rows of 60 bytes per row]
        """
        self._framebuffer_black = bytearray(half_frame_one_color_size * 2)
        self._framebuffer_red = bytearray(half_frame_one_color_size * 2)
        # Framebuffers that can be drawn to directly
        self.black = framebuf.FrameBuffer(
            self._framebuffer_black, display_width, display_height, framebuf.MONO_HLSB
        )
        self.red = framebuf.FrameBuffer(
            self._framebuffer_red, display_width, display_height, framebuf.MONO_HLSB
        )

        # Is the display done updating? Start set since the display is not updating before boot.
        self._display_updated = aio.Event()
        self._display_updated.set()
        # Held while something drives the panel: run() below, or an opt-in driver
        # of its own such as lib/epd_partial.py. Keeps the two from overlapping.
        self.lock = aio.Lock()
        gc.collect()

        # Set up lvgl (deinit first if soft-rebooted so C pointers don't corrupt)
        if hasattr(lvgl, "is_initialized") and lvgl.is_initialized():
            if hasattr(lvgl, "deinit"):
                lvgl.deinit()
        lvgl.init()
        # Give LVGL a buffer 2-bytes-per-pixel for RGB565 format
        self._lv_draw_buf = bytearray(display_width * display_height * 2)

        self.lv_display = lvgl.display_create(display_width, display_height)
        self.lv_display.set_color_format(lvgl.COLOR_FORMAT.RGB565)
        self.lv_display.set_buffers(
            self._lv_draw_buf,
            None,
            len(self._lv_draw_buf),
            lvgl.DISPLAY_RENDER_MODE.PARTIAL,
        )
        self.lv_display.set_flush_cb(self._lv_flush)
        self.lv_screen = lvgl.screen_active()
        self.lv_screen.set_size(display_width, display_height)

    def _lv_flush(self, disp, area, color_p):
        print("Calling EPaperDisplay._lv_flush (Viper)")
        leds[0] = (0, 0, 50)
        leds.write()
        x1, y1 = area.x1, area.y1
        w = area.x2 - x1 + 1
        h = area.y2 - y1 + 1
        try:
            stride = lvgl.draw_buf_width_to_stride(w, lvgl.COLOR_FORMAT.RGB565)
        except AttributeError:
            stride = w * 2
        src = color_p.__dereference__(stride * h)
        convert = _viper_lv_flush
        if (self._render_grayscale or self._render_mixed) and self.supports_grayscale:
            from lib import epd_grayscale
            convert = (epd_grayscale.rgb565_to_mixed if self._render_mixed
                       else epd_grayscale.rgb565_to_grayscale)
        convert(
            src,
            self._framebuffer_black,
            self._framebuffer_red,
            x1,
            y1,
            w,
            h,
            stride,
        )
        leds[0] = (0, 0, 0)
        leds.write()
        disp.flush_ready()

    @property
    def supports_grayscale(self):
        return self._supports_grayscale

    def _check_modes(self, grayscale, mixed):
        if grayscale and mixed:
            raise ValueError("Choose grayscale or mixed mode")
        if (grayscale or mixed) and not self.supports_grayscale:
            raise ValueError("Grayscale refresh requires a B1 panel")

    def refresh_lvgl(self, grayscale=False, mixed=False):
        """Render LVGL into the framebuffers; use the same mode in update().

        Default rendering keeps the original B/W/red thresholds. Grayscale
        produces B/W/gray; mixed preserves red and adds gray as a fourth color.
        """
        self._check_modes(grayscale, mixed)
        previous_grayscale = self._render_grayscale
        previous_mixed = self._render_mixed
        try:
            self._render_grayscale = grayscale
            self._render_mixed = mixed
            lvgl.refr_now(self.lv_display)
        finally:
            self._render_grayscale = previous_grayscale
            self._render_mixed = previous_mixed

    def update(self, grayscale=False, mixed=False):
        """Queue the latest frame; gray waveforms are opt-in and require B1.

        grayscale=True uses 00 white, 01 black, 10 gray (no red).
        mixed=True uses 00 white, 01 black, 10 red, 11 gray in software.
        Match the mode used by refresh_lvgl()/draw_png().
        """
        self._check_modes(grayscale, mixed)
        self._requested_grayscale = grayscale
        self._requested_mixed = mixed
        self._display_updated.clear()
        self._framebuffer_ready.set()

    def clear(self):
        """Clear framebuffers to full white"""
        self.black.fill(0)
        self.red.fill(0)

    def is_done_updating(self):
        """Is the display done updating?"""
        return self._display_updated.is_set()

    def draw_image(self, image, height: int, width: int, row: int = 0, col: int = 0):
        for img_r in range(height):
            for img_c in range(width):
                color = image[img_r][img_c]
                red = color[0] > 127
                black = color[0] < 128 and color[1] < 128 and color[2] < 128
                self.black.pixel((col + img_c), (row + img_r), int(black))
                self.red.pixel((col + img_c), (row + img_r), int(red))

    """
    Draw a PNG
    
    If drawing a PNG from a byte array, pass None to `filename` and pass the byte array in `data`
    """
    def draw_png(self, filename: str = None, row: int = 0, col: int = 0, grayscale: bool = False, mixed: bool = False, data: bytes = None):
        """Draw a PNG; use the same grayscale/mixed mode in update()."""
        self._check_modes(grayscale, mixed)
        gc.collect()
        self._png_blink = not self._png_blink
        leds[0] = (0, 0, 50) if self._png_blink else (0, 0, 10)
        leds.write()
        print(f"Loading {filename} This will take a while... {time.time()}")
        show_png(filename, self._framebuffer_black, self._framebuffer_red, row, col, grayscale, mixed, data)
        leds[0] = (0, 0, 0)
        leds.write()
        print("Done loading png")
        gc.collect()

    async def _blink_red_while_refreshing(self):
        state = False
        while self._refreshing_hw:
            state = not state
            leds[0] = (40, 0, 0) if state else (0, 0, 0)
            leds.write()
            await aio.sleep_ms(200)

    async def run(self):
        black_left = bytearray(half_frame_one_color_size)
        black_right = bytearray(half_frame_one_color_size)
        red_left = bytearray(half_frame_one_color_size)
        red_right = bytearray(half_frame_one_color_size)
        while True:
            if self._framebuffer_ready.is_set():
                # Clear before awaiting hardware so a later request is retained.
                self._framebuffer_ready.clear()
                grayscale = self._requested_grayscale
                mixed = self._requested_mixed
                self._display_updated.clear()
                t0 = time.ticks_ms()
                if self._reverse_fb_bits and not (grayscale or mixed):
                    _fast_split_and_reverse(
                        self._framebuffer_black, black_left, black_right, _REVERSE_LUT
                    )
                    _fast_split_and_reverse(
                        self._framebuffer_red, red_left, red_right, _REVERSE_LUT
                    )
                else:
                    _fast_split_no_reverse(
                        self._framebuffer_black, black_left, black_right
                    )
                    _fast_split_no_reverse(
                        self._framebuffer_red, red_left, red_right
                    )
                print("Viper split & reverse ms:", time.ticks_diff(time.ticks_ms(), t0))

                mode = (grayscale, mixed)
                if self._same_as_last(mode, black_left, red_left, black_right, red_right):
                    # The panel already shows exactly this: a refresh would only
                    # flash for ~35 s (60 s for gray) and change nothing.
                    print("EPD: frame unchanged, refresh skipped")
                    if not self._framebuffer_ready.is_set():
                        self._display_updated.set()
                    continue
                self._remember(black_left, red_left, black_right, red_right)

                leds.clear()
                self._refreshing_hw = True
                _blink_task = aio.create_task(self._blink_red_while_refreshing())

                old_rev = self._reverse_fb_bits
                # Gray helpers consume logical-order planes and reverse on send.
                if not (grayscale or mixed):
                    self._reverse_fb_bits = False
                try:
                    async with self.lock:
                        if mixed:
                            from lib.epd_grayscale import mixed_update
                            await mixed_update(self, black_left, red_left, black_right, red_right)
                        else:
                            await self.global_update(
                                black_left, red_left, black_right, red_right, grayscale=grayscale)
                except BaseException:
                    self._panel_monochrome = False
                    self._panel_has_red = True
                    raise
                finally:
                    self._reverse_fb_bits = old_rev
                    self._refreshing_hw = False
                    await aio.sleep_ms(50)
                self._last_mode = mode

                # Done first: the green LED and gc used to hold this back ~0.7 s.
                if not self._framebuffer_ready.is_set():
                    self._display_updated.set()
                leds[0] = (0, 40, 0)
                leds.write()
                aio.create_task(self._led_off_later())
                gc.collect()
            await aio.sleep_ms(50)

    async def _led_off_later(self):
        await aio.sleep_ms(400)
        if not self._refreshing_hw:
            leds.clear()

    def forget_panel(self):
        """Something other than run() drew on the panel: never skip the next refresh."""
        self._last_mode = None

    def _same_as_last(self, mode, black_left, red_left, black_right, red_right):
        last = self._last_planes
        return (
            self._last_mode == mode
            and last is not None
            and last[0] == black_left
            and last[1] == red_left
            and last[2] == black_right
            and last[3] == red_right
        )

    def _remember(self, black_left, red_left, black_right, red_right):
        """Copy the frame about to be sent (mixed_update edits its planes in place)."""
        self._last_mode = None  # until the refresh succeeds
        if self._last_planes is None:
            self._last_planes = [bytearray(half_frame_one_color_size) for _ in range(4)]
        for keep, plane in zip(self._last_planes, (black_left, red_left, black_right, red_right)):
            keep[:] = plane

    async def global_update(
        self,
        black_left: bytearray | bytes | memoryview[int],
        red_left: bytearray | bytes | memoryview[int],
        black_right: bytearray | bytes | memoryview[int],
        red_right: bytearray | bytes | memoryview[int],
        grayscale=False,
    ):
        if grayscale:
            self._check_modes(True, False)
            from lib.epd_grayscale import grayscale_update
            await grayscale_update(self, black_left, red_left, black_right, red_right)
            return
        # Keep the ordinary three-color sequence and overlapping-plane behavior.
        has_red = any(red_left) or any(red_right)
        self._panel_monochrome = False
        self._panel_has_red = True  # unknown until this finishes
        print("Updating display to new image")
        await self._reset()
        tick = time.ticks_ms()
        self._send_images(black_left, red_left, black_right, red_right)
        tock = time.ticks_ms()
        print(tock - tick)

        print("display working")
        tick = time.ticks_ms()
        await self.cog_initial()
        await self._dcdc_soft_start()
        await self._display_refresh()
        await self._dcdc_soft_shutdown()
        print("Done updating display")
        tock = time.ticks_ms()
        print(tock - tick)
        self._panel_monochrome = not has_red
        self._panel_has_red = has_red

    async def cog_initial(self):
        w = self._waits
        self._send_index_data(0x05, [0x7D])
        await aio.sleep_ms(w[4])
        self._send_index_data(0x05, [0x00])
        await aio.sleep_ms(w[5])
        self._send_index_data(0xD8, [0x80])  # MS_SYNC)
        self._send_index_data(0xD6, [0x00])  # BVSS
        self._send_index_data(0xA7, [0x10])
        await aio.sleep_ms(w[6])
        self._send_index_data(0xA7, [0x00])
        await aio.sleep_ms(w[7])
        self._send_index_data(0x03, [0x00, 0x11])  # OSC

        # Display 1
        self._send_index_data_left(0x44, [0x00])
        self._send_index_data_left(0x45, [0x80])
        self._send_index_data_left(0xA7, [0x10])
        await aio.sleep_ms(w[6])
        self._send_index_data_left(0xA7, [0x00])
        await aio.sleep_ms(w[7])
        self._send_index_data_left(0x44, [0x06])
        self._send_index_data_left(0x45, [0x82])  # Temperature 0x82@25C
        self._send_index_data_left(0xA7, [0x10])
        await aio.sleep_ms(w[6])
        self._send_index_data_left(0xA7, [0x00])
        await aio.sleep_ms(w[7])

        # Display 2
        self._send_index_data_right(0x44, [0x00])
        self._send_index_data_right(0x45, [0x80])
        self._send_index_data_right(0xA7, [0x10])
        await aio.sleep_ms(w[6])
        self._send_index_data_right(0xA7, [0x00])
        await aio.sleep_ms(w[7])
        self._send_index_data_right(0x44, [0x06])
        self._send_index_data_right(0x45, [0x82])  # Temperature 0x82@25C
        self._send_index_data_right(0xA7, [0x10])
        await aio.sleep_ms(w[6])
        self._send_index_data_right(0xA7, [0x00])
        await aio.sleep_ms(w[7])

        self._send_index_data(0x60, [0x25])  # TCON
        self._send_index_data_left(0x61, [0x01])  # STV_DIR for Display 1
        self._send_index_data(0x02, [0x00])  # VCOM

    async def cog_poweroff(self):
        """CoG shutdown function
        Shuts down the CoG and DC/DC circuit after all update functions
        """
        self._send_index_data(0x09, [0x7F, 0x7D, 0x00])
        await aio.sleep_ms(200)
        while not EPD_BUSY1_PIN.value() and not EPD_BUSY2_PIN.value():
            await aio.sleep_ms(1)
        EPD_DC_PIN.value(False)
        EPD_CS1_PIN.value(False)
        await aio.sleep_ms(150)
        EPD_RST_PIN.value(False)

    def _send_index_data(self, index: int, data: bytes | list):
        EPD_DC_PIN.value(False)
        EPD_CS1_PIN.value(False)
        EPD_CS2_PIN.value(False)
        self.spi.write(bytes([index]))
        EPD_CS1_PIN.value(True)
        EPD_CS2_PIN.value(True)
        EPD_DC_PIN.value(True)
        EPD_CS1_PIN.value(False)
        EPD_CS2_PIN.value(False)
        self.spi.write(_buf(data))
        EPD_CS1_PIN.value(True)
        EPD_CS2_PIN.value(True)

    def _send_index_data_left(self, index: int, data: bytes | list | memoryview[int]):
        EPD_DC_PIN.value(False)
        EPD_CS1_PIN.value(False)
        EPD_CS2_PIN.value(True)
        self.spi.write(bytes([index]))
        EPD_CS1_PIN.value(True)
        EPD_DC_PIN.value(True)
        EPD_CS1_PIN.value(False)
        self.spi.write(_buf(data))
        EPD_CS1_PIN.value(True)

    def _send_index_data_right(self, index: int, data: bytes | list | memoryview[int]):
        EPD_DC_PIN.value(False)
        EPD_CS1_PIN.value(True)
        EPD_CS2_PIN.value(False)
        self.spi.write(bytes([index]))
        EPD_CS2_PIN.value(True)
        EPD_DC_PIN.value(True)
        EPD_CS2_PIN.value(False)
        self.spi.write(_buf(data))
        EPD_CS2_PIN.value(True)

    async def _display_refresh(self):
        while not EPD_BUSY1_PIN.value() and not EPD_BUSY2_PIN.value():
            await aio.sleep_ms(10)
        self._send_index_data(0x15, [0x3C])
        await aio.sleep_ms(5)

    async def _reset(self):
        """CoG driver power-on hard reset"""
        w = self._waits
        await aio.sleep_ms(w[0])
        EPD_RST_PIN.value(True)
        await aio.sleep_ms(w[1])
        EPD_RST_PIN.value(False)
        await aio.sleep_ms(w[2])
        EPD_RST_PIN.value(True)
        await aio.sleep_ms(w[3])
        EPD_CS1_PIN.value(True)
        EPD_CS2_PIN.value(True)
        await aio.sleep_ms(5)
        self._send_index_data(0x70, [0x71, 0x36])  # B1 booster config, itc97; Errata 9

    def _send_images(
        self,
        black_left: bytearray | bytes | memoryview[int],
        red_left: bytearray | bytes | memoryview[int],
        black_right: bytearray | bytes | memoryview[int],
        red_right: bytearray | bytes | memoryview[int],
    ):
        assert len(black_left) == half_frame_one_color_size
        assert len(red_left) == half_frame_one_color_size
        assert len(black_right) == half_frame_one_color_size
        assert len(red_right) == half_frame_one_color_size

        # DUW for both left and right halves
        self._send_index_data(0x13, [0x00, 0x3B, 0x00, 0x00, 0x9F, 0x02])
        # DRFW for Both left and right halves
        self._send_index_data(0x90, [0x00, 0x3B, 0x00, 0xA9])
        ram_rw_data = [0x3B, 0x00, 0x14]
        self._send_index_data_left(0x12, ram_rw_data)  # RAM_RW for left half
        self._send_index_data_left(0x10, self._plane(black_left))  # Black frame for left half
        self._send_index_data_left(0x12, ram_rw_data)  # RAM_RW for left half
        self._send_index_data_left(0x11, self._plane(red_left))  # Red frame for left half
        self._send_index_data_right(0x12, ram_rw_data)  # RAM_RW for right half
        self._send_index_data_right(0x10, self._plane(black_right))  # black frame for right half
        self._send_index_data_right(0x12, ram_rw_data)  # RAM_RW for right half
        self._send_index_data_right(0x11, self._plane(red_right))  # Red frame for right half

    def _plane(self, data):
        """A plane as the panel takes it. On B1 every byte's bits are reversed:
        viper with a lookup table, ~17 ms a plane. The Python generator this
        replaces took ~1.08 s a plane (Refresh Lab), 4.3 s per _send_images."""
        if not self._reverse_fb_bits:
            return data
        if self._rev_buf is None:
            self._rev_buf = bytearray(half_frame_one_color_size)
        _reverse_copy(data, self._rev_buf, _REVERSE_LUT, half_frame_one_color_size)
        return self._rev_buf  # sent before the next plane reuses it

    async def _dcdc_soft_start(self):
        """DC-DC soft-start command
        Implemented after image data are uploaded to CoG
        Specific to mid-sized EPDs only
        """
        self._send_index_data(0x51, [0x50, 0x01])

        for val in range(1, 5):
            self._send_index_data(0x09, [0x1F])
            self._send_index_data(0x51, [0x50, val])
            self._send_index_data(0x09, [0x9F])
            await aio.sleep_ms(2)

        for val in range(1, 11):
            self._send_index_data(0x09, [0x1F])
            self._send_index_data(0x51, [0x0A, val])
            self._send_index_data(0x09, [0x9F])
            await aio.sleep_ms(2)

        for val in range(3, 11):
            self._send_index_data(0x09, [0x7F])
            self._send_index_data(0x51, [0x0A, val])
            self._send_index_data(0x09, [0xFF])
            await aio.sleep_ms(2)

        for val in range(9, 1, -1):
            self._send_index_data(0x09, [0x7F])
            self._send_index_data(0x51, [val, 0x0A])
            self._send_index_data(0x09, [0xFF])
            await aio.sleep_ms(2)

        self._send_index_data(0x09, [0xFF])
        await aio.sleep_ms(10)

    async def _dcdc_soft_shutdown(self):
        """DC-DC soft-shutdown command
        Implemented after image data are uploaded to CoG
        Specific to mid-sized EPDs only
        """
        while not EPD_BUSY1_PIN.value() and not EPD_BUSY2_PIN.value():
            await aio.sleep_ms(10)
        self._send_index_data(0x09, [0x7F])
        self._send_index_data(0x05, [0x7D])
        self._send_index_data(0x09, [0x00])
        await aio.sleep_ms(200)
        while not EPD_BUSY1_PIN.value() and not EPD_BUSY2_PIN.value():
            await aio.sleep_ms(10)
        EPD_DC_PIN.value(False)
        EPD_CS1_PIN.value(False)
        EPD_RST_PIN.value(False)
        EPD_CS1_PIN.value(True)


display = EPaperDisplay()
