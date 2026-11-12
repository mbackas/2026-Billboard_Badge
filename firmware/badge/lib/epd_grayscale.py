"""Optional B1 grayscale and short-waveform refreshes for Billboard Badge

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
import micropython
from micropython import const
import time

WIDTH = const(960)
HALF_BYTES = const(40320)
WHITE = const(0)
BLACK = const(1)
RED = const(2)
GRAY = const(3)

_gray_waveform = None


@micropython.viper
def rgb565_to_grayscale(source, black, gray, x1: int, y1: int,
                                     width: int, height: int, stride: int):
    # Neutral RGB565 coverage: 00 paper, 01 black, 10 gray; never select 11.
    src = ptr8(source)
    black_buf = ptr8(black)
    gray_buf = ptr8(gray)
    for row in range(height):
        base = (y1 + row) * WIDTH + x1
        row_off = row * stride
        for col in range(width):
            offset = row_off + col * 2
            color = src[offset] | (src[offset + 1] << 8)
            level = ((color >> 5) & 63) * 255 // 63
            pixel = base + col
            byte = pixel >> 3
            mask = 128 >> (pixel & 7)
            clear = 255 ^ mask
            if level < 96:
                black_buf[byte] = black_buf[byte] | mask
                gray_buf[byte] = gray_buf[byte] & clear
            elif level < 204:
                black_buf[byte] = black_buf[byte] & clear
                gray_buf[byte] = gray_buf[byte] | mask
            else:
                black_buf[byte] = black_buf[byte] & clear
                gray_buf[byte] = gray_buf[byte] & clear


@micropython.viper
def rgb565_to_mixed(source, black, red, x1: int, y1: int,
                                 width: int, height: int, stride: int):
    # Software colors: 00 white, 01 black, 10 red, 11 gray.
    # Mixed update removes 11 before the normal full-color waveform.
    src = ptr8(source)
    black_buf = ptr8(black)
    red_buf = ptr8(red)
    for row in range(height):
        base = (y1 + row) * WIDTH + x1
        row_off = row * stride
        for col in range(width):
            offset = row_off + col * 2
            color = src[offset] | (src[offset + 1] << 8)
            level = ((color >> 5) & 63) * 255 // 63
            pixel = base + col
            byte = pixel >> 3
            mask = 128 >> (pixel & 7)
            clear = 255 ^ mask
            if (color & 0x8410) == 0x8000:
                black_buf[byte] = black_buf[byte] & clear
                red_buf[byte] = red_buf[byte] | mask
            elif level < 96:
                black_buf[byte] = black_buf[byte] | mask
                red_buf[byte] = red_buf[byte] & clear
            elif level < 204:
                black_buf[byte] = black_buf[byte] | mask
                red_buf[byte] = red_buf[byte] | mask
            else:
                black_buf[byte] = black_buf[byte] & clear
                red_buf[byte] = red_buf[byte] & clear


@micropython.viper
def _extract_gray(black, red, gray, offset: int, count: int):
    # Mutate only the worker's private snapshots, making gray sites white.
    b = ptr8(black)
    r = ptr8(red)
    g = ptr8(gray)
    for i in range(count):
        selected = b[i] & r[i]
        g[offset + i] = selected
        keep = 255 ^ selected
        b[i] = b[i] & keep
        r[i] = r[i] & keep


@micropython.viper
def _gray_hold_planes(black, red, gray, offset: int, count: int, paint: int):
    b = ptr8(black)
    r = ptr8(red)
    g = ptr8(gray)
    for i in range(count):
        hold = 255 ^ g[offset + i]
        b[i] = hold
        r[i] = 255 if paint else hold


@micropython.viper
def _planes_overlap(black, red, count: int) -> int:
    b = ptr8(black)
    r = ptr8(red)
    for i in range(count):
        if b[i] & r[i]:
            return 1
    return 0


@micropython.viper
def grayscale_kind(green_value: int) -> int:
    # These thresholds match the RGB565 grayscale flush for every 8-bit input.
    if green_value < 96:
        return BLACK
    if green_value < 204:
        return RED  # The second plane selects gray during a grayscale update.
    return WHITE


@micropython.viper
def mixed_kind(red_value: int, green_value: int, blue_value: int) -> int:
    if red_value >= 128 and green_value < 128 and blue_value < 128:
        return RED
    if green_value < 96:
        return BLACK
    if green_value < 204:
        return GRAY
    return WHITE


def gray_waveform(display):
    global _gray_waveform
    if not display.supports_grayscale:
        raise ValueError("Grayscale refresh requires a B1 panel")
    if _gray_waveform is None:
        from data import rendered_luts
        table = bytearray(rendered_luts.lutb_base)
        if len(table) != 400:
            raise ValueError("Grayscale refresh requires a 400-byte B1 waveform")
        # Keep slot 2's balanced black shake, without its later drive. Six
        # white preparation pulls make the resulting gray independent of ink.
        table[160:240] = table[80:160]
        for group in range(1, 5):
            table[160 + group * 8] = 0
        _gray_waveform = bytes(table)
    return _gray_waveform


def _validate_halves(black_left, red_left, black_right, red_right):
    for part in (black_left, red_left, black_right, red_right):
        if len(part) != HALF_BYTES:
            raise ValueError("Invalid framebuffer size")


async def wait_ready(timeout_ms=10000):
    # Custom waveforms must finish on both COGs before RAM or rails change.
    # Keep this separate from the ordinary driver's upstream BUSY behavior.
    from lib.epd import EPD_BUSY1_PIN, EPD_BUSY2_PIN
    start = time.ticks_ms()
    while not EPD_BUSY1_PIN.value() or not EPD_BUSY2_PIN.value():
        if time.ticks_diff(time.ticks_ms(), start) >= timeout_ms:
            raise RuntimeError("EPD custom waveform BUSY completion timed out")
        await aio.sleep_ms(5)


async def pull(display):
    from lib.epd import EPD_BUSY1_PIN, EPD_BUSY2_PIN
    await wait_ready()
    display._send_index_data(0x15, [0x3C])
    # Observe both assertions before yielding; another app's synchronous
    # rendering can otherwise hide the entire BUSY pulse from this task.
    start = time.ticks_ms()
    busy_left = False
    busy_right = False
    while not (busy_left and busy_right):
        busy_left = busy_left or not EPD_BUSY1_PIN.value()
        busy_right = busy_right or not EPD_BUSY2_PIN.value()
        if time.ticks_diff(time.ticks_ms(), start) >= 100:
            raise RuntimeError("EPD custom waveform BUSY assertion timed out")
        time.sleep_ms(1)
    await wait_ready()


async def shutdown(display):
    from lib.epd import EPD_DC_PIN, EPD_CS1_PIN, EPD_RST_PIN
    await wait_ready()
    display._send_index_data(0x09, [0x7F])
    display._send_index_data(0x05, [0x7D])
    display._send_index_data(0x09, [0x00])
    await aio.sleep_ms(200)
    await wait_ready()
    EPD_DC_PIN.value(False)
    EPD_CS1_PIN.value(False)
    EPD_RST_PIN.value(False)
    EPD_CS1_PIN.value(True)


async def _cleanup_error(display):
    from lib.epd import EPD_RST_PIN
    display._panel_monochrome = False
    display._panel_has_red = True  # unknown after a failure
    try:
        await aio.wait_for(shutdown(display), 15)
    except Exception as error:
        # Preserve the caller's primary failure, including cancellation.
        print("EPD: custom waveform cleanup failed:", error)
    finally:
        EPD_RST_PIN.value(False)


async def _finish(display):
    from lib.epd import EPD_RST_PIN
    try:
        await aio.wait_for(shutdown(display), 15)
    except BaseException:
        display._panel_monochrome = False
        EPD_RST_PIN.value(False)
        raise


async def _start(display, black_left, red_left, black_right, red_right, waveform):
    await display._reset()
    display._send_images(black_left, red_left, black_right, red_right)
    await display.cog_initial()
    display._send_index_data(0xB0, waveform)
    await display._dcdc_soft_start()


async def grayscale_update(display, black_left, gray_left, black_right, gray_right):
    """Three tones: 00 white, 01 black, 10 gray; 11 is unsupported here."""
    _validate_halves(black_left, gray_left, black_right, gray_right)
    if (_planes_overlap(black_left, gray_left, HALF_BYTES)
            or _planes_overlap(black_right, gray_right, HALF_BYTES)):
        raise ValueError("Three-tone grayscale does not support selector 11")
    waveform = gray_waveform(display)
    white = bytearray(HALF_BYTES)
    # The full-waveform clean-up (~35 s) is only for red, which the gray waveform
    # can't remove. The Refresh Lab (2026-11-12) showed it doesn't otherwise make the
    # gray result any cleaner: the six white preparation pulls below do that job.
    needs_cleanup = getattr(display, "_panel_has_red", not display._panel_monochrome)
    display._panel_monochrome = False
    try:
        if needs_cleanup:
            # Never send gray selectors to the panel's normal OTP waveform.
            await aio.wait_for(display.global_update(white, white, white, white), 90)
            display._panel_monochrome = False
        await _start(display, white, white, white, white, waveform)
        for _ in range(6):
            await pull(display)
        display._send_images(black_left, gray_left, black_right, gray_right)
        for _ in range(3):
            await pull(display)
    except BaseException:
        await _cleanup_error(display)
        raise
    await _finish(display)
    display._panel_monochrome = True
    display._panel_has_red = False


async def mixed_update(display, black_left, red_left, black_right, red_right):
    """Consume worker snapshots: ordinary color, then masked gray at 11 sites."""
    _validate_halves(black_left, red_left, black_right, red_right)
    waveform = gray_waveform(display)
    gray = bytearray(HALF_BYTES * 2)
    # Extract before touching hardware. The application's buffers stay intact.
    _extract_gray(black_left, red_left, gray, 0, HALF_BYTES)
    _extract_gray(black_right, red_right, gray, HALF_BYTES, HALF_BYTES)
    has_gray = any(gray)
    display._panel_monochrome = False
    try:
        await aio.wait_for(display.global_update(black_left, red_left, black_right, red_right), 90)
        display._panel_monochrome = False
        if has_gray:
            await aio.wait_for(_mixed_passes(
                display, black_left, red_left, black_right, red_right, gray, waveform), 45)
    except BaseException:
        await _cleanup_error(display)
        raise
    if has_gray:
        await _finish(display)


async def _mixed_passes(display, black_left, red_left, black_right, red_right, gray, waveform):
    # Prepare only gray sites with 00; 11 holds every existing B/W/red pixel.
    _gray_hold_planes(black_left, red_left, gray, 0, HALF_BYTES, 0)
    _gray_hold_planes(black_right, red_right, gray, HALF_BYTES, HALF_BYTES, 0)
    await _start(display, black_left, red_left, black_right, red_right, waveform)
    for _ in range(6):
        await pull(display)
    _gray_hold_planes(black_left, red_left, gray, 0, HALF_BYTES, 1)
    _gray_hold_planes(black_right, red_right, gray, HALF_BYTES, HALF_BYTES, 1)
    display._send_images(black_left, red_left, black_right, red_right)
    for _ in range(3):
        await pull(display)
