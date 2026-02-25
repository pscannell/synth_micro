#!/usr/bin/env python3
import argparse
import collections
import curses
import socket
import time


def parse_stat(line: str):
    state = {}
    payload = line[len("STAT ") :].strip()
    for token in payload.split():
        if "=" not in token:
            continue
        k, v = token.split("=", 1)
        state[k] = v
    return state


def parse_knobs(raw: str):
    try:
        vals = [int(x) for x in raw.split(",")]
        if len(vals) == 6:
            return vals
    except ValueError:
        pass
    return [0, 0, 0, 0, 0, 0]


def keys_from_hex(raw: str):
    try:
        bits = int(raw, 16)
    except ValueError:
        bits = 0
    return [i for i in range(28) if (bits >> i) & 1], bits


def run_ui(stdscr, host: str, port: int):
    curses.curs_set(1)
    stdscr.nodelay(True)
    stdscr.timeout(40)

    sock = socket.create_connection((host, port), timeout=5)
    sock.setblocking(False)

    buf = b""
    logs = collections.deque(maxlen=8)
    last = {"knobs": "0,0,0,0,0,0", "keys": "0", "enc": "0", "encbtn": "0", "foot": "0", "led": "0"}
    input_line = ""
    last_rx = time.time()

    def send_cmd(cmd: str):
        nonlocal logs
        sock.sendall((cmd.strip() + "\n").encode("utf-8"))
        logs.appendleft(f"> {cmd.strip()}")

    send_cmd("help")
    send_cmd("dump")

    while True:
        ch = stdscr.getch()
        if ch != -1:
            if ch in (ord("q"), 27):
                break
            if ch in (curses.KEY_BACKSPACE, 127, 8):
                input_line = input_line[:-1]
            elif ch in (10, 13):
                if input_line.strip():
                    send_cmd(input_line)
                input_line = ""
            elif 32 <= ch <= 126:
                input_line += chr(ch)
            elif ch in (ord("d"),):
                send_cmd("dump")
            elif ch in (ord("h"),):
                send_cmd("help")
            elif ord("0") <= ch <= ord("7"):
                send_cmd(f"led {chr(ch)}")

        try:
            chunk = sock.recv(4096)
            if not chunk:
                logs.appendleft("Disconnected by RTT server")
                break
            last_rx = time.time()
            buf += chunk
            while b"\n" in buf:
                raw_line, buf = buf.split(b"\n", 1)
                line = raw_line.decode("utf-8", errors="replace").strip()
                if not line:
                    continue
                if line.startswith("STAT "):
                    last.update(parse_stat(line))
                else:
                    logs.appendleft(line)
        except BlockingIOError:
            pass

        stdscr.erase()
        height, width = stdscr.getmaxyx()

        knobs = parse_knobs(last.get("knobs", "0,0,0,0,0,0"))
        keys_pressed, key_bits = keys_from_hex(last.get("keys", "0"))
        age_ms = int((time.time() - last_rx) * 1000)

        stdscr.addnstr(0, 0, "STM32 RTT Monitor (q to quit)", width - 1)
        stdscr.addnstr(1, 0, f"RTT: {host}:{port}   last_rx={age_ms}ms", width - 1)
        stdscr.addnstr(3, 0, f"Knobs: {knobs}", width - 1)
        stdscr.addnstr(
            4,
            0,
            f"Encoder: {last.get('enc', '0')}   Button: {last.get('encbtn', '0')}   Foot: {last.get('foot', '0')}",
            width - 1,
        )
        stdscr.addnstr(5, 0, f"LED State: {last.get('led', '0')}", width - 1)
        stdscr.addnstr(6, 0, f"Keys bits: 0x{key_bits:08X}", width - 1)
        stdscr.addnstr(7, 0, f"Pressed keys: {keys_pressed}", width - 1)
        stdscr.addnstr(9, 0, "Logs:", width - 1)

        row = 10
        for log in logs:
            if row >= height - 2:
                break
            stdscr.addnstr(row, 0, log, width - 1)
            row += 1

        stdscr.addnstr(height - 1, 0, f"cmd> {input_line}", width - 1)
        stdscr.move(height - 1, min(width - 1, 5 + len(input_line)))
        stdscr.refresh()

    sock.close()


def main():
    parser = argparse.ArgumentParser(description="RTT telemetry UI for synth_micro")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=9090)
    args = parser.parse_args()
    curses.wrapper(run_ui, args.host, args.port)


if __name__ == "__main__":
    main()
