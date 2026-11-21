# -*- coding: utf-8 -*-
"""AFG-2225 的 TCP 中继：PC 上有 COM17，树莓派上没有可用串口链路时用它。

为什么需要这个：AFG-2225 的 USB 接口是 CDC 类、但子类是 0x20（非标准 ACM），
在 Linux 上 cdc_acm 会绑定成 /dev/ttyACM0，可是**只进不出** —— 发命令没反应、
面板也不变。Windows 上用厂家驱动没问题。所以让 PC 继续持有串口，把 SCPI
通过 TCP 转给树莓派。

协议（一行一条，简单到不需要文档）：
    客户端 -> 服务端：  <SCPI 命令>\\n
    服务端 -> 客户端：  仪器回的一行行文本，最后发一行 *EOT* 作为结束标记
                      （写命令没有回读时，就只发 *EOT*）

用法：
    python afg_bridge.py                 # 默认 COM17，监听 0.0.0.0:5061
    python afg_bridge.py --port COM17 --listen 5061 --baud 115200
"""
import argparse
import socket
import threading
import time

QUIET = 0.8          # 一条命令后最多再等这么久收集回读
EOT = "*EOT*"


def serve_client(conn, addr, ser, lock):
    print("[bridge] client %s:%d" % addr)
    buf = b""
    try:
        while True:
            chunk = conn.recv(4096)
            if not chunk:
                break
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                cmd = line.decode("ascii", "replace").strip()
                if not cmd:
                    continue
                out = []
                with lock:
                    try:
                        ser.reset_input_buffer()
                        ser.write((cmd + "\n").encode("ascii"))
                        ser.flush()
                    except Exception as exc:
                        out.append("ERR %r" % (exc,))
                    else:
                        t0 = time.time()
                        while time.time() - t0 < QUIET:
                            ln = ser.readline()
                            if not ln:
                                if out:
                                    break
                                continue
                            txt = ln.decode("ascii", "replace").strip()
                            if txt:
                                out.append(txt)
                conn.sendall(("\n".join(out) + "\n" + EOT + "\n").encode("ascii"))
    except Exception as exc:
        print("[bridge] client error: %r" % (exc,))
    finally:
        try:
            conn.close()
        except Exception:
            pass
        print("[bridge] client %s:%d closed" % addr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM17", help="串口名，默认 COM17")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--listen", type=int, default=5061)
    args = ap.parse_args()

    import serial
    ser = serial.Serial(args.port, args.baud, timeout=0.25, write_timeout=2.0)
    time.sleep(0.3)
    ser.reset_input_buffer()
    print("[bridge] serial %s @ %d open" % (args.port, args.baud))

    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", args.listen))
    srv.listen(4)
    print("[bridge] listening on 0.0.0.0:%d" % args.listen)

    lock = threading.Lock()
    try:
        while True:
            conn, addr = srv.accept()
            conn.settimeout(None)
            threading.Thread(target=serve_client, args=(conn, addr, ser, lock),
                             daemon=True).start()
    except KeyboardInterrupt:
        pass
    finally:
        srv.close()
        ser.close()


if __name__ == "__main__":
    main()
