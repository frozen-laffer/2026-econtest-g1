#!/bin/bash
# 开机自启动入口：等 Wayland 就绪 → 关掉残留实例 → 常亮屏幕 → 起界面。
# 由 ~/.config/autostart/lsa-touch.desktop 调用；也可手动执行。
LSA_DIR=/home/frozen-laffer/lsa
cd "$LSA_DIR" || exit 1

export DISPLAY=:0
export XAUTHORITY=/home/frozen-laffer/.Xauthority
export WAYLAND_DISPLAY=wayland-0
export XDG_RUNTIME_DIR=/run/user/1000
export PYTHONUNBUFFERED=1

LOG=/tmp/lsa_touch.log

# 1) 等 Wayland socket：开机时 compositor 可能还没起来
for _i in $(seq 1 60); do
    [ -S "$XDG_RUNTIME_DIR/$WAYLAND_DISPLAY" ] && break
    sleep 1
done
sleep 2

# 2) 板子只允许一条 TCP 连接，先把残留的旧界面清掉
for p in $(pgrep -f lsa_touch.py); do
    kill "$p" 2>/dev/null
done
sleep 1

# 3) 屏幕常亮（比赛现场不能黑屏）
wlopm --on '*' >>"$LOG" 2>&1

# 4) 起主程序（每次启动前把日志分段，避免无限增长）
if [ -f "$LOG" ] && [ "$(stat -c%s "$LOG")" -gt 1000000 ]; then
    mv -f "$LOG" "$LOG.1"
fi
echo "=== boot_start $(date '+%F %T') ===" >>"$LOG"
exec python3 lsa_touch.py >>"$LOG" 2>&1
