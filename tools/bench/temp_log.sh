#!/bin/bash
# [Pi] 11-26 오래 켜 둘 때 온도·스로틀·CPU 기록 (10 초마다) → ~/scr_logs/<날짜>/temp_<시각>.csv
#   tmux new -d -s temp "bash ~/semantic-companion-robot/tools/bench/temp_log.sh"   끄기: tmux kill-session -t temp
#   열: 시각, 온도 °C, 스로틀 (0x0 = 한 번도 안 걸림), 1 분 부하, Pi 전력 W (PMIC 합)
D=~/scr_logs/$(date +%Y-%m-%d); mkdir -p "$D"; F="$D/temp_$(date +%H%M).csv"
echo "t,temp_c,throttled,load1,watt" > "$F"; echo "기록: $F"
while true; do
  T=$(vcgencmd measure_temp | grep -oE '[0-9.]+'); TH=$(vcgencmd get_throttled | cut -d= -f2)
  L=$(cut -d' ' -f1 /proc/loadavg)
  W=$(vcgencmd pmic_read_adc 2>/dev/null | python3 -c "import re,sys; t=sys.stdin.read(); a={k:float(x) for k,x in re.findall(r'(\S+)_A \w+\(\d+\)=([\d.]+)',t)}; v={k:float(x) for k,x in re.findall(r'(\S+)_V \w+\(\d+\)=([\d.]+)',t)}; print(round(sum(a[k]*v[k] for k in a if k in v),2))")
  echo "$(date +%H:%M:%S),$T,$TH,$L,$W" >> "$F"; sleep 10
done
