# 002 — Pi 첫 부팅: SSH로 못 찾음 → 로그 수집 중 부팅 멈춤 → ROS 설치 의존성 오류

| 항목 | 내용 |
|---|---|
| 상태 | **해결** |
| 기간 | 2026-10-27 22:40 ~ 10-28 01:10 (약 2.5시간) |
| 환경 | Raspberry Pi 5 8GB, Ubuntu Server 24.04 (Imager 2.0 사용자 지정 = cloud-init NoCloud), cloud-init 26.1, Wi-Fi만 (랜선·micro-HDMI 없음) |
| 영향 범위 | Pi 초기 설치 전용. 하드웨어·Wi-Fi 설정·공유기는 처음부터 정상이었음 |

## 요약
문제 3개가 겹쳤다. 하드웨어 문제는 0개.
1. **Pi를 못 찾음** — 실제로는 첫 부팅부터 Wi-Fi에 붙어 있었다. Mac에서 `arp -a`를 Raspberry Pi 맥 주소 앞자리(OUI)로 걸렀는데, 이 Pi 5의 OUI `98:fe:54`가 목록에 없어서 매번 걸러졌다.
2. **로그를 남기려던 수정이 부팅을 멈춤** — (a) instance-id가 `cmdline.txt`(`ds=nocloud;i=...`)에도 있어 `meta-data`만 바꾼 건 무시됨, (b) 그걸 고친 뒤엔 내가 넣은 `bootcmd: systemd-run ...`이 부팅 초반(init-local)에 멈춤, (c) 같은 instance-id면 cloud-init은 SD의 user-data가 아니라 **저장해 둔 사본**을 다시 실행 → 지워도 또 멈춤.
3. **`setup_pi.sh` 3단계 ROS 설치 실패** — 이미지의 apt 소스에 `noble-updates`가 빠져 있음 → 설치된 라이브러리(updates 버전)와 `-dev` 패키지(release 버전)가 안 맞음.

---

## 1. 증상
- 첫 부팅: 초록 LED 깜빡임 → 고정(부팅 완료로 보임). 10분 넘게 기다려도 Mac에서 Pi가 안 보임.
- `ssh jasper@scr-pi.local` 불가 (avahi는 setup_pi.sh가 설치하므로 첫 부팅엔 원래 없음 → 예상된 실패).
- Mac 핑 스캔 + `arp -a | grep -iE "2c:cf:67|d8:3a:dd|dc:a6:32|e4:5f:01|28:cd:c1"` → 결과 없음 (여러 번, 재부팅 후에도).

## 2. 가설과 확인

| # | 가설 | 확인 | 결과 | 판정 |
|---|---|---|---|---|
| H1 | 아직 부팅 중 | 10분 대기, 버튼 종료 → 빨간 LED | OS는 정상 동작(전원 버튼에 반응해 정상 종료) | 기각 |
| H2 | Wi-Fi 이름/비밀번호 오타 | `system-boot/network-config` 확인 | SSID 일치, 비밀번호 일치 | 기각 |
| H3 | `network-config` 파일 깨짐(둥근 따옴표·탭) | `LC_ALL=C grep -n '[^ -~]'`, `cat -A` | ASCII만, 들여쓰기 정상 | 기각 |
| H4 | 첫 부팅에서만 Wi-Fi가 안 올라옴 | 재부팅 후 재스캔 | 여전히 안 보임 (사실은 필터 문제였음) | 기각 |
| H5 | Pi 안쪽 로그를 보자 → `runcmd`로 로그를 boot 파티션에 쓰기 | `meta-data` instance-id 변경 후 부팅 | 파일 안 생김 | → H6 |
| H6 | cloud-init이 다시 안 돌았다 | `cmdline.txt` 확인 | **`ds=nocloud;i=rpi-imager-...`** — 커널 인자의 instance-id가 우선 | 채택 |
| H7 | 리눅스 파티션 로그를 직접 읽자 | Mac `sudo dd if=/dev/rdisk13s2 ... count=2048` (앞 8GB) → Linux에서 `debugfs -c` | `/var/log/cloud-init*.log`, `status.json` 읽기 성공 | 결정적 |

### H7에서 나온 것
```text
cloud-init-output.log:  wlan0 ... Hw-Address 98:fe:54:xx:xx:xx   ← Pi Wi-Fi 맥 주소
23:27 의 arp -a:        (<Pi IP>) at 98:fe:54:xx:xx:xx      ← 처음부터 망에 있었음
status.json:            stage "init-local", finished null         ← 마지막 부팅이 init-local에서 멈춤
cloud-init.log 마지막:  Running module bootcmd ... Running command ['/bin/sh', '/run/cloud-init/tmp/...sh']
```
- `98:fe:54`는 내 OUI 필터 목록에 없었다 → **원인 1**.
- cloud-init 26.1 + NoCloud(dsmode=local)는 `bootcmd`를 **init-local** 단계(네트워크·D-Bus 이전)에서 실행한다. 여기서 `systemd-run --on-active=...`가 반환하지 않아 부팅이 멈춤 → 전원 버튼도 무반응 → **원인 2(b)**.
- 원인 2(c): `/var/lib/cloud/instance/cloud-config.txt`(캐시)를 읽는다. instance-id가 같으면 SD의 user-data를 고쳐도 반영 안 됨.

## 3. 원인
1. Pi 찾기 필터의 OUI 누락 (`98:fe:54`). 이 Pi 5의 Wi-Fi·이더넷 맥 주소가 이 대역.
2. cloud-init 동작에 대한 오해 3개: instance-id는 `cmdline.txt`가 우선 / `bootcmd`는 init-local에서 돈다 / 같은 instance-id면 캐시 사용.
3. Ubuntu Server 이미지(2026-09 빌드)의 `/etc/apt/sources.list.d/ubuntu.sources`가 `Suites: noble`만 있음 (`noble-updates`, `noble-backports` 없음). 왜 빠졌는지는 미확인 (Imager의 `apt: preserve_sources_list` 관련 추정, [가정]).

## 4. 수정
- 디버그용 `runcmd`/`bootcmd`를 user-data에서 제거 (원래 내용으로 복원) → instance-id를 새 값(`cmdline.txt`의 `i=`와 `meta-data` 둘 다)으로 바꿔 캐시를 버리게 함 → 정상 부팅, SSH 접속.
- apt 소스에 updates/backports 추가 후 `setup_pi.sh` 재실행 → 완료.
  ```bash
  sudo sed -i 's/^Suites: noble$/Suites: noble noble-updates noble-backports/' /etc/apt/sources.list.d/ubuntu.sources
  ```
- `setup_pi.sh` 1단계에 같은 검사·추가를 넣음 (다음에 SD를 새로 구워도 재발 안 함).

## 5. 회귀 테스트 / 결과
- `[Pi] hostname; date; uname -m; free -h; df -h /; cloud-init status` → `scr-pi`, KST 정확, aarch64, 7.8Gi, 117G, `status: done`
- `setup_pi.sh` → `=== 완료 ===`, colcon 6 패키지 56.7 s
- 재로그인 후 `groups` → `dialout` 포함, `ros2 pkg list | grep scr_` → 4개
- Pi 찾기 (Mac):
  ```bash
  NET=$(ipconfig getifaddr en0 | cut -d. -f1-3)   # 예: 192.168.0
  ( for i in $(seq 1 254); do ping -c1 -t1 $NET.$i >/dev/null 2>&1 & done; wait ); arp -a | grep -iE "98:fe:54|2c:cf:67|d8:3a:dd|dc:a6:32|e4:5f:01|28:cd:c1"
  ```
  (avahi 설치 후엔 `ssh jasper@scr-pi.local`)

## 6. 배운 점
- **찾는 도구가 틀리면 대상이 없는 것처럼 보인다.** 필터를 걸기 전에 필터 없는 원본(`arp -a` 전체)을 먼저 보고, 모르는 기기를 하나씩 지웠어야 했다. 원본에 답이 있었다.
- **화면 없는 장비를 디버깅할 땐 "로그를 꺼내는 길"부터 확보.** 우선순위: 유선(eth0 DHCP) → 모니터+키보드(Imager 비밀번호로 콘솔 로그인 가능) → SD의 ext4를 `dd` + `debugfs`로 읽기. 마지막 방법은 Pi를 건드리지 않고 사실을 그대로 볼 수 있어 결정적이었다.
- **로그 수집 장치가 시스템을 바꾸면 안 된다.** 진단용으로 넣은 `bootcmd`가 부팅을 멈춰 새 문제를 만들었다. 진단 코드는 부팅 경로 밖(`runcmd` 이후, 또는 읽기 전용 방법)에 둔다.
- cloud-init 메모: instance-id 우선순위 = 커널 인자(`cmdline.txt`의 `i=`) > `meta-data`. 같은 instance-id면 캐시 사용. `bootcmd`는 매 부팅, 그리고 init-local에서 돌 수 있다.
- 남은 일: rosdep이 Pi에 `ros-jazzy-turtlebot3-gazebo`를 설치함(어떤 package.xml 의존성) → Pi에선 불필요, 정리 대상.
