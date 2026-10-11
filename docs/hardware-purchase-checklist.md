# 하드웨어 구매 기준서 (v1, 2026-10-11)

목적: 되돌릴 수 없는 구매 전에, 각 품목이 **프로젝트 요구사항을 만족하는지**와 **결제 직전에 무엇을 확인할지**를 한 곳에 정리한다.
교차 검증(다른 AI·판매처 문의)용. 기준 아키텍처: [hardware-baseline.md](hardware-baseline.md).

> 표기: ✅ 공식 자료로 확인 / ⚠️ 공식 자료에 없음·불일치 → 구매 전 확인 필요 / ❓ 미확인

---

## 0. 프로젝트 요구사항 (구매 판단의 기준)
| # | 요구사항 | 이유 |
|---|---|---|
| R1 | Pi 5에서 Ubuntu 24.04 + ROS 2 Jazzy 구동 | 시뮬레이션 v0.1과 같은 스택 유지 |
| R2 | 핵심 이동·안전 기능은 인터넷·Mac 없이 Pi + LiDAR + 베이스만으로 동작 | 설계 원칙 |
| R3 | 베이스를 ROS 2에서 속도 명령으로 제어하고, 엔코더·IMU 값을 받을 수 있음 | `/cmd_vel` → 모터, 엔코더/IMU → `/odom`, `/imu` |
| R4 | 360° 2D LiDAR가 ROS 2 `/scan`을 발행 | SLAM·Nav2 입력 |
| R5 | 배터리로 Pi 5 + LiDAR + 모터를 동시에 안정적으로 구동 | 주행 중 전원 문제 방지 |
| R6 | 상단에 LiDAR와 iPhone을 장착할 공간·레일 | 센서 거치 |

## 1. 품목별 기준

### 1-1. Waveshare UGV02 (6x4 Off-Road UGV, 온보드 ESP32)
| 항목 | 공식 자료 | 판정 | 프로젝트 영향 |
|---|---|---|---|
| 호스트 통신 | 40핀 UART(기본 115200), JSON 명령. 속도 명령 T:13 (선속도 m/s, 각속도 rad/s) | ✅ R3 | 내가 작성할 C++ 브리지의 프로토콜 |
| 안전 | 3초간 이동 명령이 없으면 자동 정지(heartbeat) | ✅ R2 | 브리지가 주기적으로 명령을 보내야 함 |
| 속도 범위 | 속도 명령 -0.5 ~ +0.5 m/s | ✅ | Nav2 최대 속도 상한 |
| 구동 | 6륜 4WD, JGB37-520 12V 감속모터(1:30, 정격 250RPM, 정격 ≤1A) | ✅ | 스키드 스티어 → 회전 시 미끄러짐으로 odom 오차 큼 |
| 엔코더 | 있음. **FAQ상 앞바퀴 엔코더만 읽음** | ⚠️ | odom 정밀도 한계 → LiDAR SLAM/IMU 보정이 중요해짐 (프로젝트 소재) |
| IMU | 9축, 모델명 미표기 | ✅/❓ | `/imu` |
| ROS | ROS 드라이버 저장소(ugv_base_ros) 링크. **ROS 2 언급 없음** | ⚠️ | ROS 2 브리지는 직접 작성 (원래 계획과 일치) |
| 테스트된 호스트 | Raspberry Pi 5, Pi 4B (Jetson Nano/Orin Nano 연결 옵션) | ✅ R1 | |
| 배터리 | 18650 3직렬(3S), 2200mAh 이상·4C 방전 권장. **"리튬 배터리 미포함"**, 12.6V 2A 충전기 포함 | ⚠️ R5 | 견적서의 "판매처 증정 시 불필요"와 충돌 → 판매처 구성 확인 필수 |
| 호스트용 전원 출력 | UPS 모듈에서 5V, 3.3V 출력. **전류 용량 미표기** | ⚠️ R5 | Pi 5를 이 5V로 구동할 수 있는지 불명 → 2절 참고 |
| 장착 | 1020 알루미늄 레일 2개(간격 203mm) + 너트·볼트 | ✅ R6 | LiDAR·iPhone 거치대 설계 기준 |

### 1-2. Raspberry Pi 5 8GB
| 항목 | 기준 | 판정 |
|---|---|---|
| 메모리 | 8GB (SLAM + Nav2 + 브리지 + rosbag 여유) | ✅ 요구 |
| OS | Ubuntu 24.04 공식 이미지 지원 | ❓ 설치 시 확인 |
| 정품 | 공식 유통사 / 정상 가격대 | ⚠️ 국내 가격 편차 큼 |
| 전원 | 공식 27W(5V 5A, USB-PD) 어댑터 = 벤치 개발용 | ✅ |
| 냉각 | Active Cooler (SLAM·Nav2 지속 부하) | ✅ |

### 1-3. RPLIDAR C1 (Development Kit)
| 항목 | 공식 자료 | 판정 |
|---|---|---|
| 구성품 | C1 본체, **어댑터**, USB Type-A ↔ Type-C 케이블 | ✅ (어댑터가 USB 변환인지 문구상 불명확 → ⚠️ 상품 사진/문의로 확인) |
| 전원 | 5V, 동작 전류 약 230mA (1.15W) | ✅ |
| 크기·무게 | 55.6 × 55.6 × 41.3mm, 110g | ✅ |
| 성능 | 0.05~12m(흰색)/6m(검은색), ±30mm, 10Hz, 0.72° | ✅ R4 |
| 인터페이스 | UART(TTL) 460800 | ✅ |
| ROS 2 드라이버 | `sllidar_ros2`에 C1 지원 + `view_sllidar_c1_launch.py`. **Jazzy는 문서에 명시 없음** | ⚠️ 소스 빌드로 확인 예정 |

### 1-4. 저장장치·기타
| 품목 | 기준 |
|---|---|
| microSD 128GB A2 | 정품, A2 등급 (OS + rosbag) |
| 18650 × 3 | UGV02에 미포함일 경우: **보호회로 고려된 정품 셀, 2200mAh 이상, 4C 이상, 같은 제품 3개** |

## 2. 전원 판단 (가장 중요한 미확인 사항)
- Pi 5는 USB-PD로 5V 5A를 협상하지 못하면 USB 포트 총 전류를 **600mA로 제한**한다 (5A 협상 시 1.6A). `usb_max_current_enable=1`로 수동 해제 가능. (Raspberry Pi 공식 백서)
- 우리 구성에서 Pi USB로 전원을 받는 장치는 **LiDAR(약 230mA)뿐** → 600mA 제한이 걸려도 LiDAR 자체는 범위 안.
- 진짜 위험은 **Pi 5 본체 부하(최대 약 25W)를 UGV02의 5V 출력이 감당하는지**. 공식 자료에 전류 용량이 없음.
- 대응:
  1. 개발 초기: 공식 27W 어댑터로 벤치 작업 (주행 없이 bring-up)
  2. 판매처/Waveshare에 5V 출력 전류 용량 문의
  3. 부족하거나 불명이면: 3S 배터리(9~12.6V) → **5V 5A 강압(벅) 컨버터**를 별도 추가. 실물 수령 후 부하 측정으로 결정해도 늦지 않음.

## 3. 결제 직전 체크리스트
- [ ] UGV02: **18650 배터리 포함 여부** (공식: 미포함). 미포함이면 셀 3개 별도 구매
- [ ] UGV02: 상품명이 "UGV02 (6x4 Off-Road, ESP32)"이고 레일·충전기 포함 구성인지
- [ ] UGV02: (가능하면) 5V 출력 전류 용량 문의
- [ ] RPLIDAR C1: **Development Kit(어댑터 + USB 케이블 포함)** 구성인지
- [ ] Raspberry Pi 5 **8GB**인지(4GB 아님), 정품 유통, 정상 가격대
- [ ] Active Cooler가 **Pi 5 전용**인지
- [ ] 27W 어댑터가 **Pi 5 공식(5.1V 5A, USB-PD)** 인지
- [ ] microSD 128GB **A2**, 정품

## 4. 교차 검증 질문 (다른 AI·판매처용)
1. UGV02의 5V 출력으로 Raspberry Pi 5를 주행 중 안정적으로 구동한 사례나 전류 사양이 있는가?
2. UGV02가 앞바퀴 엔코더만 읽을 때, ROS 2 odom 품질에 어떤 한계가 있는가?
3. `sllidar_ros2`를 ROS 2 Jazzy(Ubuntu 24.04, ARM64)에서 빌드·구동한 사례가 있는가?
4. 이 구성에서 빠진 필수 부품(케이블, 스페이서, 전원 분배 등)이 있는가?

## 출처
- Waveshare UGV02 Wiki: https://www.waveshare.com/wiki/UGV02
- Waveshare RPLIDAR C1 상품 페이지: https://www.waveshare.com/product/robotics/drivers-sensors/lidar/rplidar-c1.htm
- RPLIDAR C1 Wiki: https://waveshare.com/wiki/RPLIDAR_C1
- Slamtec sllidar_ros2: https://github.com/Slamtec/sllidar_ros2
- Raspberry Pi, "USB power delivery on Raspberry Pi 5" (RP-009856-WP): https://pip-assets.raspberrypi.com/categories/685-app-notes-guides-whitepapers/documents/RP-009856-WP-1-USB%20Power%20delivery%20on%20Raspberry%20Pi%205.pdf
