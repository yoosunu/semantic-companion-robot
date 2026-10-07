# 하드웨어 기준 아키텍처 v1 (2026-10-01)

## 목적
ROS2 기반 실제 모바일 로봇에서 LiDAR SLAM, Nav2, TF/odometry, 센서 통합, iPhone 기반 Vision/AR 연동을 단계적으로 구현한다.

## 1차 확정 구성
| No. | 품목 | 수량 | 역할/비고 | 상태 |
|---|---|---:|---|---|
| 1 | Waveshare UGV02 | 1 | 차체/모터/ESP32/IMU/엔코더/전원관리 포함 | 확정 |
| 2 | Raspberry Pi 5 8GB | 1 | Ubuntu 24.04 + ROS2 Jazzy 중앙 컴퓨터 | 확정 |
| 3 | RPLIDAR C1 Development Kit | 1 | 2D 360° LiDAR, USB Adapter 포함 | 확정 |
| 4 | Raspberry Pi 5 Active Cooler | 1 | Pi 5 지속 부하 냉각 | 확정 |
| 5 | Raspberry Pi 27W USB-C PSU | 1 | 초기 세팅/벤치 개발용 | 확정 |
| 6 | microSD 128GB A2 | 1 | Ubuntu/ROS2 및 초기 rosbag 저장 | 확정 |
| 7 | 18650 배터리 | 3 | UGV02 3S UPS용. 증정 시 별도 구매 불필요 | 조건부 |
| 8 | LiDAR/iPhone 거치대·케이블류 | 1식 | 실물 치수 확인 후 구매/제작 | 후속 |

## 아키텍처
```text
iPhone
  └─ Camera / ARKit / CoreML Vision
          │
          ▼
Raspberry Pi 5 8GB
  └─ Ubuntu 24.04 LTS + ROS2 Jazzy
      ├─ RPLIDAR C1 → /scan
      ├─ TF / Odometry
      ├─ SLAM Toolbox
      ├─ Nav2
      ├─ Task / Safety Manager
      └─ rosbag / logging
          │ UART / JSON
          ▼
UGV02 내장 ESP32
  └─ Encoder / IMU / Motor closed-loop control
          ▼
        Motors
```

## 호환성 검증 결론
- UGV02는 ROS2 모바일 로봇 베이스로 사용 가능.
- Pi ↔ UGV02는 UART/JSON으로 연결, 인터넷 없이 핵심 제어 가능.
- RPLIDAR C1 Dev Kit은 USB Adapter로 Pi에 직접 연결.
- UGV02 skid-steer의 odometry slip은 LiDAR SLAM/IMU/ARKit VIO와 비교·보정하는 프로젝트 요소로 활용.
- LiDAR 360° scan plane을 가리지 않도록 iPhone/LiDAR 거치 구조는 실물 수령 후 설계.

## 지금은 사지 않는 것
별도 ESP32 · 별도 Motor Driver · 5V/5A Buck Converter(필요 시 후속) · NVMe SSD · Jetson

## 개발 전제
- Ubuntu 24.04 LTS · ROS 2 Jazzy · Gazebo Harmonic · SLAM Toolbox 우선 · Nav2
- 직접 작성: UGV02 ROS2 bridge/hardware interface, Task Manager, Safety/Decision layer, semantic world model, iPhone 연동
- 핵심 이동/안전 기능은 인터넷이나 Mac 없이 Pi + LiDAR + UGV02만으로 동작

## 열린 이슈
- 주행 중 Pi 5 전원: Pi 5는 5V/5A 기준. 전원이 약하면 USB 총 출력 600mA 제한 → LiDAR 불안정 가능.
  UGV02 5V 출력 전류 용량 확인 필요, 부족하면 5V/5A 벅 컨버터 추가.
