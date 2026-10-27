# Debugging records

문제 하나당 문서 하나. 형식: 증상 → 가설 → 실험 → 원인 → 수정 → 회귀 테스트 → 배운 점.
근거(로그, 스크린샷)는 `NNN/` 폴더에 함께 보관한다.

| # | 제목 | 상태 | 한 줄 원인 |
|---|---|---|---|
| [001](001-slam-map-quality.md) | 시뮬레이션 LiDAR가 모든 빔에서 최소 거리를 반환 | 해결 | VM(Ogre1 + GL 2.1)의 gpu_lidar가 렌더 맨 위 행 깊이를 못 읽음 → 수직 1줄 LiDAR 전체 실패 |
| [002](002-pi-first-boot.md) | Pi 첫 부팅: SSH로 못 찾음 → 로그 수집 중 부팅 멈춤 → ROS 설치 의존성 오류 | 해결 | Pi 찾기 필터에 OUI `98:fe:54` 누락 + cloud-init instance-id/캐시 오해 + apt 소스에 `noble-updates` 없음 |
