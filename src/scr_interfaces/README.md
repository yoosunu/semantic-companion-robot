# scr_interfaces

NERI 노드끼리 주고받는 **메시지 모양**만 정의하는 패키지. 코드는 없음.

- `.msg` 파일 = "이 토픽엔 이런 칸들이 있다"는 약속. 빌드하면 C++ 헤더와 Python 클래스가 자동으로 생김.
- 그래서 Python 노드에서 `from scr_interfaces.msg import People` 처럼 씀.
- 칸을 바꾸면 **다시 빌드**해야 하고, 그 메시지를 쓰는 노드도 다시 켜야 함.

| 파일 | 쓰는 곳 |
|---|---|
| `msg/Person.msg` | 사람 한 명 (트랙 번호, 이름, 방향, 얼굴 크기 …) |
| `msg/People.msg` | `/people` 토픽: 지금 보이는 사람 전부 |

나중에 추가: `action/FaceEnroll.action` (등록 절차), `srv/FaceForget.srv` (지우기).

확인: `ros2 interface show scr_interfaces/msg/People`
