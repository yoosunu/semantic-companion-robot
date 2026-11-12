# 화면에 보이는 사람 한 명 = 트랙 하나. face_perception 이 냄.

uint32 track_id     # 추적 번호. 계속 보이는 동안 같은 번호, 놓치면(1.5 s) 다음에 새 번호
string name         # 알아본 이름. 모르면 ""
float32 name_conf   # 이름 확신 0~1 (최근 투표 중 그 이름 비율)
uint16 votes        # 지금까지 인식 투표 수

float32 pan         # 사람 방향 (도). /pantilt/cmd 와 같은 좌표 → 이 값을 그대로 보내면 쳐다봄
float32 tilt
float32 face_px     # 얼굴 폭 (320 폭 화면 기준 px). 클수록 가깝고 인식 잘 됨 (15 px 아래는 약함, 11-12 실측)
float32 det_score   # YuNet 검출 점수 0~1

float32 yaw         # 얼굴이 돌아간 정도 (도, 0 = 정면, + = 사람 기준 왼쪽) — 점 5개로 추정, 대략값
float32 pitch       # 위아래 (도, + = 위)

float32 age_s       # 트랙이 생긴 지 몇 초
float32 unseen_s    # 마지막으로 본 지 몇 초 (0 = 이번 영상에 있음)
