#!/usr/bin/env python3
"""Nav2 원본 설정 위에 내 손잡이 파일을 덮어써서 실제로 쓸 설정 파일을 만든다.

사용: merge_params.py <원본.yaml> <my.yaml> <결과.yaml>   (robot_nav.sh가 자동으로 호출)
- my.yaml 에 적은 값만 바뀌고, 나머지는 원본 그대로.
- 바뀐 값을 "원본 → 새 값"으로 출력한다 (nav2 창 맨 위에서 확인).
- 원본에 없는 키는 경고한다: 오타면 Nav2는 조용히 무시하기 때문 ("바꿨는데 왜 안 되지?"의 흔한 원인).
"""
import sys
import yaml


def merge(base, over, path, changes, unknown):
    for key, value in over.items():
        p = f"{path}.{key}" if path else str(key)
        if value is None:   # 하위 줄이 전부 주석인 섹션 (예: "goal_checker:" 만 남음) → 변경 없음
            continue
        if key not in base:
            unknown.append(p)
            base[key] = value
        elif isinstance(value, dict) and isinstance(base[key], dict):
            merge(base[key], value, p, changes, unknown)
        else:
            if base[key] != value:
                changes.append((p, base[key], value))
            base[key] = value


def main():
    base_path, my_path, out_path = sys.argv[1:4]
    base = yaml.safe_load(open(base_path))
    mine = yaml.safe_load(open(my_path)) or {}   # 전부 주석이면 None → 변경 없음
    if not isinstance(mine, dict):
        sys.exit(f"[설정] 형식 오류: {my_path} 의 최상위는 노드 이름(controller_server: ...)이어야 함")

    changes, unknown = [], []
    merge(base, mine, "", changes, unknown)

    print(f"[설정] {my_path}")
    if not changes and not unknown:
        print("[설정]   변경 없음 (원본 그대로)")
    for p, old, new in changes:
        print(f"[설정]   {p.replace('.ros__parameters', '')}: {old} → {new}")
    for p in unknown:
        print(f"[설정]   ⚠ 원본에 없는 키: {p.replace('.ros__parameters', '')}  ← 오타인지 확인 (오타면 Nav2가 무시함)")

    with open(out_path, "w") as f:
        f.write(f"# 자동 생성: {base_path} + {my_path}. 직접 고치지 말 것.\n")
        yaml.safe_dump(base, f, sort_keys=False, allow_unicode=True)


if __name__ == "__main__":
    main()
