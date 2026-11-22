# sim — 바퀴 속도 제어 SIL 시뮬레이션 (노트북)

> 공학프로그래밍 텀프로젝트 진행표 5 (`docs/TERM_PROJECT.md §6`, main 브랜치).
> 🚧 작성 중 — 2단계 뼈대(빈칸 A·B·C)까지.

## 무엇을 하나

`firmware/term_drive_v2/` 의 제어 클래스(`Wheel` · `WheelSpeedController` · `PIDController` · `DriveBase`)를
**한 글자도 고치지 않고** `#include` 해서 노트북에서 20 ms 주기로 돌린다.
실물이 없는 모터·바퀴·엔코더만 수식(플랜트 모델)으로 대신한다 = SIL(Software-In-the-Loop).

```
목표 속도 ─▶ Wheel::setTarget ─▶ updateController ─▶ updateOutput ─▶ PWM
                                                                       │
   Wheel::measure(엔코더 누적 칸 수) ◀── 엔코더 양자화 ◀── 플랜트 모델(PWM → 바퀴 속도) ◀┘
```

## 검증하는 것 / 못 하는 것

| | 내용 |
|---|---|
| ✅ 검증한다 | 코드가 **손계산(이론)대로** 계산하는가 — FF 값 · 필터 시정수 · 적분 속도 · 부스트/램프 순서 |
| ❌ 못 한다 | 실제 로봇이 이렇게 움직인다는 것 — 플랜트 모델은 **가정**이다. 실차 비교(진행표 6·7)가 맡는다 |

## 플랜트 모델의 숫자 — 출처

| 숫자 | 값 | 출처 | 상태 |
|---|---|---|---|
| 모터 시정수 | 0.10 s (가정) | 기존 bag(`air_0812_*`)은 `/drive/diag` 가 1 Hz 뿐이라 뽑을 수 없음 | **가정값으로 시작 → 실차에서 측정 후 교체** |
| PWM → 정상상태 속도 | v = 0.003691 × (PWM − 30.99) | 지면 실측 3점 직선 맞춤 (main `docs/MASTER_PLAN.md` FF 산출 절) | ⚠ 맞춘 구간 PWM 54.6~66.5 뿐 — 145 PWM 에서 0.42 예측 vs 실측 0.33 |

## 진행

| 단계 | 내용 | 상태 |
|---|---|---|
| 1 | 손계산 정답지 (FF · 엔코더 1칸 거리 · 20 ms 칸 수 · 필터 시정수 · PI) | 🟡 계산 완료(11-22), 정답지 정리 남음 |
| 2 | `sim_main.cpp` — 20 ms 반복문 + 플랜트 모델 + CSV 출력 | 🟡 뼈대 커밋, 빈칸 A·B(`motor_model.h`) · C(`sim_main.cpp`) 남음 |
| 3 | 시뮬 출력 ↔ 손계산 대조 | ⏳ |
| 4 | 실험 4종: 스텝 응답 · Kp 30 vs 60 · 와인드업 · 필터 (예측 먼저 기록) + matplotlib 그래프 | ⏳ |
| 5 | 실차 스텝 응답과 겹쳐 보기 → 시정수 교체 | ⏳ (굽기 후) |

## 파일

| 파일 | 내용 |
|---|---|
| `sim_config.h` | 펌웨어 `.ino` 설정값 사본(줄 번호 표기). 🔴 `.ino` 를 바꾸면 같이 바꾼다 |
| `motor_model.h` | 가짜 모터·바퀴·엔코더 (가정) |
| `sim_main.cpp` | 20 ms 반복문 → CSV |

빌드: `cd sim && mkdir -p build && g++ -std=c++11 -Wall -Wextra -I ../firmware/term_drive_v2 sim_main.cpp -o build/sim && ./build/sim > build/step.csv`

## 작업 방식

결과를 정하는 부분(손계산 · 반복문 작성 · 모델 숫자 선택 · 실험 예측)은 사람이 직접 하고,
결과에 영향이 없는 도구(그래프 스크립트)와 설명·리뷰는 AI 도구(Claude)의 도움을 받는다.
실험마다 "예측 → 결과 → 이유" 를 기록한다.

## 쓰지 않는 표현

모터 = 브러시 DC + MDD10A (BLDC 아님) · 제어 = 피드포워드 + PI (D 항 구현돼 있으나 꺼짐, "PID 제어" 아님).
