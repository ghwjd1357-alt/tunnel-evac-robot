# CURRENT_HANDOFF.md — 지금 할 한 묶음

> 현재 묶음 + 미해결 보류만 유지한다. 구현자의 완료 주장은 독립검토 판정이 아니다.
> 텀프로젝트의 고정 사실·역할·일정·저장소 상태·열린 위험 = **`docs/TERM_PROJECT.md`** (먼저 읽는다).

- **프로젝트 상태**: 🏁 한이음 종료(태그 `hanium-final`) → 학교 텀프로젝트로 재활용 중.
  §95 범위 확정은 끝났다(결과 = `docs/TERM_PROJECT.md §1~§5`).
- **현재 단계**: §95 텀프로젝트 범위 확정(✅) → **§96 보고서 초안 리뷰**

## 이번 한 묶음 목표 — 보고서 초안 리뷰 (§96)

팀 회의(11-23) 전에 사용자가 먼저 초안을 리뷰한다. Claude 는 리뷰를 **돕는다** — 사용자가 이해하고
판단하도록 설명·근거 확인을 하고, 고칠지는 사용자가 정한다.

- 초안 = https://claude.ai/artifact/VRBQoznEJHHgAe8LzEC7ac (Claude Docs 공유 문서)
- 진행: 장 하나씩. 읽기는 해당 장만(문서 전체를 한 번에 읽지 않는다 — 토큰).

## 완료조건

| # | 항목 | 통과선 |
|---|---|---|
| 1 | 이해 | 사용자가 각 장 내용을 자기 말로 설명할 수 있다 |
| 2 | 사실 오류 | 수치·이름·동작을 코드/실측 근거와 대조 — 틀린 곳은 근거와 함께 표시 |
| 3 | 버릴 것 / 더할 것 | 장마다 후보 목록 (결정은 사용자·팀) |
| 4 | 과장·거짓 | 확인 안 된 것을 된 것처럼 쓴 문장 제거 (예: 실차 미검증인데 "검증했다") |
| 5 | 검증 장(6장)의 보고서 적합성 | 974,258 단계 비교·변이 검사·FMA 를 교수님이 읽을 수준으로 쓸 수 있는지 판단 |

## 완료 판정

5행 모두에 대해 사용자가 결론을 냈고, 회의에 가져갈 수정 목록이 문서 댓글 또는 본문에 남았으면 끝난다.

## 금지 범위

- 🔴 문서를 Claude 가 **임의로 크게 고치지 않는다** — 리뷰 단계다. 고치는 것은 사용자가 승인한 것만.
- 🔴 `term/class-refactor` 를 main 에 합치지 않는다.
- 🔴 **보고서·발표에 쓰지 않는 표현** — 관제 지도를 *"실측 그대로"*, 관제 MOCK 4곳(카메라·탐지목록·
  배터리·Jetson 부하)을 *"실측"*, *"카메라로 사람을 인식"*·*"새로운 알고리즘"* (`tools/viz/README.md`),
  *"E-stop 을 검증했다"*, 모터 *"BLDC"*, 제어 *"PID 제어"*(실제는 피드포워드 + PI).
- 🔴 **`TODO(D+0)` 11건을 추측으로 다시 열거나 닫지 않는다** — 종결 계약이다(`JETSON_SETUP §9`).
- 로봇 안전·자산 규칙(`MAX_*_CMD` · Nav2/EKF/URDF 원본 · 지도 자산 등)은 한이음 판 그대로 —
  `docs/legacy/hanium/CURRENT_HANDOFF_hanium_final.md` 금지 범위.

## 최종 회귀

- 기준선(11-15 재측정 · 이 묶음은 `src/**` 변경 없음):
  pytest **285 passed** / adapter **123** / tools **386 + skip 1** /
  colcon **477 tests, 0 fail, 3 skip** / **test_harness_guards 24 검사** /
  test_gate_regression **14 케이스**.

## 완료 후 다음 단계

`docs/TERM_PROJECT.md §6` 진행표 5번(노트북 SIL 시뮬 + 그래프)부터.

| 트랙 | 상태 | 진행 |
|---|---|---|
| **진행 중 묶음** | §96 — 보고서 초안 리뷰 | 📋 착수 |
| 텀프로젝트 총괄 | `docs/TERM_PROJECT.md` | 진행표 = §6 |
| 한이음 | 🏁 종료 · 태그 `hanium-final` | ✅ |

## 근거 문서

- 총괄 = `docs/TERM_PROJECT.md` · 코드 구조 = `docs/STRUCTURE_GUIDE.md`.
- 한이음 마지막 핸드오프 전문 = `docs/legacy/hanium/CURRENT_HANDOFF_hanium_final.md`.
- 클래스 펌웨어 = 브랜치 `term/class-refactor` 의 firmware/term_drive_v2/README.md (main 에는 없음).
- `ELECTRICAL_BASELINE §7` · `REAL_ROBOT_VALUES §1-o` · `MASTER_PLAN §7`(열린 예약).
