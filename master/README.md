# AX_MASTER

ARMIGO 4축 마스터 컨트롤러 펌웨어입니다. NUCLEO-F411RE에서 마스터
AX-12A 4개의 현재 위치를 읽고, 키패드·LCD·E-STOP을 처리한 뒤
Fusion UART로 `AX_SLAVE`에 명령을 보냅니다.

대응 저장소: [Armigos/AX_SLAVE](https://github.com/Armigos/AX_SLAVE)

## 역할

- Master AX-12A ID `10, 11, 12, 14` 위치 읽기
- Admin JOG, Teaching, Auto 센서 모드
- Preset 1~10, Preset당 최대 30 Step Flash 저장
- BTN16 Home 안전 인터록
- PB2 E-STOP 및 슬레이브 실제 현재 위치 고정
- LCD 상태 표시와 ST-LINK VCP TelePlot 출력

## 전체 구조

```text
Master AX-12A x4 (10, 11, 12, 14)
        │ USART1 Half-Duplex / 1 Mbps
        ▼
AX_MASTER / NUCLEO-F411RE
        │ USART6 / 115200 또는 HC-05
        ▼
AX_SLAVE / NUCLEO-F411RE
        │ USART1 Half-Duplex / 1 Mbps
        ▼
Slave AX-12A x4 (1, 2, 3, 5)
```

## 키패드 운용

| 입력 | 기능 |
|---|---|
| BTN16 | 4축 Home 위치 `512` 이동 |
| BTN15 | Admin JOG 진입, JOG 중 재입력 시 Dashboard |
| BTN13 | Teaching 모드 진입 |
| BTN1~10 | Teaching/Auto Preset 선택 |
| BTN12 | 현재 자세를 선택 Preset의 다음 Step으로 저장 |
| BTN11 | 선택 Preset 전체 삭제 |
| BTN14 | Auto 센서 모드 진입 |

### Teaching

1. BTN13을 누릅니다.
2. BTN1~10으로 Preset을 선택합니다.
3. 로봇 자세를 만든 뒤 BTN12를 눌러 Step을 저장합니다.
4. 다음 자세마다 BTN12를 반복합니다.

LCD에는 `1-10: SELECT PRESET`, `12: SAVE STEP`, `11: DELETE 14:AUTO`
가이드가 표시됩니다. Preset을 선택하지 않고 BTN12를 누르면 저장되지
않습니다.

### Auto 센서 모드

1. BTN16으로 Home을 완료합니다. Leader와 Follower 양쪽 축이 모두 `512`에 도착한 뒤에만 Home을 완료로 판정하며, Follower가 먼저 도착해도 Leader의 이동 중 위치를 다시 전송하지 않습니다. Home 완료 직후에는 양쪽 Torque를 해제하고 자동 실행하지 않고
   대기합니다.
2. BTN14를 눌러 Auto 센서 모드에 들어갑니다.
3. BTN1~10으로 실행할 Teaching Preset을 선택합니다.
4. 선택한 Preset이 저장되어 있고 Sharp 센서가 5초 동안 감지되면 해당
   Preset의 전체 Step을 한 번 실행합니다.
5. 한 사이클의 마지막 Step이 끝나면 양쪽 로봇팔은 기본 위치 `512`로
   자동 복귀합니다.
6. 실행 중 또는 완료 직후 물체가 계속 감지되면 재실행하지 않습니다.
   재무장 대기 중에는 센서 상태를 10ms 주기로 확인하며, 물체가 센서
   범위에서 사라진 뒤 다시 5초 감지되면 재실행됩니다.

센서 감지 조건은 `AX_SLAVE`에서 10~17cm 범위의 샘플 3회 연속 확인으로
판정합니다. 슬레이브는 범위를 벗어난 샘플도 3회 연속 확인한 뒤 감지를
해제하여 순간적인 센서 노이즈로 카운트다운이 초기화되지 않게 합니다.
LCD에는 `OBJECT 10-17 CM`으로 표시하며, 감지 중단 또는 슬레이브 상태
회신 타임아웃이 안정적으로 확인되면 카운트다운을 초기화합니다.

## E-STOP

PB2 E-STOP이 발생하면 현재 Home/JOG/Auto를 중단하고 `HOLD_CURRENT`
명령을 보냅니다. 슬레이브는 캐시된 목표 위치가 아니라 그 순간 읽은
실제 Present Position을 Goal로 고정합니다. BTN15로 안전 JOG를 재개할
때도 이전 Auto 목표를 다시 실행하지 않습니다.

## Fusion UART

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

Checksum은 `CMD + LEN + PAYLOAD` 하위 8비트입니다.

| CMD | 이름 | 용도 |
|---:|---|---|
| `0x01` | SET_GOAL_POS | 개별 목표 위치 |
| `0x02` | SET_TORQUE | 슬레이브 Torque |
| `0x03` | REQ_STATUS | 상태 요청 |
| `0x04` | HOME_POS | 4축 Home |
| `0x05` | SET_ALL_POS | JOG/Teaching 위치 |
| `0x06` | START_AUTO | 첫 Auto Step 등록 |
| `0x07` | RUN_AUTO | 이후 Auto Step 실행 |
| `0x08` | HOLD_CURRENT | 현재 자세 고정 |
| `0x83` | STATUS_REPLY | 위치·부하·Auto·Sharp 상태 |

`STATUS_REPLY`는 위치 8바이트, 부하 8바이트, 상태 플래그 1바이트,
Sharp 전압 2바이트, 환산 거리 1바이트의 총 20바이트입니다.
AX_MASTER와 AX_SLAVE는 같은 상태 프레임 버전을 사용해야 합니다.

## TelePlot

마스터 USART2와 ST-LINK Virtual COM Port를 `115200, 8N1`로 연결합니다.

```text
>master_sharp_mv:1234
>master_sharp_cm:25
>master_sharp_detected:1
>auto_selected_preset:3
>auto_countdown:5
```

주요 축/상태 값은 `Core/Src/freertos.c`의 TelePlot 태스크에서 출력합니다.

## 빌드

```bash
cmake --preset Debug
cmake --build --preset Debug
```

결과 파일:

```text
build/Debug/AX_MASTER.elf
```

VS Code에서는 `CMake: Delete Cache and Reconfigure` 후 `CMake: Build`를
실행합니다. Windows에서 생성된 `build/Debug` 캐시는 macOS에서 재사용하지
말고 다시 구성하십시오. ST-LINK 업로드는 `AX_MASTER.ioc` 프로젝트의
Run/Debug 설정을 사용합니다.

## 주요 파일

| 파일 | 역할 |
|---|---|
| `AX_MASTER.ioc` | CubeMX 핀·UART·FreeRTOS 설정 |
| `Core/Src/freertos.c` | 키패드·모드·LCD·통신·E-STOP |
| `Core/Src/ax12.c` | AX-12 Protocol 1.0 드라이버 |
| `Core/Src/teaching_storage.c` | Teaching Flash 저장 |
| `Core/Inc/ax12_config.h` | ID·속도·모션 설정 |
