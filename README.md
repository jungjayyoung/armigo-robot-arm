# AX_SLAVE

STM32F411RE 기반 ARMIGO 로봇팔의 최종 슬레이브 펌웨어입니다.
AX_MASTER의 Fusion UART 명령을 수신해 AX-12A 4축을 구동하고, Sharp
센서로 첫 Auto Step을 해제하며 위치·부하·Auto 상태를 회신합니다.

- 운영 브랜치: `main`
- 최종 코드 기준: `ryu@98b79d2`
- 이전 main 보관: `test`
- 대응 마스터: https://github.com/Armigos/AX_MASTER
- 프로젝트 문서: https://app.notion.com/p/3a6e8d3fa943806caac4f5ef209e2772

## 핵심 기능

- Slave AX-12A ID `1, 2, 3, 5` 제어
- FreeRTOS SlaveControl, Telemetry, SharpSensor 태스크
- USART6 바이트 RX 인터럽트 기반 Fusion 프레임 파싱
- 최신 JOG 명령 우선 처리와 중요 제어 명령 우선순위
- 인터럽트 기반 AX-12 Broadcast Sync Write
- Home, Teaching/JOG, Auto별 모션 프로파일
- 첫 Auto Step의 Sharp 센서 감지 대기
- S-Curve Auto 이동과 4축 도착 피드백
- E-STOP용 실제 현재 자세 고정
- 17바이트 Position/Load/Auto 상태 회신

## 시스템 구조

```text
AX_MASTER / NUCLEO-F411RE
             |
             | USART6 또는 HC-05, 115200
             v
AX_SLAVE / NUCLEO-F411RE
             |
             | USART1 Half-Duplex, 1 Mbps
             v
AX-12A Slave x4 (ID 1, 2, 3, 5)

Sharp sensor -> ADC1 interrupt -> first Auto Step release
ST-LINK VCP <-> USART2, 115200 <-> TelePlot
```

## UART 및 배선

| 인터페이스 | 핀 | 속도 | 용도 |
|---|---|---:|---|
| USART1 | PA9 / D8 | 1,000,000 | AX-12A Half-Duplex DATA |
| USART2 | PA2 TX, PA3 RX | 115,200 | ST-LINK 콘솔 및 TelePlot |
| USART6 | PC6 TX, PC7 RX | 115,200 | AX_MASTER 또는 HC-05 |

유선 연결:

| AX_MASTER | AX_SLAVE |
|---|---|
| PA11 / USART6_TX | PC7 / USART6_RX |
| PA12 / USART6_RX | PC6 / USART6_TX |
| GND | GND |

AX-12A는 외부 9~12V로 구동하고 외부 전원과 Nucleo GND를 공통
연결합니다.

## 모터 매핑

| Master | Slave |
|---:|---:|
| ID 10 | ID 1 |
| ID 11 | ID 2 |
| ID 12 | ID 3 |
| ID 14 | ID 5 |

`Core/Src/ax12_config.c`의 `reversed`와 `offset`으로 관절 방향과
중심을 보정합니다.

## Fusion UART 프로토콜

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

| CMD | 이름 | 슬레이브 처리 |
|---:|---|---|
| `0x01` | SET_GOAL_POS | 개별 축 목표 적용 |
| `0x02` | SET_TORQUE | 네 모터 Torque 변경 |
| `0x03` | REQ_STATUS | 상태 패킷 송신 |
| `0x04` | HOME_POS | Home 프로파일로 4축 이동 |
| `0x05` | SET_ALL_POS | Teaching/JOG 프로파일 적용 |
| `0x06` | START_AUTO | 목표 준비 후 Sharp 감지 대기 |
| `0x07` | RUN_AUTO | Sharp 대기 없이 다음 Auto Step 실행 |
| `0x08` | HOLD_CURRENT | 실제 현재 위치를 Goal로 고정 |
| `0x83` | STATUS_REPLY | Position 8B + Load 8B + Auto flags 1B |

중요 제어 명령과 상태 요청은 연속 JOG 프레임보다 우선 처리되며,
오래된 JOG 대기 프레임은 최신 값으로 교체됩니다.

## 모션 프로파일

| 모드 | Moving Speed | Goal max step |
|---|---:|---:|
| Home | 80 | 1 |
| Teaching/JOG | 300 | 6 |
| Auto | 80 | 6 |

- Goal 갱신: 5ms
- Position 폴링: 10ms
- 비동기 읽기 Timeout: 4ms
- Goal deadband: 1
- Auto S-Curve 갱신: 20ms
- Auto S-Curve 최소 시간: 300ms
- Auto S-Curve middle step: 10

설정은 `Core/Inc/ax12_config.h`에서 관리합니다.

## Auto 및 Sharp 센서

1. `START_AUTO(0x06)` 수신 시 첫 목표와 S-Curve를 준비합니다.
2. SharpSensor 태스크가 20ms마다 ADC 변환을 시작합니다.
3. ADC 값이 유효 감지 범위에 들어오면 첫 Auto 이동을 해제합니다.
4. 목표 도착 후 AX_MASTER가 다음 Step을 `RUN_AUTO(0x07)`로 전송합니다.
5. 이후 Step은 Sharp 감지를 다시 기다리지 않습니다.

## E-STOP 자세 고정

`HOLD_CURRENT(0x08)`을 받으면 각 AX-12A의 실제 Present Position을
읽어 그 값을 Goal로 기록한 뒤 Torque를 유지합니다. 따라서 E-STOP
해제 시 이전 Auto 또는 Home 목표로 되돌아가지 않습니다.

## 링크 타임아웃

유효 프레임이 500ms 동안 없으면 다음 로그를 출력하고 마지막 Goal을
유지합니다.

```text
HC05 link timeout: holding last goal positions
```

현재 정책은 Torque OFF가 아니라 마지막 자세 유지입니다.

## TelePlot

USART2에서 100ms마다 실제 현재 위치만 출력합니다.

```text
>slave_1_pos:512
>slave_2_pos:512
>slave_3_pos:512
>slave_4_pos:512
```

## 콘솔

```text
help
status
bus1m
torque on
torque off
hc05at
hc05exit
```

- `status`: 링크, 프레임, 매핑, Goal, Position, Torque 상태
- `bus1m`: ID 1, 2, 3, 5를 AX-12 1Mbps로 설정 및 검증
- `hc05at`: USART6 38400 AT 브리지
- `hc05exit`: USART6 115200 데이터 모드 복구

## 주요 파일

| 파일 | 역할 |
|---|---|
| `AX_SLAVE.ioc` | CubeMX 핀, UART, ADC, FreeRTOS, NVIC |
| `Core/Inc/ax12_config.h` | 프로토콜, 모션 프로파일, Timeout |
| `Core/Src/ax12_config.c` | Master→Slave 매핑과 보정 |
| `Core/Src/ax12.c` | AX-12 비동기 드라이버와 Sync Write |
| `Core/Src/ax12_app.c` | Fusion 파서, Auto, S-Curve, 상태 회신 |
| `Core/Src/freertos.c` | 제어, Telemetry, Sharp 태스크 |

## 빌드

```bash
cmake --preset Debug
cmake --build --preset Debug
```

결과 파일은 `build/Debug/AX_SLAVE.elf`입니다. STM32CubeIDE에서는
`AX_SLAVE.ioc`를 열고 Build 후 ST-LINK로 다운로드합니다.

## 최종 시험 순서

1. ID 1, 2, 3, 5와 USART1 1Mbps를 확인합니다.
2. AX_MASTER와 UART6을 교차 연결하고 GND를 공통 연결합니다.
3. BTN16 Home에서 네 축이 512로 이동하는지 확인합니다.
4. BTN15 JOG의 네 축 매핑과 방향을 확인합니다.
5. Auto 첫 Step이 Sharp 감지 전 대기하는지 확인합니다.
6. 감지 후 S-Curve 이동과 다음 Step 연속 실행을 확인합니다.
7. E-STOP 시 실제 현재 자세를 유지하는지 확인합니다.
8. `STATUS_REPLY`의 위치, 부하, Auto flags를 AX_MASTER가 받는지 확인합니다.
9. 유선 검증 후 HC-05 무선 링크로 재시험합니다.
