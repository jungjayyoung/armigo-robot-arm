# AX_SLAVE

STM32F411RE 기반 로봇팔 슬레이브 펌웨어입니다. AX_MASTER에서
Fusion UART 프로토콜로 받은 위치를 AX-12A 슬레이브 모터 4개의 목표
위치로 매핑합니다. FreeRTOS와 UART 인터럽트를 사용해 링크 수신,
AX-12 Sync Write, 현재 위치 TelePlot 출력을 분리해 처리합니다.

- GitHub: https://github.com/Armigos/AX_SLAVE
- 최적화 브랜치: `feature/uart-interrupt`
- 대응 마스터: https://github.com/Armigos/AX_MASTER

## 시스템 구성

```text
AX_MASTER + AX-12A x4
             |
             | USART6 / HC-05, 115200
             v
NUCLEO-F411RE / AX_SLAVE
             |
             | USART1 Half-Duplex, 1 Mbps
             v
AX-12A Slave x4 (ID 1, 2, 3, 5)

ST-LINK VCP <-- USART2, 115200 --> TelePlot / console
```

## UART 및 핀 설정

| 인터페이스 | STM32 핀 | 속도 | 용도 |
|---|---|---:|---|
| USART1 | PA9 / D8 | 1,000,000 | AX-12A Half-Duplex DATA |
| USART2 | PA2 TX, PA3 RX | 115,200 | ST-LINK 콘솔 및 TelePlot |
| USART6 | PC6 TX, PC7 RX | 115,200 | HC-05 또는 AX_MASTER 링크 |

UART 형식은 8 data bits, no parity, 1 stop bit입니다.

## 모터 매핑

| 마스터 논리축 | 마스터 ID | 슬레이브 ID |
|---|---:|---:|
| Master 1 | 10 | 1 |
| Master 2 | 11 | 2 |
| Master 3 | 12 | 3 |
| Master 4 | 14 | 5 |

매핑, 방향 반전, 중심 오프셋은 `Core/Src/ax12_config.c`에서 설정합니다.

```c
const AX12_SlaveMotorConfig AX12_SLAVE_MOTORS[] = {
    {.master_id = 10U, .slave_id = 1U, .reversed = false, .offset = 0},
    {.master_id = 11U, .slave_id = 2U, .reversed = false, .offset = 0},
    {.master_id = 12U, .slave_id = 3U, .reversed = false, .offset = 0},
    {.master_id = 14U, .slave_id = 5U, .reversed = false, .offset = 0},
};
```

관절 방향이 반대면 `reversed=true`, 중심 보정이 필요하면 `offset`을
AX-12 position 단위로 조정합니다.

## Fusion UART 프로토콜

Master_fusion과 동일한 가변 길이 패킷을 사용합니다.

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

Checksum은 `CMD + LEN + 모든 Payload`의 하위 8비트입니다.

| CMD | 이름 | 처리 |
|---:|---|---|
| `0x01` | SET_GOAL_POS | 지정 축 하나의 목표 위치 변경 |
| `0x02` | SET_TORQUE | 네 슬레이브 모터 Torque 변경 |
| `0x03` | REQ_STATUS | 20바이트 상태 회신 |
| `0x04` | HOME_POS | 네 축 Home 위치 적용 |
| `0x05` | SET_ALL_POS | 네 마스터 위치를 슬레이브 목표로 적용 |
| `0x83` | STATUS_REPLY | 슬레이브→마스터 상태 패킷 |

기존 `A5 5A` 헤더의 고정 18바이트 프레임은 현재 최적화 브랜치에서
사용하지 않습니다.

## 실시간 처리

- USART6는 1바이트 RX 인터럽트로 프레임을 계속 조립합니다.
- 새 위치 프레임이 연속으로 들어오면 오래된 대기 명령 대신 최신 명령을
  우선해 지연 누적을 막습니다.
- SlaveControl 태스크는 1 ms 주기로 링크와 AX-12 상태 머신을 처리합니다.
- 목표값은 기본 5 ms마다 최대 3 position tick씩 이동합니다.
- 네 목표 위치는 인터럽트 기반 Broadcast Sync Write로 함께 전송합니다.
- 현재 위치 읽기는 비동기이며 새 링크 명령이 오면 제어 명령을 우선합니다.
- 현재 위치는 모터당 25 ms 기준으로 순환 폴링합니다.
- 링크가 500 ms 끊기면 마지막 목표 위치를 유지합니다.

부드러움과 추종 지연의 균형은 `Core/Inc/ax12_config.h`에서 조절합니다.

```c
#define AX12_GOAL_UPDATE_MS   5U
#define AX12_GOAL_MAX_STEP    3U
#define AX12_GOAL_DEADBAND    1U
```

`AX12_GOAL_MAX_STEP`을 키우면 더 빨리 따라가지만 급격해질 수 있고,
줄이면 부드러워지지만 지연이 커집니다.

## 안전 초기화

1. ID 1, 2, 3, 5를 Ping합니다.
2. 모든 슬레이브 모터 Torque를 OFF합니다.
3. 각 모터의 현재 위치를 읽어 초기 Goal로 사용합니다.
4. 유효한 마스터 명령을 기다립니다.
5. Torque 명령 또는 위치 명령을 받으면 Torque를 켜고 추종을 시작합니다.
6. 500 ms 링크 타임아웃 시 마지막 목표 위치를 유지합니다.

링크 단절 시 Torque OFF가 아니라 위치 유지 정책이므로 실제 장비의 안전
요구사항에 맞는지 반드시 검토하십시오.

## 보드 간 유선 테스트

| Master_fusion | AX_SLAVE |
|---|---|
| PA11 / USART6_TX | PC7 / USART6_RX |
| PA12 / USART6_RX | PC6 / USART6_TX |
| GND | GND |

TX/RX를 교차하고 GND를 공통 연결합니다. 양쪽 USART6은 115200입니다.

## HC-05 연결

| AX_SLAVE | Slave HC-05 |
|---|---|
| PC6 / USART6_TX | RXD |
| PC7 / USART6_RX | TXD |
| GND | GND |
| 5V | VCC |

Slave HC-05는 `ROLE=0`, 데이터 UART는 115200으로 설정합니다.

## TelePlot

USART2에서 100 ms마다 슬레이브의 실제 현재 위치 네 개만 출력합니다.

```text
>slave_1_pos:512
>slave_2_pos:512
>slave_3_pos:512
>slave_4_pos:512
```

예전 그래프가 남아 있으면 TelePlot의 Clear를 누른 뒤 위 네 항목만
추가합니다.

## 콘솔 명령

```text
help
status
bus1m
torque on
torque off
hc05at
hc05exit
```

- `status`: 링크, 프레임, 매핑, Goal, Present Position, Torque 상태 출력
- `bus1m`: ID 1, 2, 3, 5의 AX-12 baud를 1 Mbps로 설정 및 검증
- `hc05at`: USART6을 HC-05 AT 모드 38400으로 바꾸고 USART2와 브리지
- `hc05exit`: USART6을 데이터 모드 115200으로 복구

HC-05 AT 모드 사용 시 모듈의 KEY/EN을 활성화한 상태로 부팅하고
ST-LINK Virtual COM Port는 USART2 속도인 115200으로 엽니다.

## 주요 파일

| 파일 | 역할 |
|---|---|
| `AX_SLAVE.ioc` | CubeMX 핀, UART, FreeRTOS, NVIC 설정 |
| `Core/Inc/ax12_config.h` | 속도, 프로토콜, 램핑, 타임아웃 설정 |
| `Core/Src/ax12_config.c` | Master→Slave 매핑, 반전, 오프셋 |
| `Core/Src/ax12.c` | AX-12 Protocol 1.0 비동기 드라이버와 Sync Write |
| `Core/Src/ax12_app.c` | Fusion 파서, 제어 상태 머신, 콘솔 |
| `Core/Src/freertos.c` | 제어 및 TelePlot 태스크 |

## 빌드

STM32CubeIDE에서 `AX_SLAVE.ioc`를 열어 빌드하거나 CMake preset을
사용합니다.

```bash
cmake --preset Debug
cmake --build --preset Debug
```

결과 파일:

```text
build/Debug/AX_SLAVE.elf
```

## 테스트 순서

1. 슬레이브 모터를 한 개씩 연결해 ID 1, 2, 3, 5와 1 Mbps를 확인합니다.
2. USART2의 `status`로 네 모터 초기화를 확인합니다.
3. HC-05 없이 Master_fusion과 USART6을 교차 연결합니다.
4. 양쪽 보드를 Reset하고 마스터의 버튼 15를 누릅니다.
5. 각 마스터 축을 움직여 대응 슬레이브 축을 확인합니다.
6. TelePlot에서 `slave_1_pos`부터 `slave_4_pos`를 확인합니다.
7. 유선 시험 성공 후 HC-05 두 개로 교체합니다.
8. 부하가 없는 상태에서 `reversed`와 `offset`을 조정합니다.

## 문제 해결

- 프레임이 0개면 USART6 핀, TX/RX 교차, 115200, 공통 GND를 확인합니다.
- 프레임은 증가하지만 모터가 안 움직이면 Torque, AX-12 1 Mbps, ID,
  외부 전원을 확인합니다.
- 특정 축만 실패하면 `ax12_config.c` 매핑과 해당 모터 ID를 확인합니다.
- 모터가 툭툭 움직이면 링크 프레임 간격, AX-12 응답 실패, 전원 강하,
  `AX12_GOAL_UPDATE_MS`와 `AX12_GOAL_MAX_STEP`을 함께 확인합니다.
- HC-05 문제를 의심하기 전에 반드시 USART6 유선 시험을 먼저 통과시킵니다.
