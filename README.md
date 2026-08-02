# AX_SLAVE

ARMIGO 4축 슬레이브 컨트롤러 펌웨어입니다. NUCLEO-F411RE에서
`AX_MASTER`의 Fusion UART 명령을 수신하고 Slave AX-12A 4개를 구동합니다.

대응 저장소: [Armigos/AX_MASTER](https://github.com/Armigos/AX_MASTER)

## 역할

- Slave AX-12A ID `1, 2, 3, 5` 제어
- Master 위치 명령 수신 및 4축 Goal 갱신
- Sharp 적외선 센서 ADC 측정
- Auto 첫 Step의 감지 대기와 이후 S-Curve 모션
- 현재 위치 피드백, 부하, 전압/온도 상태 회신
- E-STOP 시 실제 Present Position을 Goal로 고정

## 연결

```text
AX_MASTER USART6 TX PA11 ──> AX_SLAVE USART6 RX PC7
AX_MASTER USART6 RX PA12 <── AX_SLAVE USART6 TX PC6
GND ──────────────────────── GND
```

AX-12A 버스는 USART1 Half-Duplex `1 Mbps`를 사용합니다. AX-12A 전원은
외부 `9~12V`를 사용하고, 외부 전원 GND와 STM32 GND를 공통으로 연결합니다.

## 모터 매핑

| Slave 축 | AX-12A ID |
|---:|---:|
| 1 | 1 |
| 2 | 2 |
| 3 | 3 |
| 4 | 5 |

## Sharp 센서

Sharp 센서는 ADC로 측정하며 상태값을 다음 순서로 계산합니다.

1. 여러 샘플을 평균하여 전압(mV)을 계산합니다.
2. GP2Y0A21YK0F 보정표로 거리(cm)를 환산합니다.
3. 유효 범위 `10~30cm`가 3회 연속 확인되면 `sharp_detected`를 켭니다.
4. 범위를 벗어나면 감지 상태를 해제합니다.

첫 Auto Step은 이 감지 플래그를 기다리고, 한 번 시작된 이후의 다음
Step들은 Sharp 센서를 다시 기다리지 않습니다.

## Fusion UART

```text
AA 55 CMD LEN PAYLOAD... CHECKSUM
```

Checksum은 `CMD + LEN + PAYLOAD` 하위 8비트입니다.

| CMD | 이름 | 처리 |
|---:|---|---|
| `0x01` | SET_GOAL_POS | 개별 축 목표 |
| `0x02` | SET_TORQUE | 전체 Torque |
| `0x03` | REQ_STATUS | 상태 회신 |
| `0x04` | HOME_POS | 네 축을 512로 이동 |
| `0x05` | SET_ALL_POS | JOG/Teaching 목표 |
| `0x06` | START_AUTO | 첫 목표를 준비하고 Sharp 대기 |
| `0x07` | RUN_AUTO | 다음 Auto 목표 실행 |
| `0x08` | HOLD_CURRENT | 현재 실제 위치를 Goal로 고정 |
| `0x83` | STATUS_REPLY | 상태 회신 |

`STATUS_REPLY` payload는 다음 총 20바이트입니다.

```text
Position 8B + Load 8B + Auto flags 1B + Sharp mV 2B + Sharp cm 1B
```

AX_MASTER와 AX_SLAVE는 같은 상태 프레임 길이를 사용해야 합니다. 한쪽만
업데이트하면 Master가 상태 회신을 무시해 Home이 `MOVING`에 남을 수 있습니다.

## E-STOP / HOLD_CURRENT

`HOLD_CURRENT`를 받으면 이전에 저장된 `motor_present` 캐시를 사용하지 않고
각 AX-12A의 현재 Present Position을 즉시 읽습니다. 읽은 위치를 Goal과
Target에 동시에 기록한 뒤 Torque를 유지하므로, 비상정지 순간의 실제
자세에서 멈춥니다.

## TelePlot

슬레이브 USART2와 ST-LINK Virtual COM Port를 `115200, 8N1`로 연결합니다.

```text
>sharp_mv:1234
>sharp_cm:25
>sharp_detected:1
>slave_1_pos:512
```

마스터로 상태 프레임이 전달되면 AX_MASTER에서도 다음 값을 볼 수 있습니다.

```text
>master_sharp_mv:1234
>master_sharp_cm:25
>master_sharp_detected:1
```

## 빌드

```bash
cmake --preset Debug
cmake --build --preset Debug
```

결과 파일:

```text
build/Debug/AX_SLAVE.elf
```

VS Code에서는 `CMake: Delete Cache and Reconfigure` 후 `CMake: Build`를
실행합니다. Windows에서 생성된 CMake 캐시는 macOS에서 재사용하지 말고
다시 구성하십시오. ST-LINK 업로드 후에는 반드시 AX_MASTER와 동일한
프로토콜 버전인지 확인합니다.

## 주요 파일

| 파일 | 역할 |
|---|---|
| `AX_SLAVE.ioc` | CubeMX 핀·UART·FreeRTOS 설정 |
| `Core/Src/ax12_app.c` | Fusion 명령·모션·상태 회신 |
| `Core/Src/main.c` | Sharp ADC 측정·거리 환산 |
| `Core/Src/freertos.c` | 센서·슬레이브·TelePlot 태스크 |
| `Core/Inc/ax12_config.h` | 프로토콜·ID·모션 설정 |
