# Windows에서 AX_SLAVE 사용하기

이 프로젝트는 소스 경로 기준의 상대 경로 CMake 설정을 사용한다. 따라서 `D:\ryu\STM32\AX_SLAVE`에 두거나 다른 폴더로 옮겨도 프로젝트 폴더 전체(`Core`, `Drivers`, `Middlewares`, `cmake`, `AX_SLAVE.ioc`)를 함께 유지하면 된다.

## STM32CubeIDE로 열기 (권장)

1. STM32CubeIDE를 실행한다.
2. **File → Open Projects from File System…**를 선택한다.
3. `D:\ryu\STM32\AX_SLAVE`를 Directory로 추가한다.
4. **AX_SLAVE** 프로젝트를 선택해 Finish한다.
5. 프로젝트를 우클릭하여 **Refresh** 후 **Build Project**를 실행한다.

`AX_SLAVE.ioc`를 열어 코드 재생성을 실행할 경우 사용자 코드 영역 밖의 변경 사항이 덮어써질 수 있으므로, 먼저 Git 커밋 또는 백업을 남긴다.

## CMake로 빌드하기

STM32CubeIDE의 개발자 명령 프롬프트 또는 `arm-none-eabi-gcc`, `cmake`, `ninja`가 PATH에 등록된 터미널에서 실행한다.

```powershell
Set-Location D:\ryu\STM32\AX_SLAVE
cmake --preset Debug
cmake --build --preset Debug
```

생성물은 `build\Debug\AX_SLAVE.elf`이다.

## Master_fusion과 UART6로 직접 연결할 때

| Master_fusion | AX_SLAVE |
|---|---|
| PA11 / USART6_TX | PC7 / USART6_RX |
| PA12 / USART6_RX | PC6 / USART6_TX |
| GND | GND |

두 보드 모두 115200, 8N1 설정이다. Bluetooth를 사용할 때도 각 보드의 UART6 TX/RX는 모듈의 RXD/TXD와 교차하고 GND를 공통으로 연결한다.

## 다른 PC 또는 다른 위치로 옮긴 경우

다른 PC에서 생성한 `build` 폴더는 복사하지 않는다. 새 위치에서 `cmake --preset Debug`를 먼저 실행하여 로컬 컴파일러와 새 경로로 빌드 캐시를 생성한다.
