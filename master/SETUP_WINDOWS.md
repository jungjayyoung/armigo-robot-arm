# Windows에서 AX_MASTER 사용하기

이 프로젝트는 소스 경로 기준의 상대 경로 CMake 설정을 사용한다. 따라서 `D:\ryu\STM32\AX_MASTER` 또는 다른 폴더로 옮겨도 전체 프로젝트 폴더 구조(`Core`, `Drivers`, `Middlewares`, `cmake`, `.ioc`)만 유지하면 된다.

## STM32CubeIDE로 열기 (권장)

1. STM32CubeIDE를 실행한다.
2. **File → Import → General → Existing Projects into Workspace**를 선택한다.
3. `D:\ryu\STM32\AX_MASTER`를 루트 디렉터리로 지정한다.
4. 목록의 **Armigo**를 선택해 Import한다. `Copy projects into workspace`는 체크하지 않는다.
5. 프로젝트를 우클릭하여 **Refresh** 한 뒤 **Build Project**를 실행한다.

`AX_MASTER.ioc`를 열었을 때 코드 재생성을 제안하면, 필요한 설정만 확인한 뒤 생성한다. 사용자 코드 영역 밖의 변경 사항은 덮어쓸 수 있으므로 Git 커밋 또는 백업 뒤에 진행한다.

## CMake로 빌드하기

STM32CubeIDE의 개발자 명령 프롬프트 또는 `arm-none-eabi-gcc`, `cmake`, `ninja`가 PATH에 등록된 터미널에서 실행한다.

```powershell
Set-Location D:\ryu\STM32\AX_MASTER
cmake --preset Debug
cmake --build --preset Debug
```

생성물은 `build\Debug\AX_MASTER.elf`이다.

## 다른 PC 또는 다른 경로로 옮긴 경우

이전 PC에서 생성한 `build` 폴더는 컴파일러와 절대 경로를 기억하므로 복사하지 않는다. 다음 명령으로 새 경로에서 다시 생성한다.

```powershell
Set-Location <프로젝트_경로>
cmake --preset Debug
cmake --build --preset Debug
```

빌드 설정 자체는 `CMakePresets.json`의 `${sourceDir}` 상대 경로를 사용하므로, 프로젝트 위치를 코드에 따로 입력할 필요가 없다.
