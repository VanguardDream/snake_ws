# WSL (Windows Subsystem for Linux)에서 ROS 환경 구축하기
> WSL을 이용하면, Windows 시스템과 동시에 Linux 시스템을 활용할 수 있습니다.

환경 구축 방법은 아래와 같이 3단계로 이루어집니다.
1. WSL 설치
2. ROS 설치
3. GPU 드라이버 설치


## WSL 설치
> WSL은 Powershell을 이용하여 쉽게 설치할 수 있습니다. (모든 스크립트는 관리자 권한으로 실행)
> https://docs.microsoft.com/ko-kr/windows/wsl/install-win10

우선 WSL 기능을 킵니다.
```powershell
dism.exe /online /enable-feature /featurename:Microsoft-Windows-Subsystem-Linux /all /norestart
```

그 다음 가상화 기능을 킵니다.
```powershell
dism.exe /online /enable-feature /featurename:VirtualMachinePlatform /all /norestart
```

이후 WSL2 버전을 사용하도록 명시합니다.
```powershell
wsl --set-default-version 2
```

이후 윈도우 스토어에서 Ubuntu 배포판을 설치합니다. (18.04 or 20.04 설치가 가능합니다.)
![Windows_Store_Ubuntu](https://docs.microsoft.com/ko-kr/windows/wsl/media/store.png)

설치가 되면 일반적인 스토어 앱 실행과 같이 설치된 배포판을 실행하여 UNIX 사용자, 비밀번호를 설정하여 이용합니다.
![UNIX_Setting_on_WSL](https://docs.microsoft.com/ko-kr/windows/wsl/media/ubuntuinstall.png)

>참고로 WSL에서는 기본적으로 GUI 프로그램 실행이 안됩니다. XSever와 같은 Third-Party 응용프로그램을 이용해서 GUI 프로그램을 이용할 수 있습니다. 관련 사항은 직접 찾아보세요.

## ROS 설치
> WSL 설치이후, ROS 설치를 진행합니다.  (저는 기존 코드와 호환을 위해서 18.04 + Melodic으로 설정했습니다.)
> http://wiki.ros.org/melodic/Installation/Ubuntu 

Package 관리자에 ROS PPA를 추가합니다.
1
```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

2
```bash
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

PPA를 추가했으므로 Package list를 업데이트합니다.
```bash
sudo apt update
```

Package 관리자를 이용하여 ROS를 설치합니다.
```bash
sudo apt install ros-melodic-desktop-full
```

이후 Workspace 설정 관련은 생략하도록 하겠습니다. (공식 설치 과정을 따라하세요.)

## GPU 드라이버 설치
> 위에서 GUI는 지원하지 않는다고 말했지만, 사실 Microsoft는 WSL에 최종적으로 GUI를 지원하려고 하고 있습니다. 현 시점(2021년 5월)에서는 Windows Preview 버전에서만 GUI를 기본적으로 제공하고 있습니다. 
> 저는 Preview 버전을 사용하고 있기 때문에 GUI를 사용할 수 있습니다. 하지만 문제는 그래픽 드라이버가 설치되지 않아서 가벼운 3D 프로그램을 실행시켜도 CPU Load가 100%를 달성하는 문제가 생깁니다.
> 이를 해결하기 위해서 WSL에서 GPU드라이버를 설치합니다.
> https://docs.nvidia.com/cuda/wsl-user-guide/index.html


NVIDIA의 PPA를 추가합니다.
```bash
sudo apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/7fa2af80.pub
sh -c 'echo "deb http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64 /" > /etc/apt/sources.list.d/cuda.list'
```

추가했으므로, 업데이트 합니다.
```bash
sudo apt-get update
```

`!주의`할 점인데, 일반적인 Ubuntu에서 CUDA 드라이버를 설치하는 것과 다르게 아래의 명령어로 특정한 드라이버를 설치하라고 지시하고 있습니다.
```bash
sudo apt-get install -y cuda-toolkit-11-0
```