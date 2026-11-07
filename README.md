# XT-M60 LiDAR Target Tracking & Point Cloud System

XinTan XT-M60 FLASH/ToF 3D LiDAR 센서와 Digilent Zybo (Zynq-7000) 임베디드 보드를 활용한 실시간 3D 점군 수신, 배경 평면 분할, 목표물(박스) 검출 및 추적, 3D 시각화 통합 패키지입니다.

---

## 📁 디렉터리 구조

```
lidar-all/
├── src/                    # Zybo 임베디드 소스 코드 (C++)
│   ├── live_sender.cpp     # LiDAR SDK 연동, 30 FPS 점군 수신/추적/TCP 송신 메인
│   ├── target_detector.h   # 링 평면 피팅(ringPlane) 및 3D 박스 객체 분할(splitBoxes)
│   ├── box_tracker.h       # 센서 좌표계 기반 Alpha-Beta 다중 표적 추적기
│   └── frame_console.cpp   # XT-M60 30프레임 점군 CSV 덤프 CLI 도구
├── viewer/                 # PC용 시각화 도구
│   ├── zybo_live_viewer.py # PyQt5 / pyqtgraph 기반 3D 점군 및 추적 결과 실시간 뷰어
│   ├── pointcloud-viewer.html # 웹 브라우저 기반 3D 점군 인터랙티브 뷰어 (HTML5 Canvas)
│   └── test_viewer.py      # PyQtGraph 3D 점군 렌더링 기본 검증 스크립트
├── data/                   # 실측 라이다 점군 데이터셋 (.txt)
│   ├── box_o.txt           # Frame 1276 (8,655점, 박스 검출 성공 데이터)
│   ├── box_x.txt           # Frame 1622 (8,662점, 박스 검출 비교 데이터)
│   ├── box.txt             # Frame 17744 (선택된 박스 표적 점군 123점)
│   ├── wall.txt            # Frame 11970 (벽면 점군 193점)
│   └── balloon.txt         # Frame 11612 (풍선 표적 점군 166점)
├── cmake/                  # 빌드 환경 설정
│   └── arm-toolchain.cmake # ARM (Cortex-A9) 크로스 컴파일 CMake 툴체인
├── .gitignore
└── README.md
```

---

## ⚙️ 시스템 사양 및 통신 규격

- **센서 기종**: XinTan XT-M60 (FLASH / ToF 3D LiDAR)
  - 해상도: 160 × 60 (프레임당 최대 9,600점)
  - 프레임 레이트: 30 FPS
  - 유효 거리: 1,500 mm (1.5 m) ~ 964,000 mm
- **네트워크 프로토콜**:
  - **LiDAR → Zybo (UDP 7687)**: 센서 원시 데이터 전송 (`xtsdk` 라이브러리 사용)
  - **Zybo → PC 뷰어 (TCP 9000)**:
    - `XTO1`: 클러스터링 및 알고리즘 수행 통계 헤더 (유효 점 수, 후보군, 소요시간 등)
    - `XTM5`: 점군(XYZ, depth, 라벨) 및 추적 표적 레코드 페이로드

---

## 🚀 실행 방법

### 1. Zybo 보드에서 송신 프로그램 실행
```bash
# 사용법: ./zybo_live <센서_IP> <Zybo_IP> <뷰어_PC_IP>
./zybo_live 192.168.1.100 192.168.1.10 192.168.1.50
```

### 2. PC에서 실시간 3D 뷰어 실행
필요 라이브러리: Python 3, PyQt5, pyqtgraph, numpy
```bash
pip install PyQt5 pyqtgraph numpy
python viewer/zybo_live_viewer.py --port 9000
```
- 마우스 드래그: 3D 뷰 회전
- 마우스 휠: 확대 / 축소
- Shift + 드래그: 특정 영역 점군 데이터 상세 확인
- `Save FULL frame (.txt)`: 현재 프레임의 전체 점군 데이터를 파일로 저장

### 3. 웹 뷰어 (HTML)
`viewer/pointcloud-viewer.html`을 웹 브라우저에서 열고 `data/` 폴더 내의 `.txt` 또는 `.csv` 점군 파일을 드래그 앤 드롭하여 바로 3D 점군을 확인하실 수 있습니다.

---

## 📤 GitHub에 올리는 방법

새로운 GitHub 저장소를 생성하신 후 터미널에서 다음 명령어를 실행하세요:

```bash
cd /home/user/lidar-all
git init
git add .
git commit -m "Initial commit: XT-M60 LiDAR tracking and viewer package"
git branch -M main
git remote add origin https://github.com/<사용자_계정>/<저장소_이름>.git
git push -u origin main
```
