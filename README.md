# Deli-Way | ROS 2 LIMO capstone

국민대학교 3인 팀 캡스톤 프로젝트에서 LIMO 로봇을 이용한 배송 시나리오를 구현했습니다. 저는 **주행 알고리즘, 객체 탐지, 주행 제어의 연결**을 담당했습니다. 현장 조사를 바탕으로 약 10 m × 6 m의 시험 환경을 구성해 인지·경로·제어 흐름을 검증했습니다.

## 이 저장소에서 볼 수 있는 것

- [LAB 기반 적응형 차선 인식](limo_ros2_application/lab_adaptive_lane.py): 조도 변화에 대응하기 위한 색공간 변환, 적응형 이진화, 형태학적 필터링, 윤곽선 선택.
- [기본 차선 인식](limo_ros2_application/detect_line.py) 및 [모델 기반 차선 인식 예제](limo_ros2_application/dl_detect_line.py).
- [주행 제어](limo_ros2_application/limo_control.py), [직진 주행](limo_ros2_application/drive_straight.py), [목표 자세로 이동](limo_ros2_application/move_to_pose.py) 등의 ROS 2 노드.
- [Launch](launch/), [파라미터](params/), [RViz 설정](rviz/)과 [패키지 정의](package.xml).

## 프로젝트 흐름

카메라 기반 인지 결과를 ROS 2 주행 노드에 연결하고, Nav2 Waypoint 방문과 구간별 설정을 사용해 배송 경로를 구성했습니다. 배송 시나리오에서는 목적지 도착 후 수령 승인 또는 시간 초과를 구분하고, 미수령 목적지를 저장해 다시 방문하도록 설계했습니다.

## 실행 시 참고

이 저장소는 **공개 가능한 ROS 2 코드 아카이브**입니다. 전체 배송 서비스의 모든 구성 요소가 여기에 포함된 것은 아닙니다. 전체 시스템을 실행하려면 LIMO 하드웨어, 호환 ROS 2 환경, 패키지에 명시된 외부 의존성이 필요합니다. 객체 탐지 예제는 이 저장소에 포함되지 않은 로컬 TensorRT 파일도 참조합니다.

팀 프로젝트와 제공된 플랫폼을 바탕으로 한 작업이며, 저장소의 모든 구성 요소를 단독 작성했다고 주장하지 않습니다.
