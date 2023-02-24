# dummybot

본 모듈은 RobotImageInfo 메시지를 배포합니다.

1. 도커 이미지 생성

```
dockerbuild.sh
```

2. 이미지 배포 기능 실행

현재는 식탁 이미지 한 장을 반복하여 배포합니다.

```
run_container.sh
```

3. 메시지 배포 현황 확인

```
run_container_showimg.sh
```

## ROS2 Interface

### Published Topics

#### ``/camera/robot_image_info`` ([RobotImageInfo](https://github.com/aai4r/ai-containers/blob/main/aai4r_edge_interfaces/msg/RobotImageInfo.msg))

