# 설치와 실행 방법

1. 도커 이미지 생성

```
./dockerbuild_meal.sh
```

```meal_aai4r.Dockerfile```의 111, 112행의 코멘트를 없애고 빌드를 해서 성공하면 아래 2~4단계를 수행하지 않아도 됩니다.
단, 이 경우 먼저 아래 링크의 2절에서 안내하는 방법으로 Docker의 daemon.json 파일을 수정해서 기본 런타임을 NVidia 런타임으로 설정해 주셔야 합니다.

- [https://hanseokhyeon.tistory.com/entry/docker-runtimenvidia-default로-바꾸기-GPU-사용-default](https://hanseokhyeon.tistory.com/entry/docker-runtimenvidia-default로-바꾸기-GPU-사용-default)

2. 식사맥락이해 지능모듈의 내부 연산자 컴파일 (자동으로 안되어서... 수작업으로 합니다.)

```
docker run -it --rm --gpus all aai4r/meal2 bash
cd /aai4r/aai4r-TableServiceDetection/MultiStreamDeformableDETR/models/ops
./make.sh
```

3. 컴파일이 잘 되었는지 확인

```
cd /aai4r/aai4r-TableServiceDetection
python3 table_service_alarm_interface.py
```

'Possible Service Results'라고 뜨면서 결과를 몇 개 출력하면 성공입니다.

4. 컴파일을 통해 변경된 컨테이너를 이미지로 저장

새 터미널을 열고 아래 명령을 실행

```
docker container ps
<you will see a running container ID of aai4r/meal2>
docker commit <ID> aai4r/meal2
```

5. Run the container.

meal2 지능모듈의 컨테이너 실행!

```
docker run --gpus all --name aai4r_meal2 --rm -it aai4r/meal2 /aai4r/run_meal.sh
```

## 인터페이스

### ```/camera/robot_image_info (RobotImageInfo)```

- 이미지와 각종 부가 정보를 수신함

### ```/meal/event (String)```

- 식사 시작 이벤트를 수신함
- 메시지는 JSON 형식으로 아래와 같이 ```meal_start_time``` 필드에 시간 정보를 포함해야 함

```
{'meal_start_time': '2023-02-24-10-10-54'}
```

- 식사 맥락 이해 기능은 이 이벤트를 받은 이후에만 동작함
