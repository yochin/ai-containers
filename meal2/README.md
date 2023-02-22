# Install

1. Run dockerbuild

```
./dockerbuild_meal.sh
```

2. Build the intelligence model inside the docker container

```
docker run -it --rm --gpus all aai4r/meal2 bash
cd /aai4r/aai4r-TableServiceDetection/MultiStreamDeformableDETR/models/ops
./make.sh
```

3. 