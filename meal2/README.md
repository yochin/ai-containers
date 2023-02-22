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

3. Run a test script.

```
cd /aai4r/aai4r-TableServiceDetection
python3 table_service_alarm_interface.py
```

If you see a 'Possible Service Results', it is OK.

4. Commit the container into an image.

In a new terminal, run the following commands.

```
docker container ps
<you will see a running container ID of aai4r/meal2>
docker commit <ID> aai4r/meal2
```

5. Run the container.

```
docker run --gpus all --name aai4r_meal2 --rm -it aai4r/meal2 /aai4r/run_meal.sh
```