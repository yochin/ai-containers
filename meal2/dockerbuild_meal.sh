cp -rf ../aai4r_edge_interfaces .
docker build --gpus all --rm -t aai4r/meal2 -f meal_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/meal2 robot-registry.ainize.ai/aai4r/meal