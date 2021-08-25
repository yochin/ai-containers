cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/meal -f meal_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/meal robot-registry.ainize.ai/aai4r/meal