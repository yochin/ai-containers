cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/meal2 -f meal_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
