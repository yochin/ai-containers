cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/facial -f facial_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/facial robot-registry.ainize.ai/aai4r/facial