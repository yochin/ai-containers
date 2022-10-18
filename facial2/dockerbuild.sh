cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/facial2 -f facial2_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/facial2 robot-registry.ainize.ai/aai4r/facial2