cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/loom -f loom_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/loom robot-registry.ainize.ai/aai4r/loom