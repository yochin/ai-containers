cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/dummybot -f Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/dummybot robot-registry.ainize.ai/aai4r/dummybot