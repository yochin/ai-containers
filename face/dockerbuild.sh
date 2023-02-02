cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/facial2 -f Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/facial2 asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry/etri_facial2