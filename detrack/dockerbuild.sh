cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/detrack -f detrack_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/detrack asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry/etri_detrack