cp -rf ../aai4r_edge_interfaces .
docker build --rm --runtime=nvidia -t aai4r/loom -f loom_aai4r.Dockerfile .
rm -rf aai4r_edge_interfaces
docker tag aai4r/loom asia-northeast1-docker.pkg.dev/lg-robot-dev/lg-ai-registry/etri_loom