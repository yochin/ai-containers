cp -rf ../aai4r_edge_interfaces .
docker build --rm -t aai4r/dummybot_ipcam -f Dockerfile .
rm -rf aai4r_edge_interfaces
