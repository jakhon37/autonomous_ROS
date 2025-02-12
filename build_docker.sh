
docker build -t autonomous_ros_project .


docker run -d \
  --restart=always \
  --name autonomous_ros_project \
  --network=host \
  autonomous_ros_project
