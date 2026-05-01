# xhost +local:docker # If you're on the actual device itself, then create a port to the docker that allows it to open the gui there
docker build -t octo-image:latest . 
docker run --name octo-container --network=host -v /home/ozeronat/VLA_project:/workspace -it octo-image:latest 
# Commands to run if the container has already been created
# docker start octo-container
# docker exec -it octo-image:latest bash
