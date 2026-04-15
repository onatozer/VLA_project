# xhost +local:docker # If you're on the actual device itself, then create a port to the docker that allows it to open the gui there
sudo docker build -t octo-container:latest . 
sudo docker run -v /home/ozeronat/VLA_project:/workspace -it octo-container:latest