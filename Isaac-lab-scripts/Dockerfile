FROM isaac-lab-base:latest

ARG SIM_REPO_PATH

ENV SIM_DEST_PATH=../sim_to_real

COPY ${SIM_REPO_PATH} ${SIM_DEST_PATH}

RUN /isaac-sim/python.sh -m pip install -e $SIM_DEST_PATH



