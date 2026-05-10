FROM isaac-lab-base:latest

ARG GEN3_REPO_URL=https://github.com/louislelay/kinova_isaaclab_sim2real.git
ARG GEN3_BRANCH=main

ENV SIM_TO_REAL_REPO=../sim_to_real

RUN git -C $SIM_TO_REAL_REPO pull || git clone --depth 1 ${GEN3_REPO_URL} -b ${GEN3_BRANCH} $SIM_TO_REAL_REPO
RUN /isaac-sim/python.sh -m pip install -e $SIM_TO_REAL_REPO/source/gen3



