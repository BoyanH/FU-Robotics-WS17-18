# Intel Graphics Accelaration + ROS Kinetic docker file
FROM osrf/ros:kinetic-desktop-xenial

RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full=1.3.1-0*

RUN apt-get install --yes liburdfdom-tools \
    openssh-server \
    ssl-cert \
    vim \
    tmux \
    zsh \
    python-setuptools \
    python-pip \
    python-nose \
    python-pytest \
    python-sklearn \
    python-skimage \
    python-h5py \
    python-matplotlib \
    python-scipy \
    python-matplotlib \
    python-pip \
    python-flake8 \
    ipython \
    ipython-notebook \
	libgl1-mesa-glx \
	libgl1-mesa-dri

RUN pip install --upgrade pip

RUN pip install \
    jupyter \
    xgboost \
    seaborn \
    pytest-cov \
    pytest-benchmark \
    pytest-flake8 \
    more_itertools \
    scikit-image \
    click \
    pandas

RUN mkdir -p ~/seat && \
    cd ~/seat && \
    git clone -b version-3-kinetic https://github.com/AutoModelCar/model_car.git model_car_3 && \
    source /opt/ros/kinetic/setup.bash && \
    cd ~/seat/model_car_3/catkin_ws && \
    rm -rf build devel && \
    catkin_make --only-pkg-with-deps odometry

RUN source ~/seat/model_car_3/catkin_ws/devel/setup.bash && \
    mkdir -p ~/seat/catkin_ws/src && \
    cd ~/seat/catkin_ws && \
    catkin_make

RUN source ~/seat/catkin_ws/devel/setup.bash && \
    roscd && \
    cd ../src && \
    GIT_SSL_NO_VERIFY=1 git clone https://gitlab.iri.upc.edu/seat_adc/seat_car_simulator.git && \
    rosdep install -i -y --from-paths seat_car_simulator && \
    roscd && \
    cd .. && \
    catkin_make --only-pkg-with-deps seat_car_simulator

RUN echo "source ~/seat/catkin_ws/devel/setup.bash" >> ~/.bashrc
