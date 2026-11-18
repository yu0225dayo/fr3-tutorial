# FR3 実機・シミュレーション外の作業用イメージ: ROS 2 Jazzy（Ubuntu 24.04, Python 3.12）+ MoveIt + RViz(ブラウザ表示)
# + 方策側 Python 3.11（/opt/policy-venv、conda isaaclab 環境の版に合わせたもの）。
# ROS 側(rclpy)と方策側は Python の版が違うので、ソケット（ZMQ）でつなぐ。
FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-moveit ros-jazzy-moveit-configs-utils ros-jazzy-moveit-ros-visualization \
      ros-jazzy-ros2-control ros-jazzy-ros2-controllers ros-jazzy-controller-manager \
      ros-jazzy-kinematics-interface-kdl ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-rmw-cyclonedds-cpp \
      python3-colcon-common-extensions python3-pip \
      tigervnc-standalone-server tigervnc-tools novnc websockify openbox x11-utils \
      iputils-ping net-tools curl ca-certificates git \
    && rm -rf /var/lib/apt/lists/*

# 方策側 Python 3.11（uv で入れる。OS の Python 3.12 とは別）
ENV UV_PYTHON_INSTALL_DIR=/opt/uv-python
RUN pip3 install --break-system-packages --no-cache-dir uv \
    && uv venv --python 3.11 /opt/policy-venv
COPY policy-requirements.txt overrides.txt /tmp/
# gym[classic-control] が pygame==2.1.0（ソースビルドで失敗）を要求するので、wheel のある版に差し替える
RUN uv pip install --python /opt/policy-venv/bin/python -r /tmp/policy-requirements.txt --override /tmp/overrides.txt \
    && chmod -R a+rX /opt/uv-python /opt/policy-venv

COPY start_gui.sh /usr/local/bin/start_gui.sh
RUN chmod +x /usr/local/bin/start_gui.sh
ENV POLICY_PY=/opt/policy-venv/bin/python
WORKDIR /work
