# Usa a imagem oficial do ROS 2 Humble
FROM osrf/ros:humble-desktop

# Atualiza o sistema e instala dependências comuns para Python e ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    nano \
    git \
    && rm -rf /var/lib/apt/lists/*

# (Opcional) Se não estiver usando o venv, recoloque o RUN pip install aqui:
# RUN pip3 install ultralytics pyserial opencv-python matplotlib

WORKDIR /driverless_ws
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc