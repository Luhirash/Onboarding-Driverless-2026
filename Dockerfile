# Usa a imagem oficial do ROS 2 Humble
FROM osrf/ros:humble-desktop

# Atualiza o sistema e instala dependências comuns para Python e ROS
RUN apt-get update && apt-get install -y \
    python3-pip \
    nano \
    git \
    && rm -rf /var/lib/apt/lists/*

# Define o diretório de trabalho padrão dentro do container
WORKDIR /driverless_ws

# Garante que o ambiente do ROS 2 seja carregado automaticamente
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
