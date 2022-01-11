#!/bin/bash

<<COMMENT
    Copyright 2021 @ AGEVE
   ---------------------------------------------------------
   Authors: Martí Bolet, Albert Arla
   Contact: support.idi@movvo.eu
COMMENT


# ====================
#      VARIABLES 
# ====================
# Python
PYTHON_V=3.8.0
ENV=agl

# Variables Globales
N_PROC=$(nproc)
THIS_PATH=` dirname "$(realpath $0)"`
ROOT_PATH="$(dirname "$THIS_PATH")"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
working_dir="${SCRIPT_DIR}/../../"
enviornment_exist=$(grep -c "export AGL_PATH" ~/.bashrc)
if [ ${enviornment_exist} -gt 0  ]; then
    working_dir_=$(echo $working_dir | sed 's_/_\\/_g')
    sed -i -e "s/\(^export AGL_PATH=\).*/\1${working_dir_}/" ~/.bashrc
else 
    echo "export AGL_PATH=${working_dir}" >> ~/.bashrc
fi;

enviornment_exist=$(grep -c "export AGL_PATH_ENV" ~/.bashrc)
if [ ${enviornment_exist} -gt 0  ]; then
    sed -i -e "s/\(^export AGL_PATH_ENV=\).*/\1${ENV}/" ~/.bashrc
else 
    echo "export AGL_PATH_ENV=${ENV}" >> ~/.bashrc
fi;

echo "alias ccb='colcon build --symlink-install'" >> ~/.bashrc
echo "alias roborun='ros2 launch agl_bringup start_launch.py'" >> ~/.bashrc

source ~/.bashrc

agl_working_dir="${THIS_PATH}"
echo ${agl_working_dir}
utils_working_dir="${ROOT_PATH}/ageve_utils"
interfaces_working_dir="${ROOT_PATH}/ageve_interfaces"

# Dependencies directory
cd ${agl_working_dir}
if [ ! -d "dependencies" ]; then
	mkdir dependencies;
fi;
dependencies="${agl_working_dir}/dependencies/"
cd ${dependencies}
git clone https://github.com/ageve/rplidar_ros.git

# COLORS
COLOR_YELLOW='\033[1;33m'
COLOR_RED='\033[1;31m'
COLOR_GREEN='\033[1;32m'
COLOR_BLUE='\033[1;34m'
NOCOLOR='\033[0m'

echo -e "\n${COLOR_BLUE}====================================="
echo "AGEVE @ 2021 --- INSTALACIÓN AGL"
echo -e "=====================================${NOCOLOR}"

# INSTALL ROS2
echo -e "\n${COLOR_BLUE}====================================="
echo "INSTALAR ROS2"
echo -e "=====================================${NOCOLOR}"
# SETUP LOCAL
sudo apt update && sudo apt install locales;
sudo locale-gen en_US en_US.UTF-8;
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8;
export LANG=en_US.UTF-8;

# SETUP SOURCES
sudo apt update && sudo apt install -y curl gnupg2 lsb-release;
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -;
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list';

sudo apt update;
sudo apt install -y ros-foxy-desktop
source /opt/ros/foxy/setup.bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
sudo apt install -y python3-colcon-common-extensions
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
sudo apt install -y ros-foxy-ament-cmake-python
sudo apt-get install -y openssh-server
sudo apt-get install -y  libssh-dev
sudo apt install -y ros-foxy-xacro
sudo apt install -y ros-foxy-navigation2
sudo apt install -y ros-foxy-nav2-bringup
sudo apt install -y python3-rosdep2
sudo apt-get install -y libserial-dev 

echo -e "\n${COLOR_BLUE}====================================="
echo "INSTALAR DEPENDENCIAS AGL ..."
echo -e "=====================================${NOCOLOR}"

# VSCtool
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
sudo apt-get update
sudo apt-get install -y python3-vcstool

# General
sudo apt update && sudo apt upgrade -y --no-install-recommends && sudo apt install -y \
	build-essential \
	pkg-config \
	cmake \
	git \
	wget \
	curl \
	unzip \
	libatlas-base-dev \
	libsuitesparse-dev \
	libgtk-3-dev \
	ffmpeg \
	libavcodec-dev \
	libavformat-dev \
	libavutil-dev \
	libswscale-dev \
	libavresample-dev \
	gfortran \
	libyaml-cpp-dev \
	libgoogle-glog-dev \
	libgflags-dev

# Database
sudo apt install -y \
	postgresql \
	pgadmin4 \

# Install dependencies of ROS
sudo apt install -y ros-foxy-gazebo-ros-pkgs \
		     ros-foxy-image-common \
		     ros-foxy-image-transport-plugins

echo "================================================"
echo " Preparando entorno trabajo python ..."
echo "================================================"

cd ~
if [ ! -d ".pyenv" ]; then

    echo "PYENV no encontrado, INSTALANDO ..."
	# Install dependencies
    sudo apt-get install -y make build-essential libssl-dev zlib1g-dev \
    libbz2-dev libreadline-dev libsqlite3-dev wget curl llvm libncurses5-dev \
    libncursesw5-dev xz-utils tk-dev libffi-dev liblzma-dev python-openssl

    # Install pyenv
    curl https://pyenv.run | bash

    # Path to pyenv
    echo '#===========PYENV CONFIG===========' >> ~/.bashrc
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc
    echo 'eval "$(pyenv init --path)"' >> ~/.bashrc
    echo '#==================================' >> ~/.bashrc
    
    eval "$(pyenv virtualenv-init -)"
    echo $SHELL

else
    echo "PYENV encontrado, no hace falta instalar"
fi;

# Install python version igual a PYTHON_V si no existe (-s)
/home/${USER}/.pyenv/bin/pyenv install ${PYTHON_V} -s
# Si existe lo dice y sigue
/home/${USER}/.pyenv/bin/pyenv virtualenv ${PYTHON_V} ${ENV}
/home/${USER}/.pyenv/bin/pyenv local ${ENV}
/home/${USER}/.pyenv/bin/pyenv global ${ENV}
# Upgrade pip
/home/$USER/.pyenv/versions/${ENV}/bin/pip install --upgrade pip

echo "====================================="
echo " Instalando dependencias Python ..."
echo "====================================="

/home/${USER}/.pyenv/versions/${ENV}/bin/pip install 	psycopg2-binary \
													 	mysql-connector \
														xacro \
														catkin_pkg \
														empy \
														lark \
														numpy \
														cryptography \
														bcrypt \
														PyYAML \
														datetime \
														opencv-python

echo -e "\n${COLOR_BLUE}======================================================"
echo "¡REQUISITOS INSTALADOS!"
echo -e "======================================================${NOCOLOR}"

# echo -e "\n${COLOR_BLUE}======================================================"
# echo "CONFIGURAR BASE DE DATOS"
# echo -e "======================================================${NOCOLOR}"
# while true; do
#     read -p "¿Configurar base de datos propia de ME-00?(y/n): " yn
#     case $yn in
#         [Yy]* ) 
# 		echo "==========================";
# 		echo "Configurando base de datos";
# 		echo "==========================";
# 		cd ${agl_working_dir}
# 		echo $agl_working_dir
# 		cd src/me00_database/utils;
# 		/home/${USER}/.pyenv/versions/${ENV}/bin/python createdb.py break;;
#         [Nn]* ) break;;
#         * ) echo "Responde y/n.";;
#     esac
# done

echo -e "${COLOR_GREEN}=====================================";
echo "INSTALANDO SERVICIOS"
echo -e "=====================================${NOCOLOR}";
# Copy services
sudo cp ${agl_working_dir}/services/*.service /etc/systemd/system/
sudo cp ${agl_working_dir}/services/*.timer /etc/systemd/system/

variable=$(echo ${agl_working_dir} | sed 's_/_\\/_g')
WorkingDirectory_service=$(echo ${THIS_PATH} | sed 's_/_\\/_g')

# Executable services

# PYTHON services

# # BASH services
# exec_agl=$(echo $(which bash) -c "'${ROOT_PATH}/me-00/bash/start_me00.sh'" | sed 's_/_\\/_g')  

# for filename_path in ${THIS_PATH}/services/*.service; do
#     filename=${filename_path##*/}
#     sudo sed -i -e "s/\(^WorkingDirectory=\).*/\1${WorkingDirectory_service}/" /etc/systemd/system/${filename}
#     sudo sed -i -e "s/\(^User=\).*/\1${USER}/" /etc/systemd/system/${filename}
#     sudo sed -i -e "s/\(^Environment=\).*/\1AGL_PATH=${variable}/" /etc/systemd/system/${filename}
#     if [ ${filename} == "start_me00.service" ]; then
#         sudo sed -i -e "s/\(^ExecStart=\).*/\1${exec_me00}/" /etc/systemd/system/${filename}
#         # Add PYTHON_ENV as a Environment varaible for bash script
#         sudo sed -i -e "s/\(^Environment=AGL_PATH=${variable}\).*/\1\nEnvironment=USER=${USER}/" /etc/systemd/system/${filename}
#         sudo sed -i -e "s/\(^Environment=AGL_PATH=${variable}\).*/\1\nEnvironment=AGL_PATH_ENV=${ENV}/" /etc/systemd/system/${filename}
#     fi;
#     # Enable service to autoexecute on startup
#     sudo systemctl enable ${filename}
#     echo 'Enable ->' ${filename}
# done

# Reload services
sudo systemctl daemon-reload

# Restart services once all is installed and enabled
for filename_path in ${THIS_PATH}/services/*.service; do
    filename=${filename_path##*/}  
    filename_short=$(echo "${filename}" | cut -f1 -d".")
    sudo systemctl restart ${filename_short}
    echo 'Restart ->' ${filename_short}
done


echo -e "\n${COLOR_BLUE}======================================================"
echo "COMPILACIÓN DEL CODIGO"
echo -e "======================================================${NOCOLOR}"

# Compile
echo "PARA COMPILAR NECESARIO CLONAR ageve_utils, ageve_interfaces, ageve_rviz y ageve_slamtoolbox!!"
cd ${working_dir}
colcon build --symlink-install

echo "source ${working_dir}/install.setup.bash"
source ${working_dir}/install/setup.bash



echo -e "\n${COLOR_BLUE}======================================================"
echo "¡INSTALACIÓN COMPLETA!"
echo -e "======================================================${NOCOLOR}"

