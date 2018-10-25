FROM ign-gazebo-base:latest

EXPOSE 11317
EXPOSE 11318

COPY ./docker/ign-gazebo_entrypoint.sh /ign-gazebo_entrypoint.sh
ENTRYPOINT ["/ign-gazebo_entrypoint.sh"]

COPY . ign-gazebo
RUN mkdir /ign-gazebo/build && \
    cd /ign-gazebo/build && \
    cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache && \
    make && \
    make install

