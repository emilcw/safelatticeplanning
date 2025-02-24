FROM gitlab.liu.se:5000/srg/srg/base:latest
ENV TZ=Europe/Stockholm
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

ENV QT_X11_NO_MITSHM=1

RUN git lfs install

RUN sudo rosdep init \
    && rosdep update
