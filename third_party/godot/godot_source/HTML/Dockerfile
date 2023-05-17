FROM golang:latest
# COPY . /usr/share/nginx/html
# COPY nginx.conf /etc/nginx/conf.d/default.conf
RUN mkdir -p html
COPY index.* html
RUN apt update
RUN apt install -y git
RUN git clone https://github.com/pipejakob/godot_web.git
WORKDIR godot_web
RUN make
RUN cp godot_web /usr/bin/
WORKDIR ../html
# CMD ["sh", "-c", "while :; do sleep 1; done"]
CMD ["sh", "-c", "godot_web --external --port 6081"]