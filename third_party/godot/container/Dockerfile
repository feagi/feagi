FROM neuraville/feagi-godot

COPY ./* /root/
RUN chmod a+x /root/*

ENV OPENBOX_ARGS='--startup "/root/FEAGI_initalize.sh"'
