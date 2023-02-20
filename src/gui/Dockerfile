FROM neuraville/feagi-base-gui-prod
ARG INSTANCE_IP
ENV PATH /gui/node_modules/.bin:$PATH
ENV REACT_APP_INSTANCE_IP=$INSTANCE_IP
RUN echo "111 instance id is $INSTANCE_IP"
RUN echo "222 instance id is $REACT_APP_INSTANCE_IP"
COPY package.json package-lock.json ./
RUN npm install
COPY . ./
RUN npm run build
RUN echo "app instance id is $REACT_APP_INSTANCE_IP"
CMD ["serve", "-s", "build"]
