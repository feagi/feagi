FROM neuraville/feagi-base-gui-prod
ENV PATH /gui/node_modules/.bin:$PATH
ENV REACT_APP_INSTANCE_IP $REACT_APP_INSTANCE_IP
COPY package.json package-lock.json ./
RUN npm install
COPY . ./
RUN npm run build
RUN echo "app instance id is $REACT_APP_INSTANCE_IP"
CMD ["serve", "-s", "build"]
