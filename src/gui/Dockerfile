FROM neuraville/feagi-base-gui-prod
ARG INSTANCE_IP
ENV PATH /gui/node_modules/.bin:$PATH
COPY package.json package-lock.json ./
RUN npm install

COPY public ./public
COPY src ./src
RUN npm run build

COPY ./env.sh .
COPY .env .

# Add bash
RUN apk add --no-cache bash
RUN chmod +x env.sh

# CMD ["serve", "-s", "build"]
CMD ["/bin/bash", "-c", "./env.sh && cp ./env-config.js ./build/. && serve -s build"]
