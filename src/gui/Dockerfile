FROM neuraville/feagi-base-gui-prod
ENV PATH /gui/node_modules/.bin:$PATH
COPY package.json package-lock.json ./
RUN npm install
COPY . ./
RUN npm run build
CMD ["serve", "-s", "build"]
