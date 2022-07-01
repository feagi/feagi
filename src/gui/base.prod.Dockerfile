FROM node:16.14.2-alpine
WORKDIR /gui
RUN npm install react-scripts@5.0.1 serve -g
