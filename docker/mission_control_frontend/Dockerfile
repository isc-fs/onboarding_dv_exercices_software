# --- build stage ---
FROM node:22-alpine AS builder
WORKDIR /app
COPY tools/mission_control/frontend/package*.json ./
RUN npm ci
COPY tools/mission_control/frontend/ .
RUN npm run build

# --- serve stage ---
FROM nginx:alpine
COPY --from=builder /app/dist /usr/share/nginx/html
COPY docker/mission_control_frontend/nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
