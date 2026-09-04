# Native-arch Lichtblick build for IFSSIM.
#
# Why this exists:
#   * Upstream's CI only publishes linux/amd64 to ghcr.io. On Apple
#     Silicon hosts that forced the container under Rosetta, which
#     pinned ~1 CPU core on the host running the DV stack and the
#     UE5 sim. With the lid closed during a trackdrive run we'd see
#     LiDAR drop frames whenever the Lichtblick tab was open.
#   * Upstream's Dockerfile invokes `yarn run web:build:prod` (a
#     webpack production build) without bumping Node's heap. On
#     buildx with the default 4 GB build container ceiling that OOMs
#     mid-bundle. We need NODE_OPTIONS=--max-old-space-size=8192 to
#     give webpack room.
#
# Pinned via the LICHTBLICK_REF build arg so a Lichtblick upgrade
# is one number change here. Tag picked from
# https://github.com/lichtblick-suite/lichtblick/releases.

ARG LICHTBLICK_REF=v1.25.0

# ---------- Build stage ----------
# Pull the upstream source by git ref. Using a small alpine + git
# stage keeps the final image lean (no git in production).
FROM alpine/git:latest AS source
ARG LICHTBLICK_REF
WORKDIR /src
RUN git clone --depth 1 --branch ${LICHTBLICK_REF} \
    https://github.com/lichtblick-suite/lichtblick.git .

FROM node:22 AS build
WORKDIR /src
COPY --from=source /src ./
RUN corepack enable
RUN yarn install --immutable
# Bump Node heap for webpack's prod bundle; the default 4 GB OOMs on
# the main chunk after ~108 s of build time. 6 GB is the sweet spot
# on a Docker Desktop with the default 8 GB pool — leaves ~1.5 GB
# for the OS + webpack worker processes. If your Docker Desktop has
# more (Settings → Resources → Memory), bump this further; if less,
# the build will OOM here and you need to raise Docker's pool first.
RUN NODE_OPTIONS=--max-old-space-size=6144 yarn run web:build:prod

# ---------- Release stage ----------
# caddy:2.5.2-alpine is multi-arch (arm64 + amd64); building locally
# on Apple Silicon picks arm64 automatically — no `platform:` needed.
FROM caddy:2.5.2-alpine
WORKDIR /src
COPY --from=build /src/web/.webpack ./
EXPOSE 8080

# Same entrypoint upstream uses — patches a custom default-layout.json
# into index.html if one is bind-mounted at /lichtblick.
COPY <<EOF /entrypoint.sh
mkdir -p /lichtblick
touch /lichtblick/default-layout.json
index_html=\$(cat index.html)
replace_pattern='/*LICHTBLICK_SUITE_DEFAULT_LAYOUT_PLACEHOLDER*/'
replace_value=\$(cat /lichtblick/default-layout.json)
echo "\${index_html/"\$replace_pattern"/\$replace_value}" > index.html
exec "\$@"
EOF

ENTRYPOINT ["/bin/sh", "/entrypoint.sh"]
CMD ["caddy", "file-server", "--listen", ":8080"]
