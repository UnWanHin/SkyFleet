# syntax=docker/dockerfile:1
# Build ns-3 inside the image: the final image never depends on a host bridge.
FROM registry.fedoraproject.org/fedora:41 AS ns3-build
RUN dnf -y install --setopt=install_weak_deps=False \
      cmake ninja-build gcc-c++ git curl tar python3 \
    && dnf clean all
ARG NS3_VERSION=ns-3.46
RUN curl --fail --location --retry 3 --output /tmp/ns3.tar.gz \
      "https://gitlab.com/nsnam/ns-3-dev/-/archive/${NS3_VERSION}/ns-3-dev-${NS3_VERSION}.tar.gz" \
    && mkdir -p /src/ns3 \
    && tar -xf /tmp/ns3.tar.gz -C /src/ns3 --strip-components=1 \
    && cmake -S /src/ns3 -B /src/ns3-build -G Ninja \
      -DNS3_EXAMPLES=OFF -DNS3_TESTS=OFF \
      -DNS3_ENABLED_MODULES='core;network;mobility;wifi;internet' \
      -DCMAKE_INSTALL_PREFIX=/opt/ns3 \
    && cmake --build /src/ns3-build --parallel 4 \
    && cmake --install /src/ns3-build
COPY ns3_bridge/ /src/skyfleet/ns3_bridge/
RUN cmake -S /src/skyfleet/ns3_bridge -B /src/skyfleet/bridge-build -G Ninja \
      -DCMAKE_PREFIX_PATH=/opt/ns3 \
    && cmake --build /src/skyfleet/bridge-build --parallel 4

FROM registry.fedoraproject.org/fedora:41
RUN dnf -y install --setopt=install_weak_deps=False \
      python3 python3-pip libstdc++ mesa-libGL mesa-libEGL glib2 libgomp \
    && dnf clean all \
    && useradd --uid 10001 --create-home skyfleet
COPY --from=ns3-build /opt/ns3 /opt/ns3
COPY --from=ns3-build /src/skyfleet/bridge-build/skyfleet-ns3-bridge /usr/local/bin/skyfleet-ns3-bridge
ENV LD_LIBRARY_PATH=/opt/ns3/lib64:/opt/ns3/lib \
    SKYFLEET_NS3_BRIDGE=/usr/local/bin/skyfleet-ns3-bridge \
    MUJOCO_GL=egl PYTHONUNBUFFERED=1
WORKDIR /app
RUN python3 -m pip install --no-cache-dir --prefix=/usr/local \
      'numpy==2.2.6' 'mujoco==3.3.7'
COPY --chown=skyfleet:skyfleet skyfleet/ /app/skyfleet/
COPY --chown=skyfleet:skyfleet flightlab/ /app/flightlab/
RUN chmod -R a+rX /app/skyfleet /app/flightlab \
    && mkdir -p /app/out && chown -R skyfleet:skyfleet /app/out
USER skyfleet
EXPOSE 8765
VOLUME ["/app/out"]
ENTRYPOINT ["python3", "-m", "skyfleet.cli", "live"]
CMD ["--config", "skyfleet/scenarios/ns3_mujoco_urban_multihop.json", \
     "--city", "skyfleet/scenarios/d5_urban_city.geojson", \
     "--host", "0.0.0.0", "--port", "8765", \
     "--viewer", "web", "--duration", "120", \
     "--tick-seconds", "0.1", "--log", "out/live/dispatch.jsonl"]
