FROM python:3.10-slim-bookworm

LABEL org.opencontainers.image.title="AutoMapConverter"
LABEL org.opencontainers.image.description="Reproducible vector-map conversion and regression environment"

ARG ESMINI_VERSION=3.6.0
ARG INSTALL_ESMINI=true
ARG ASAM_QC_OPENDRIVE_VERSION=1.0.0
ARG DEBIAN_MIRROR=deb.debian.org
ARG PIP_INDEX_URL=https://pypi.org/simple

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    MPLBACKEND=Agg \
    AUTOMAP_CONVERTER_ASAM_QC_CMD=/opt/asam-qc/bin/qc_opendrive \
    PATH=/opt/esmini/bin:/opt/asam-qc/bin:${PATH}

RUN sed -i "s|http://deb.debian.org|https://${DEBIAN_MIRROR}|g" /etc/apt/sources.list.d/debian.sources \
    && apt-get -o Acquire::Retries=3 -o Acquire::https::Timeout=30 update \
    && apt-get install --yes --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        libboost-python-dev \
        libeigen3-dev \
        libfontconfig1 \
        libgeographiclib-dev \
        libgl1 \
        libgl1-mesa-dri \
        libgomp1 \
        libpugixml-dev \
        libx11-6 \
        libxcursor1 \
        libxi6 \
        libxinerama1 \
        libxrandr2 \
        osmium-tool \
        pkg-config \
        unzip \
    && rm -rf /var/lib/apt/lists/*

RUN if [ "$INSTALL_ESMINI" = "true" ]; then curl --fail --location --silent --show-error --connect-timeout 20 --max-time 300 \
        "https://github.com/esmini/esmini/releases/download/v${ESMINI_VERSION}/esmini-bin_Linux.zip" \
        --output /tmp/esmini.zip \
    && unzip -q /tmp/esmini.zip -d /tmp/esmini-dist \
    && mkdir -p /opt/esmini \
    && cp -a /tmp/esmini-dist/esmini/. /opt/esmini/ \
    && rm -rf /tmp/esmini.zip /tmp/esmini-dist \
    && test -x /opt/esmini/bin/odrplot \
    && test -x /opt/esmini/bin/odrviewer; fi

RUN python -m venv /opt/asam-qc \
    && /opt/asam-qc/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/asam-qc/bin/pip install --no-cache-dir "asam-qc-opendrive==${ASAM_QC_OPENDRIVE_VERSION}" \
    && /opt/asam-qc/bin/qc_opendrive --help >/dev/null

# Keep the checker on its own Python environment (it requires lxml < 6).
# The converter requires lxml >= 6, so its interpreter must not resolve from that venv.
ENV PATH=/opt/esmini/bin:/usr/local/bin:/usr/local/sbin:/usr/sbin:/usr/bin:/sbin:/bin
RUN ln -s /opt/asam-qc/bin/qc_opendrive /usr/local/bin/qc_opendrive

WORKDIR /workspace
COPY . /workspace
RUN python -m pip install --no-cache-dir --upgrade pip \
    && python -m pip install --no-cache-dir -e ".[dev]" \
    && python -c "import lanelet2; import automap_converter" \
    && python -m pip check \
    && /opt/asam-qc/bin/python -m pip check \
    && if command -v odrplot >/dev/null; then odrplot data/samples/opendrive/commonroad_straight_road.xodr /tmp/odrplot.csv && test -s /tmp/odrplot.csv && rm -f /tmp/odrplot.csv; fi \
    && osmium --version >/dev/null

CMD ["bash"]
