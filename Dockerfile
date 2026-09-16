FROM python:3.10-slim-bookworm

LABEL org.opencontainers.image.title="AutoMapConverter"
LABEL org.opencontainers.image.description="Reproducible vector and raster map-conversion environment"

ARG ESMINI_VERSION=3.6.0
ARG ASAM_QC_OPENDRIVE_VERSION=1.0.0

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_DISABLE_PIP_VERSION_CHECK=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    MPLBACKEND=Agg \
    AUTOMAP_CONVERTER_ASAM_QC_CMD=/opt/asam-qc/bin/qc_opendrive \
    PATH=/opt/esmini/bin:/opt/asam-qc/bin:${PATH}

RUN apt-get update \
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

RUN curl --fail --location --silent --show-error \
        "https://github.com/esmini/esmini/releases/download/v${ESMINI_VERSION}/esmini-bin_Linux.zip" \
        --output /tmp/esmini.zip \
    && unzip -q /tmp/esmini.zip -d /tmp/esmini-dist \
    && mkdir -p /opt/esmini \
    && cp -a /tmp/esmini-dist/esmini/. /opt/esmini/ \
    && rm -rf /tmp/esmini.zip /tmp/esmini-dist \
    && test -x /opt/esmini/bin/odrplot \
    && test -x /opt/esmini/bin/odrviewer

RUN python -m venv /opt/asam-qc \
    && /opt/asam-qc/bin/pip install --no-cache-dir --upgrade pip \
    && /opt/asam-qc/bin/pip install --no-cache-dir "asam-qc-opendrive==${ASAM_QC_OPENDRIVE_VERSION}" \
    && /opt/asam-qc/bin/qc_opendrive --help >/dev/null

WORKDIR /workspace
COPY . /workspace
RUN python -m pip install --no-cache-dir --upgrade pip \
    && python -m pip install --no-cache-dir -e ".[dev]" \
    && python -c "import lanelet2; import automap_converter" \
    && odrplot data/samples/opendrive/commonroad_straight_road.xodr /tmp/odrplot.csv \
    && test -s /tmp/odrplot.csv \
    && rm -f /tmp/odrplot.csv \
    && osmium --version >/dev/null

CMD ["bash"]
