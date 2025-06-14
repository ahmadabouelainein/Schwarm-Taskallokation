FROM python:3.9-slim
WORKDIR /ws

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

COPY entrypoint.sh /usr/local/bin/swarm-entrypoint
RUN chmod +x /usr/local/bin/swarm-entrypoint \
    && apt-get update -qq \
    && apt-get install -y --no-install-recommends dos2unix \
    && dos2unix /usr/local/bin/swarm-entrypoint

ENTRYPOINT ["/usr/local/bin/swarm-entrypoint"]
