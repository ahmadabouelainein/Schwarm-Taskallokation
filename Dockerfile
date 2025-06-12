FROM python:3.9-slim

WORKDIR /ws

COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY . .

# copy the entrypoint and make it executable
COPY entrypoint.sh /ws/entrypoint.sh
RUN chmod +x /ws/entrypoint.sh

ENTRYPOINT ["/ws/entrypoint.sh"]
# no default CMD, so entrypoint.sh with no args will open bash
