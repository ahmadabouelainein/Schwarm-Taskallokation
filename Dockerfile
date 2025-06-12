# Use a lightweight Python base image
FROM python:3.9-slim

# Set work directory
WORKDIR /app

# Copy and install Python dependencies
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Copy rest of the application code
COPY . .

# Default command: list available scripts
CMD ["bash", "-c", "\
    echo 'Available entrypoints:' && \
    echo '  python model.py' && \
    echo '  python optimizer.py' && \
    bash"]
