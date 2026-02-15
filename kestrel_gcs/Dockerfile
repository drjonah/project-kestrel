# Use an official Python runtime as a parent image
FROM python:3.13-slim

# Set the working directory in the container
WORKDIR /app

# Copy the requirements file into the container at /app
COPY requirements.txt .

# Install any needed packages specified in requirements.txt
# --no-cache-dir reduces image size
RUN pip install --no-cache-dir -r requirements.txt

# Copy the rest of the current directory contents into the container at /app
COPY . .

# Make port 80 available to the world outside this container
# Change 80 to whatever port your app uses (e.g., 5000 for Flask, 8000 for FastAPI)
EXPOSE 80

# Run app.py when the container launches
CMD ["python", "app.py"]