
FROM python:3.11-slim
WORKDIR /app
COPY requirements.txt ./
RUN apt update && apt-get install -y libgl1 libglib2.0-0 
RUN pip install -r requirements.txt

COPY . ./

CMD ["python", "main.py"]