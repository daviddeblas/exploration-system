FROM node:18 AS vue
WORKDIR /app/website
COPY website/package*.json ./
RUN npm ci
COPY website ./
RUN npm run build

FROM python:3.11-slim
WORKDIR /app/server
COPY server/requirements.txt ./
RUN pip install -r requirements.txt
EXPOSE 8000

COPY --from=vue /app/website/dist /app/website/dist
COPY server ./

CMD ["uvicorn", "main:app", "--host", "0.0.0.0", "--port", "8000"]