from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware

from sockets import sio_app

app = FastAPI()

app.mount('/sockets', app=sio_app)

# Interface Web
app.mount("/", StaticFiles(directory="../website/dist", html=True), name="app")


@app.exception_handler(404)
def not_found(request, exc):
    return FileResponse('../website/dist/index.html')
