from fastapi import FastAPI, Depends, APIRouter
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from fastapi.middleware.cors import CORSMiddleware

from sockets import sio_app
from database import get_db
from sqlalchemy.orm import Session
from sqlalchemy import desc
import models


app = FastAPI()

app.mount('/sockets', app=sio_app)


api = FastAPI()
api.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


@api.get("/missions")
def missions(db: Session = Depends(get_db)):
    rows = db.query(models.Mission).filter(models.Mission.end != None).order_by(
        desc(models.Mission.id)).offset(0).limit(30).all()
    return list(map(lambda r: r.as_dict(), rows))


@api.get("/logs")
def logs(skip: int = 0, start_id: int = 0, mission: int = 0, db: Session = Depends(get_db)):
    current_mission = db.query(models.Mission).order_by(
        desc(models.Mission.id)).limit(1).one_or_none()
    if current_mission is None:
        return []
    query = db.query(models.LogEntry).order_by(desc(models.LogEntry.id))
    if mission:
        query = query.filter(models.LogEntry.mission_id == mission)
    else:
        query = query.filter(models.LogEntry.mission_id == current_mission.id)
    if start_id:
        query = query.filter(models.LogEntry.id <= start_id)
    return query.offset(skip).limit(30).all()


app.mount("/api", api)


# Interface Web
app.mount("/", StaticFiles(directory="../website/dist", html=True), name="app")


@app.exception_handler(404)
def not_found(request, exc):
    return FileResponse('../website/dist/index.html')
