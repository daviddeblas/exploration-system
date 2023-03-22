from sqlalchemy import Column, ForeignKey, Integer, String, DateTime
from sqlalchemy.orm import relationship

import datetime

from database import Base, SessionLocal, engine


class Mission(Base):
    __tablename__ = "missions"

    id = Column(Integer, primary_key=True, index=True)
    start = Column(DateTime)

    log_entries = relationship("LogEntry", back_populates="mission")


class LogEntry(Base):
    __tablename__ = "logs"

    id = Column(Integer, primary_key=True, index=True)
    mission_id = Column(Integer, ForeignKey("missions.id"))

    time = Column(DateTime)

    robot = Column(String, index=True)
    category = Column(String)
    data = Column(String)

    mission = relationship("Mission", back_populates="log_entries")

    def as_dict(self):
        return {c.name: getattr(self, c.name) for c in self.__table__.columns}


db = SessionLocal()
Base.metadata.create_all(engine)
mission = Mission(start=datetime.datetime.now())
db.add(mission)
db.commit()
db.refresh(mission)
