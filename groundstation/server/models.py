from sqlalchemy import Column, ForeignKey, Integer, String, DateTime, Boolean, Float, LargeBinary
from sqlalchemy.orm import relationship
import base64

from database import Base, SessionLocal, engine


class Mission(Base):
    __tablename__ = "missions"

    id = Column(Integer, primary_key=True, index=True)
    start = Column(DateTime)
    end = Column(DateTime)
    is_sim = Column(Boolean)

    has_rover = Column(Boolean)
    has_drone = Column(Boolean)
    distance_rover = Column(Float)
    distance_drone = Column(Float)
    map_rover = Column(LargeBinary)
    map_drone = Column(LargeBinary)

    log_entries = relationship("LogEntry", back_populates="mission")

    def as_dict(self):
        m = {c.name: getattr(self, c.name) for c in self.__table__.columns}
        if self.map_rover is not None:
            m["map_rover"] = base64.b64encode(self.map_rover).decode("utf-8")
        if self.map_drone is not None:
            m["map_drone"] = base64.b64encode(self.map_drone).decode("utf-8")
        return m


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
