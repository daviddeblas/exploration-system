import unittest
from datetime import datetime
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

from database import Base
from models import Mission, LogEntry


class TestModels(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.engine = create_engine('sqlite:///:memory:')
        Base.metadata.create_all(cls.engine)
        cls.Session = sessionmaker(bind=cls.engine)

    def test_mission(self):
        session = self.Session()

        mission = Mission(
            start=datetime.utcnow(),
            end=datetime.utcnow(),
            is_sim=True,
            has_rover=True,
            has_drone=True,
            distance_rover=100.0,
            distance_drone=50.0
        )

        session.add(mission)
        session.commit()

        result = session.query(Mission).first()
        self.assertIsNotNone(result)
        self.assertEqual(result.is_sim, True)
        self.assertEqual(result.has_rover, True)
        self.assertEqual(result.has_drone, True)
        self.assertEqual(result.distance_rover, 100.0)
        self.assertEqual(result.distance_drone, 50.0)

    def test_log_entry(self):
        session = self.Session()

        mission = Mission(
            start=datetime.utcnow(),
            end=datetime.utcnow(),
            is_sim=True,
            has_rover=True,
            has_drone=True,
            distance_rover=100.0,
            distance_drone=50.0
        )

        session.add(mission)
        session.commit()

        log_entry = LogEntry(
            mission_id=mission.id,
            time=datetime.utcnow(),
            robot="Rover",
            category="Navigation",
            data="Moved 10 meters forward"
        )

        session.add(log_entry)
        session.commit()

        result = session.query(LogEntry).first()
        self.assertIsNotNone(result)
        self.assertEqual(result.robot, "Rover")
        self.assertEqual(result.category, "Navigation")
        self.assertEqual(result.data, "Moved 10 meters forward")


if __name__ == "__main__":
    unittest.main()
