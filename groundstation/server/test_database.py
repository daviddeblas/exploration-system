import unittest
from sqlalchemy import text
from database import create_engine, SessionLocal, Base
from sqlalchemy.orm import sessionmaker
from models import Mission, LogEntry


class TestDatabase(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.engine = create_engine(
            "sqlite:///:memory:", connect_args={"check_same_thread": False})
        Base.metadata.create_all(cls.engine)
        cls.Session = sessionmaker(
            autocommit=False, autoflush=False, bind=cls.engine)

    def test_create_database(self):
        # Vérifie que la base de données a été créée
        with self.engine.connect() as connection:
            result = connection.execute(
                text("SELECT name FROM sqlite_master WHERE type='table'"))
            tables = [row[0] for row in result.fetchall()]
            self.assertIn("missions", tables)
            self.assertIn("logs", tables)

    def test_get_db_connection(self):
        # Teste la fonction get_db en vérifiant qu'elle renvoie un objet de session valide
        with SessionLocal() as session:
            self.assertIsNotNone(session)


if __name__ == "__main__":
    unittest.main()
